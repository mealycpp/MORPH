#!/usr/bin/env python3
"""
Generate drone body STL geometry for CFD simulation.
Creates: central hub + 4 two-segment arms + 4 duct shrouds
for each morphing configuration (X, O, T, H).

The STL is placed in each case's constant/triSurface/ directory
for use with snappyHexMesh.
"""

import math
import os
import numpy as np

ARM_INNER = 0.159
ARM_OUTER = 0.159
BASE_HEADINGS_DEG = [45, 135, 225, 315]
EDF_RADIUS = 0.035  # 70mm EDF
DUCT_LENGTH = 0.030  # 30mm tall duct shroud
ARM_WIDTH = 0.012    # 12mm arm cross-section
ARM_HEIGHT = 0.005   # 5mm arm thickness
HUB_SIZE = 0.030     # 30mm half-width (60mm total — just the central mounting plate)
HUB_HEIGHT = 0.008   # 8mm hub thickness

CONFIGS = {
    'X': [0, 0, 0, 0],
    'O': [90, 90, 90, 90],
    'T': [-45, 45, 45, -45],
    'H': [45, -45, 45, -45],
}


def box_triangles(cx, cy, cz, dx, dy, dz):
    """Generate 12 triangles for an axis-aligned box centered at (cx,cy,cz)
    with half-dimensions (dx,dy,dz)."""
    x0, x1 = cx - dx, cx + dx
    y0, y1 = cy - dy, cy + dy
    z0, z1 = cz - dz, cz + dz

    v = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),  # bottom
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),  # top
    ]
    # 6 faces, 2 triangles each
    faces = [
        (0,1,2, 0,0,-1), (0,2,3, 0,0,-1),  # bottom
        (4,6,5, 0,0,1),  (4,7,6, 0,0,1),   # top
        (0,4,5, 0,-1,0), (0,5,1, 0,-1,0),  # front
        (2,6,7, 0,1,0),  (2,7,3, 0,1,0),   # back
        (0,3,7, -1,0,0), (0,7,4, -1,0,0),  # left
        (1,5,6, 1,0,0),  (1,6,2, 1,0,0),   # right
    ]
    tris = []
    for f in faces:
        i0, i1, i2 = f[0], f[1], f[2]
        nx, ny, nz = f[3], f[4], f[5]
        tris.append(((nx, ny, nz), [v[i0], v[i1], v[i2]]))
    return tris


def rotated_box_triangles(cx, cy, cz, length, width, height, angle_rad):
    """Generate triangles for a box rotated around Z axis."""
    # Generate axis-aligned box at origin
    dx, dy, dz = length / 2, width / 2, height / 2
    tris = box_triangles(0, 0, 0, dx, dy, dz)

    # Rotate and translate
    ca, sa = math.cos(angle_rad), math.sin(angle_rad)
    result = []
    for (nx, ny, nz), verts in tris:
        new_verts = []
        for (vx, vy, vz) in verts:
            rx = vx * ca - vy * sa + cx
            ry = vx * sa + vy * ca + cy
            new_verts.append((rx, ry, vz + cz))
        rnx = nx * ca - ny * sa
        rny = nx * sa + ny * ca
        result.append(((rnx, rny, nz), new_verts))
    return result


def cylinder_triangles(cx, cy, cz, radius, height, n_facets=24):
    """Generate triangles for a hollow cylinder (tube/duct) — no end caps."""
    tris = []
    z_top = cz + height / 2
    z_bot = cz - height / 2
    wall_thickness = 0.003  # 3mm wall

    for ring, r in [(radius, 1), (radius - wall_thickness, -1)]:
        for i in range(n_facets):
            a0 = 2 * math.pi * i / n_facets
            a1 = 2 * math.pi * (i + 1) / n_facets
            x0 = cx + r * math.cos(a0)
            y0 = cy + r * math.sin(a0)
            x1 = cx + r * math.cos(a1)
            y1 = cy + r * math.sin(a1)

            nx = ring * (math.cos(a0) + math.cos(a1)) / 2
            ny = ring * (math.sin(a0) + math.sin(a1)) / 2
            norm = math.sqrt(nx*nx + ny*ny) or 1
            nx /= norm
            ny /= norm

            tris.append(((nx*ring, ny*ring, 0), [(x0, y0, z_bot), (x1, y1, z_bot), (x1, y1, z_top)]))
            tris.append(((nx*ring, ny*ring, 0), [(x0, y0, z_bot), (x1, y1, z_top), (x0, y0, z_top)]))

    # Top and bottom annular rings
    for i in range(n_facets):
        a0 = 2 * math.pi * i / n_facets
        a1 = 2 * math.pi * (i + 1) / n_facets
        ro = radius
        ri = radius - wall_thickness
        for z, nz_dir in [(z_top, 1), (z_bot, -1)]:
            ox0 = cx + ro * math.cos(a0)
            oy0 = cy + ro * math.sin(a0)
            ox1 = cx + ro * math.cos(a1)
            oy1 = cy + ro * math.sin(a1)
            ix0 = cx + ri * math.cos(a0)
            iy0 = cy + ri * math.sin(a0)
            ix1 = cx + ri * math.cos(a1)
            iy1 = cy + ri * math.sin(a1)
            tris.append(((0, 0, nz_dir), [(ox0, oy0, z), (ox1, oy1, z), (ix1, iy1, z)]))
            tris.append(((0, 0, nz_dir), [(ox0, oy0, z), (ix1, iy1, z), (ix0, iy0, z)]))

    return tris


def generate_drone_stl(servo_angles_deg, config_name):
    """Generate complete drone STL for one configuration."""
    all_tris = []

    # Central hub (square platform)
    all_tris.extend(box_triangles(0, 0, 0, HUB_SIZE, HUB_SIZE, HUB_HEIGHT))

    for i, base_deg in enumerate(BASE_HEADINGS_DEG):
        bh = math.radians(base_deg)
        servo_rad = math.radians(servo_angles_deg[i])

        # Inner arm segment: from hub edge to servo
        inner_cx = (ARM_INNER / 2) * math.cos(bh)
        inner_cy = (ARM_INNER / 2) * math.sin(bh)
        all_tris.extend(rotated_box_triangles(
            inner_cx, inner_cy, 0,
            ARM_INNER, ARM_WIDTH, ARM_HEIGHT, bh
        ))

        # Servo joint position
        sx = ARM_INNER * math.cos(bh)
        sy = ARM_INNER * math.sin(bh)

        # Servo box (slightly larger than arm)
        all_tris.extend(rotated_box_triangles(
            sx, sy, 0,
            0.020, 0.020, 0.012, bh
        ))

        # Outer arm segment: from servo to EDF
        outer_heading = bh + servo_rad
        outer_cx = sx + (ARM_OUTER / 2) * math.cos(outer_heading)
        outer_cy = sy + (ARM_OUTER / 2) * math.sin(outer_heading)
        all_tris.extend(rotated_box_triangles(
            outer_cx, outer_cy, 0,
            ARM_OUTER, ARM_WIDTH, ARM_HEIGHT, outer_heading
        ))

        # EDF duct at arm tip — positioned above the actuator disk zone (z=-0.01 to +0.01)
        # so the duct doesn't block the momentum source
        edf_x = sx + ARM_OUTER * math.cos(outer_heading)
        edf_y = sy + ARM_OUTER * math.sin(outer_heading)
        all_tris.extend(cylinder_triangles(
            edf_x, edf_y, 0.02,
            EDF_RADIUS, DUCT_LENGTH
        ))

    # Write STL
    lines = [f"solid drone_{config_name}"]
    for (nx, ny, nz), verts in all_tris:
        lines.append(f"  facet normal {nx:.6f} {ny:.6f} {nz:.6f}")
        lines.append("    outer loop")
        for (vx, vy, vz) in verts:
            lines.append(f"      vertex {vx:.6f} {vy:.6f} {vz:.6f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid drone_{config_name}")

    return "\n".join(lines)


def write_snappy_hex_mesh_dict(case_dir, config_name):
    """Write snappyHexMeshDict to refine mesh around the drone body."""
    content = f"""FoamFile
{{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      snappyHexMeshDict;
}}

castellatedMesh true;
snap            true;
addLayers       false;

geometry
{{
    drone_{config_name}.stl
    {{
        type    triSurfaceMesh;
        name    droneBody;
    }}
}}

castellatedMeshControls
{{
    maxLocalCells       100000;
    maxGlobalCells      1000000;
    minRefinementCells  10;
    maxLoadUnbalance    0.10;
    nCellsBetweenLevels 3;
    resolveFeatureAngle 30;

    features
    (
        {{
            file    "drone_{config_name}.eMesh";
            level   2;
        }}
    );

    refinementSurfaces
    {{
        droneBody
        {{
            level   (2 3);
        }}
    }}

    refinementRegions
    {{
    }}

    locationInMesh (0.3 0.3 0.3);
    allowFreeStandingZoneFaces true;
}}

snapControls
{{
    nSmoothPatch    3;
    tolerance       2.0;
    nSolveIter      100;
    nRelaxIter      5;
    nFeatureSnapIter 10;
}}

addLayersControls
{{
    relativeSizes   true;
    layers
    {{
    }}
    expansionRatio  1.0;
    finalLayerThickness 0.3;
    minThickness    0.1;
    nGrow           0;
    featureAngle    60;
    nRelaxIter      3;
    nSmoothSurfaceNormals 1;
    nSmoothNormals  3;
    nSmoothThickness 10;
    maxFaceThicknessRatio 0.5;
    maxThicknessToMedialRatio 0.3;
    minMedialAxisAngle 90;
    nBufferCellsNoExtrude 0;
    nLayerIter      50;
}}

meshQualityControls
{{
    maxNonOrtho     65;
    maxBoundarySkewness 20;
    maxInternalSkewness 4;
    maxConcave      80;
    minVol          1e-13;
    minTetQuality   -1e30;
    minArea         -1;
    minTwist        0.02;
    minDeterminant  0.001;
    minFaceWeight   0.05;
    minVolRatio     0.01;
    minTriangleTwist -1;
    nSmoothScale    4;
    errorReduction  0.75;
}}

mergeTolerance  1e-6;
"""
    with open(os.path.join(case_dir, "system", "snappyHexMeshDict"), "w") as f:
        f.write(content)


def update_run_script(case_dir, config_name):
    """Update the run script to include snappyHexMesh step."""
    script = f"""#!/bin/bash
set -e
echo "Running {config_name}-config CFD with drone geometry..."

# Base mesh
blockMesh

# Extract surface features for snapping
surfaceFeatureExtract

# Refine mesh around drone body
snappyHexMesh -overwrite

# Create actuator disk zones
topoSet

# Solve
simpleFoam | tee log.simpleFoam

echo "Done. Open in ParaView: paraview {config_name}.foam"
touch {config_name}.foam
"""
    script_path = os.path.join(case_dir, "run.sh")
    with open(script_path, "w") as f:
        f.write(script)
    os.chmod(script_path, 0o755)


def write_surface_feature_extract_dict(case_dir, config_name):
    """Write surfaceFeatureExtractDict for edge detection."""
    content = f"""FoamFile
{{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      surfaceFeatureExtractDict;
}}

drone_{config_name}.stl
{{
    extractionMethod    extractFromSurface;

    extractFromSurfaceCoeffs
    {{
        includedAngle   150;
    }}

    writeObj    yes;
}}
"""
    with open(os.path.join(case_dir, "system", "surfaceFeatureExtractDict"), "w") as f:
        f.write(content)


def update_boundary_for_drone(case_dir):
    """Add droneBody boundary condition to all field files."""
    zero_dir = os.path.join(case_dir, "0")

    # U: no-slip wall on drone body
    u_path = os.path.join(zero_dir, "U")
    with open(u_path, 'r') as f:
        content = f.read()
    if 'droneBody' not in content:
        content = content.replace(
            '    walls\n    {\n        type            slip;\n    }\n}',
            '    walls\n    {\n        type            slip;\n    }\n    droneBody\n    {\n        type            noSlip;\n    }\n}'
        )
        with open(u_path, 'w') as f:
            f.write(content)

    # p: zeroGradient on drone body
    for field in ['p', 'k', 'omega', 'nut']:
        fpath = os.path.join(zero_dir, field)
        with open(fpath, 'r') as f:
            content = f.read()
        if 'droneBody' not in content:
            if field == 'nut':
                bc = '    droneBody\n    {\n        type            nutkWallFunction;\n        value           uniform 0;\n    }'
            elif field == 'k':
                bc = '    droneBody\n    {\n        type            kqRWallFunction;\n        value           uniform 0.1;\n    }'
            elif field == 'omega':
                bc = '    droneBody\n    {\n        type            omegaWallFunction;\n        value           uniform 5;\n    }'
            else:
                bc = '    droneBody\n    {\n        type            zeroGradient;\n    }'

            content = content.replace(
                '    walls\n    {\n        type            slip;\n    }\n}',
                '    walls\n    {\n        type            slip;\n    }\n' + bc + '\n}'
            )
            with open(fpath, 'w') as f:
                f.write(content)


# ═══ MAIN ═══
if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))

    print("Generating drone body STL for each configuration...")
    print("=" * 60)

    for config_name, servos in CONFIGS.items():
        case_dir = os.path.join(base_dir, f"config_{config_name}")
        if not os.path.exists(case_dir):
            print(f"  SKIP {config_name}: case directory not found")
            continue

        print(f"\n  {config_name}-config: servos = {servos}")

        # Generate STL
        stl_content = generate_drone_stl(servos, config_name)
        tri_dir = os.path.join(case_dir, "constant", "triSurface")
        os.makedirs(tri_dir, exist_ok=True)
        stl_path = os.path.join(tri_dir, f"drone_{config_name}.stl")
        with open(stl_path, 'w') as f:
            f.write(stl_content)

        tri_count = stl_content.count("facet normal")
        print(f"    STL: {tri_count} triangles → {stl_path}")

        # Write snappyHexMesh config
        write_snappy_hex_mesh_dict(case_dir, config_name)
        print(f"    snappyHexMeshDict written")

        # Write surfaceFeatureExtract config
        write_surface_feature_extract_dict(case_dir, config_name)
        print(f"    surfaceFeatureExtractDict written")

        # Update boundary conditions for drone body walls
        update_boundary_for_drone(case_dir)
        print(f"    Boundary conditions updated (noSlip on droneBody)")

        # Update run script
        update_run_script(case_dir, config_name)
        print(f"    run.sh updated with snappyHexMesh step")

    print("\n" + "=" * 60)
    print("To run with drone geometry:")
    print("  cd config_X && ./run.sh")
    print("\nThe mesh will have ~500K-800K cells (vs 234K without geometry).")
    print("Run time increases to ~5-10 minutes per config.")
    print("=" * 60)
