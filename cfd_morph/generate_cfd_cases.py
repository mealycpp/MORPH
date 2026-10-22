#!/usr/bin/env python3
"""
Morphing Drone CFD — Actuator Disk Analysis
=============================================
Generates OpenFOAM case directories for X, O, T, H configurations.
Each case uses actuatorDisk fvOptions to model the 4 EDFs as pressure-jump
disks, avoiding the need for rotating mesh.

Usage:
    python3 generate_cfd_cases.py        # generates all case dirs
    ./run_all.sh                          # meshes + solves all 4 configs

Output: velocity/pressure fields showing inter-rotor wake interaction
"""

import os, math, json, shutil
import numpy as np

# ═══ Drone geometry ═══
ARM_INNER = 0.159       # meters (servo midpoint)
ARM_OUTER = 0.159       # meters (servo to EDF)
BASE_HEADINGS_DEG = [45, 135, 225, 315]  # FR, FL, RL, RR
EDF_DIAMETER = 0.070    # 70mm EDF
EDF_RADIUS = EDF_DIAMETER / 2
DUCT_LENGTH = 0.060     # 60mm duct depth

# ═══ Flight conditions ═══
MASS = 2.5              # kg
G = 9.80665
HOVER_THRUST_TOTAL = MASS * G       # 24.52 N
HOVER_THRUST_PER = HOVER_THRUST_TOTAL / 4  # 6.13 N

# Air properties
RHO = 1.225             # kg/m³
NU = 1.5e-5             # kinematic viscosity m²/s

# ═══ Configurations ═══
CONFIGS = {
    'X': [0, 0, 0, 0],
    'O': [90, 90, 90, 90],
    'T': [-45, 45, 45, -45],
    'H': [45, -45, 45, -45],
}

def compute_edf_positions(servo_angles_deg):
    """Forward kinematics: returns list of (x, y, z) for each EDF center."""
    positions = []
    for i, base_deg in enumerate(BASE_HEADINGS_DEG):
        bh = math.radians(base_deg)
        sx = ARM_INNER * math.cos(bh)
        sy = ARM_INNER * math.sin(bh)
        oh = bh + math.radians(servo_angles_deg[i])
        ex = sx + ARM_OUTER * math.cos(oh)
        ey = sy + ARM_OUTER * math.sin(oh)
        positions.append((ex, ey, 0.0))  # z=0, EDFs in horizontal plane
    return positions


def compute_disk_params(thrust_N, disk_area):
    """
    Compute actuator disk pressure jump.
    From actuator disk theory:
        T = delta_p * A
        delta_p = T / A
    """
    delta_p = thrust_N / disk_area
    return delta_p


def generate_stl_cylinder(center, radius, length, n_facets=36):
    """Generate STL text for a vertical cylinder (duct) centered at (cx, cy, cz).
    The cylinder axis is along Z (vertical = thrust direction)."""
    cx, cy, cz = center
    z_top = cz + length / 2
    z_bot = cz - length / 2
    triangles = []

    for i in range(n_facets):
        a0 = 2 * math.pi * i / n_facets
        a1 = 2 * math.pi * (i + 1) / n_facets
        x0 = cx + radius * math.cos(a0)
        y0 = cy + radius * math.sin(a0)
        x1 = cx + radius * math.cos(a1)
        y1 = cy + radius * math.sin(a1)

        # Side faces (2 triangles per segment)
        nx = (math.cos(a0) + math.cos(a1)) / 2
        ny = (math.sin(a0) + math.sin(a1)) / 2
        nz = 0
        norm = math.sqrt(nx*nx + ny*ny)
        nx /= norm; ny /= norm

        triangles.append(((nx, ny, nz), [(x0, y0, z_bot), (x1, y1, z_bot), (x1, y1, z_top)]))
        triangles.append(((nx, ny, nz), [(x0, y0, z_bot), (x1, y1, z_top), (x0, y0, z_top)]))

        # Top cap
        triangles.append(((0, 0, 1), [(cx, cy, z_top), (x0, y0, z_top), (x1, y1, z_top)]))
        # Bottom cap
        triangles.append(((0, 0, -1), [(cx, cy, z_bot), (x1, y1, z_bot), (x0, y0, z_bot)]))

    lines = ["solid duct"]
    for (nx, ny, nz), verts in triangles:
        lines.append(f"  facet normal {nx:.6f} {ny:.6f} {nz:.6f}")
        lines.append("    outer loop")
        for (vx, vy, vz) in verts:
            lines.append(f"      vertex {vx:.6f} {vy:.6f} {vz:.6f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid duct")
    return "\n".join(lines)


def write_block_mesh_dict(case_dir):
    """Simple box domain large enough to contain the drone + wake."""
    content = """FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      blockMeshDict;
}

scale   1;

vertices
(
    (-0.6 -0.6 -0.8)
    ( 0.6 -0.6 -0.8)
    ( 0.6  0.6 -0.8)
    (-0.6  0.6 -0.8)
    (-0.6 -0.6  0.5)
    ( 0.6 -0.6  0.5)
    ( 0.6  0.6  0.5)
    (-0.6  0.6  0.5)
);

blocks
(
    hex (0 1 2 3 4 5 6 7) (60 60 65) simpleGrading (1 1 1)
);

edges
(
);

boundary
(
    inlet
    {
        type patch;
        faces
        (
            (4 5 6 7)
        );
    }
    outlet
    {
        type patch;
        faces
        (
            (0 3 2 1)
        );
    }
    walls
    {
        type wall;
        faces
        (
            (0 1 5 4)
            (2 3 7 6)
            (1 2 6 5)
            (0 4 7 3)
        );
    }
);
"""
    os.makedirs(os.path.join(case_dir, "system"), exist_ok=True)
    with open(os.path.join(case_dir, "system", "blockMeshDict"), "w") as f:
        f.write(content)


def write_fv_schemes(case_dir):
    content = """FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      fvSchemes;
}

ddtSchemes
{
    default         steadyState;
}

gradSchemes
{
    default         Gauss linear;
}

divSchemes
{
    default         none;
    div(phi,U)      bounded Gauss linearUpwind grad(U);
    div(phi,k)      bounded Gauss upwind;
    div(phi,omega)  bounded Gauss upwind;
    div((nuEff*dev2(T(grad(U))))) Gauss linear;
}

laplacianSchemes
{
    default         Gauss linear corrected;
}

interpolationSchemes
{
    default         linear;
}

snGradSchemes
{
    default         corrected;
}

wallDist
{
    method          meshWave;
}
"""
    with open(os.path.join(case_dir, "system", "fvSchemes"), "w") as f:
        f.write(content)


def write_fv_solution(case_dir):
    content = """FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      fvSolution;
}

solvers
{
    p
    {
        solver          GAMG;
        tolerance       1e-06;
        relTol          0.1;
        smoother        GaussSeidel;
    }

    "(U|k|omega)"
    {
        solver          smoothSolver;
        smoother        symGaussSeidel;
        tolerance       1e-06;
        relTol          0.1;
    }
}

SIMPLE
{
    nNonOrthogonalCorrectors 0;
    consistent      yes;

    residualControl
    {
        p               1e-4;
        U               1e-4;
        "(k|omega)"     1e-4;
    }
}

relaxationFactors
{
    fields
    {
        p               0.3;
    }
    equations
    {
        U               0.7;
        k               0.7;
        omega           0.7;
    }
}
"""
    with open(os.path.join(case_dir, "system", "fvSolution"), "w") as f:
        f.write(content)


def write_control_dict(case_dir, config_name):
    content = f"""FoamFile
{{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      controlDict;
}}

application     simpleFoam;

startFrom       startTime;

startTime       0;

stopAt          endTime;

endTime         500;

deltaT          1;

writeControl    timeStep;

writeInterval   100;

purgeWrite      3;

writeFormat     ascii;

writePrecision  8;

writeCompression off;

timeFormat      general;

timePrecision   6;

runTimeModifiable true;

functions
{{
    fieldAverage1
    {{
        type            fieldAverage;
        libs            (fieldFunctionObjects);
        writeControl    writeTime;
        fields
        (
            U
            {{
                mean        on;
                prime2Mean  on;
                base        time;
            }}
            p
            {{
                mean        on;
                prime2Mean  on;
                base        time;
            }}
        );
    }}
}}
"""
    with open(os.path.join(case_dir, "system", "controlDict"), "w") as f:
        f.write(content)


def write_fv_options(case_dir, edf_positions, thrusts):
    """Write fvOptions with vectorSemiImplicitSource per disk zone.
    This applies a constant body force in the -Z direction within each
    actuator disk cell zone, producing the same net thrust as the real EDF."""
    disk_area = math.pi * EDF_RADIUS ** 2
    disk_thickness = 0.02  # matches topoSet z-range (-0.01 to +0.01)

    content = """FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      fvOptions;
}
"""
    for i, (pos, thrust) in enumerate(zip(edf_positions, thrusts)):
        vol = disk_area * disk_thickness
        force_density = thrust / vol  # N/m³

        content += f"""
disk{i+1}
{{
    type            vectorSemiImplicitSource;
    active          true;

    vectorSemiImplicitSourceCoeffs
    {{
        selectionMode   cellZone;
        cellZone        disk{i+1}Zone;
        volumeMode      specific;

        sources
        {{
            U       ((0 0 {-force_density:.4f}) 0);
        }}
    }}
}}
"""

    with open(os.path.join(case_dir, "system", "fvOptions"), "w") as f:
        f.write(content)


def write_topo_set_dict(case_dir, edf_positions):
    """Define cylindrical cellZones for each actuator disk."""
    actions = []
    for i, (x, y, z) in enumerate(edf_positions):
        actions.append(f"""
    {{
        name    disk{i+1}Set;
        type    cellSet;
        action  new;
        source  cylinderToCell;
        sourceInfo
        {{
            p1      ({x:.6f} {y:.6f} -0.01);
            p2      ({x:.6f} {y:.6f}  0.01);
            radius  {EDF_RADIUS:.6f};
        }}
    }}
    {{
        name    disk{i+1}Zone;
        type    cellZoneSet;
        action  new;
        source  setToCellZone;
        sourceInfo
        {{
            set     disk{i+1}Set;
        }}
    }}""")

    content = f"""FoamFile
{{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      topoSetDict;
}}

actions
({"".join(actions)}
);
"""
    with open(os.path.join(case_dir, "system", "topoSetDict"), "w") as f:
        f.write(content)


def write_initial_conditions(case_dir):
    """Write 0/ directory with U, p, k, omega boundary conditions."""
    zero_dir = os.path.join(case_dir, "0")
    os.makedirs(zero_dir, exist_ok=True)

    # U (velocity)
    with open(os.path.join(zero_dir, "U"), "w") as f:
        f.write("""FoamFile
{
    version     2.0;
    format      ascii;
    class       volVectorField;
    object      U;
}

dimensions      [0 1 -1 0 0 0 0];

internalField   uniform (0 0 0);

boundaryField
{
    inlet
    {
        type            pressureInletOutletVelocity;
        value           uniform (0 0 0);
    }
    outlet
    {
        type            inletOutlet;
        inletValue      uniform (0 0 0);
        value           uniform (0 0 0);
    }
    walls
    {
        type            slip;
    }
}
""")

    # p (pressure)
    with open(os.path.join(zero_dir, "p"), "w") as f:
        f.write("""FoamFile
{
    version     2.0;
    format      ascii;
    class       volScalarField;
    object      p;
}

dimensions      [0 2 -2 0 0 0 0];

internalField   uniform 0;

boundaryField
{
    inlet
    {
        type            totalPressure;
        p0              uniform 0;
        U               U;
        phi             phi;
        rho             none;
        psi             none;
        gamma           1;
        value           uniform 0;
    }
    outlet
    {
        type            fixedValue;
        value           uniform 0;
    }
    walls
    {
        type            slip;
    }
}
""")

    # k (turbulent kinetic energy)
    with open(os.path.join(zero_dir, "k"), "w") as f:
        f.write("""FoamFile
{
    version     2.0;
    format      ascii;
    class       volScalarField;
    object      k;
}

dimensions      [0 2 -2 0 0 0 0];

internalField   uniform 0.1;

boundaryField
{
    inlet
    {
        type            fixedValue;
        value           uniform 0.1;
    }
    outlet
    {
        type            inletOutlet;
        inletValue      uniform 0.1;
        value           uniform 0.1;
    }
    walls
    {
        type            slip;
    }
}
""")

    # omega (specific dissipation rate)
    with open(os.path.join(zero_dir, "omega"), "w") as f:
        f.write("""FoamFile
{
    version     2.0;
    format      ascii;
    class       volScalarField;
    object      omega;
}

dimensions      [0 0 -1 0 0 0 0];

internalField   uniform 5;

boundaryField
{
    inlet
    {
        type            fixedValue;
        value           uniform 5;
    }
    outlet
    {
        type            inletOutlet;
        inletValue      uniform 5;
        value           uniform 5;
    }
    walls
    {
        type            slip;
    }
}
""")

    # nut (turbulent viscosity — required by kOmegaSST)
    with open(os.path.join(zero_dir, "nut"), "w") as f:
        f.write("""FoamFile
{
    version     2.0;
    format      ascii;
    class       volScalarField;
    object      nut;
}

dimensions      [0 2 -1 0 0 0 0];

internalField   uniform 0;

boundaryField
{
    inlet
    {
        type            calculated;
        value           uniform 0;
    }
    outlet
    {
        type            calculated;
        value           uniform 0;
    }
    walls
    {
        type            slip;
    }
}
""")


def write_transport_properties(case_dir):
    content = f"""FoamFile
{{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      transportProperties;
}}

transportModel  Newtonian;
nu              [0 2 -1 0 0 0 0] {NU};
"""
    os.makedirs(os.path.join(case_dir, "constant"), exist_ok=True)
    with open(os.path.join(case_dir, "constant", "transportProperties"), "w") as f:
        f.write(content)


def write_turbulence_properties(case_dir):
    content = """FoamFile
{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      turbulenceProperties;
}

simulationType  RAS;

RAS
{
    RASModel        kOmegaSST;
    turbulence      on;
    printCoeffs     on;
}
"""
    with open(os.path.join(case_dir, "constant", "turbulenceProperties"), "w") as f:
        f.write(content)


def solve_thrusts(servo_angles_deg):
    """Solve per-motor thrust using lstsq (same as flight controller)."""
    positions = compute_edf_positions(servo_angles_deg)
    A = np.array([
        [1, 1, 1, 1],
        [positions[0][0], positions[1][0], positions[2][0], positions[3][0]],
        [positions[0][1], positions[1][1], positions[2][1], positions[3][1]],
    ])
    b = np.array([HOVER_THRUST_TOTAL, 0, 0])
    T, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    return T


def generate_case(config_name, servo_angles_deg, base_dir):
    """Generate a complete OpenFOAM case directory for one configuration."""
    case_dir = os.path.join(base_dir, f"config_{config_name}")
    if os.path.exists(case_dir):
        shutil.rmtree(case_dir)

    positions = compute_edf_positions(servo_angles_deg)
    thrusts = solve_thrusts(servo_angles_deg)

    print(f"\n{'='*60}")
    print(f"  {config_name}-config: servos = {servo_angles_deg}")
    print(f"{'='*60}")
    labels = ['FR', 'FL', 'RL', 'RR']
    for i, (pos, thrust) in enumerate(zip(positions, thrusts)):
        dist_from_center = math.sqrt(pos[0]**2 + pos[1]**2)
        print(f"  {labels[i]}: pos=({pos[0]*1000:+7.1f}, {pos[1]*1000:+7.1f}) mm"
              f"  thrust={thrust:.2f} N  dist_center={dist_from_center*1000:.1f} mm")

    # Compute inter-rotor distances
    print(f"\n  Inter-rotor distances:")
    for i in range(4):
        for j in range(i+1, 4):
            d = math.sqrt((positions[i][0]-positions[j][0])**2 +
                          (positions[i][1]-positions[j][1])**2) * 1000
            print(f"    {labels[i]}-{labels[j]}: {d:.1f} mm ({d/70:.1f}x EDF diameter)")

    # Generate all case files
    os.makedirs(case_dir, exist_ok=True)
    write_block_mesh_dict(case_dir)
    write_fv_schemes(case_dir)
    write_fv_solution(case_dir)
    write_control_dict(case_dir, config_name)
    write_transport_properties(case_dir)
    write_turbulence_properties(case_dir)
    write_initial_conditions(case_dir)
    write_topo_set_dict(case_dir, positions)
    write_fv_options(case_dir, positions, thrusts)

    # Save metadata for post-processing
    meta = {
        "config": config_name,
        "servos": servo_angles_deg,
        "positions": positions,
        "thrusts": thrusts.tolist(),
        "edf_diameter_m": EDF_DIAMETER,
        "mass_kg": MASS,
        "hover_thrust_total_N": HOVER_THRUST_TOTAL,
    }
    with open(os.path.join(case_dir, "config_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    print(f"\n  Case directory: {case_dir}")
    return case_dir


def write_run_script(base_dir, config_names):
    """Write a bash script that meshes and runs all cases."""
    script = """#!/bin/bash
# Run all morphing drone CFD configurations
# Usage: ./run_all.sh

set -e

"""
    for name in config_names:
        case_dir = f"config_{name}"
        script += f"""
echo ""
echo "{'='*60}"
echo "  Running {name}-config"
echo "{'='*60}"
cd {case_dir}

# Generate base mesh
blockMesh

# Create cell zones for actuator disks
topoSet

# Solve
simpleFoam | tee log.simpleFoam

# Post-process: sample velocity along lines
postProcess -func "sampleDict" -latestTime || true

cd ..
"""

    script_path = os.path.join(base_dir, "run_all.sh")
    with open(script_path, "w") as f:
        f.write(script)
    os.chmod(script_path, 0o755)

    # Also write individual run scripts
    for name in config_names:
        case_dir = os.path.join(base_dir, f"config_{name}")
        ind_script = f"""#!/bin/bash
set -e
echo "Running {name}-config CFD..."
blockMesh
topoSet
simpleFoam | tee log.simpleFoam
echo "Done. Open in ParaView: paraview {name}.foam"
touch {name}.foam
"""
        script_path = os.path.join(case_dir, "run.sh")
        with open(script_path, "w") as f:
            f.write(ind_script)
        os.chmod(script_path, 0o755)

        # Create .foam file for ParaView
        with open(os.path.join(case_dir, f"{name}.foam"), "w") as f:
            pass  # empty file, ParaView just needs it to exist


def write_sample_dict(case_dir, edf_positions):
    """Write sampling lines to extract velocity profiles between EDF pairs."""
    lines = []
    labels = ['FR', 'FL', 'RL', 'RR']

    # Sample lines between all adjacent pairs
    for i in range(4):
        for j in range(i+1, 4):
            x0, y0, _ = edf_positions[i]
            x1, y1, _ = edf_positions[j]
            lines.append(f"""
        line_{labels[i]}_{labels[j]}
        {{
            type    lineUniform;
            axis    distance;
            start   ({x0:.6f} {y0:.6f} -0.1);
            end     ({x1:.6f} {y1:.6f} -0.1);
            nPoints 100;
        }}""")

    # Vertical line through each EDF (downwash profile)
    for i, (x, y, _) in enumerate(edf_positions):
        lines.append(f"""
        downwash_{labels[i]}
        {{
            type    lineUniform;
            axis    distance;
            start   ({x:.6f} {y:.6f}  0.3);
            end     ({x:.6f} {y:.6f} -0.6);
            nPoints 100;
        }}""")

    content = f"""FoamFile
{{
    version     2.0;
    format      ascii;
    class       dictionary;
    object      sampleDict;
}}

type            sets;
libs            (sampling);
writeControl    writeTime;

interpolationScheme cellPoint;
setFormat       raw;

fields          (U p);

sets
({"".join(lines)}
);
"""
    with open(os.path.join(case_dir, "system", "sampleDict"), "w") as f:
        f.write(content)


# ═══ MAIN ═══
if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))

    print("Morphing Drone CFD — Actuator Disk Analysis")
    print("=" * 60)
    print(f"Mass: {MASS} kg  |  Hover thrust: {HOVER_THRUST_TOTAL:.2f} N")
    print(f"EDF: 70mm  |  Arm: {ARM_INNER*1000:.0f}+{ARM_OUTER*1000:.0f} mm")
    print(f"Configs: {list(CONFIGS.keys())}")

    config_names = []
    for name, servos in CONFIGS.items():
        case_dir = generate_case(name, servos, base_dir)
        positions = compute_edf_positions(servos)
        write_sample_dict(case_dir, positions)
        config_names.append(name)

    write_run_script(base_dir, config_names)

    print("\n" + "=" * 60)
    print("All cases generated. To run:")
    print(f"  cd {base_dir}")
    print(f"  ./run_all.sh")
    print(f"\nOr run individually:")
    for name in config_names:
        print(f"  cd config_{name} && ./run.sh")
    print(f"\nView results in ParaView:")
    for name in config_names:
        print(f"  paraview config_{name}/{name}.foam")
    print("=" * 60)
