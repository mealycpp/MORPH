#!/usr/bin/env python3
"""
CFD Post-Processing — Publication-quality velocity field plots
================================================================
Reads OpenFOAM results and generates:
  1. Top-down velocity slices (XY plane) for each config
  2. Side-view velocity slices (XZ plane) showing downwash structure
  3. Velocity perturbation plots with contour lines (like the reference papers)
  4. Inter-rotor velocity profiles along lines between adjacent EDFs
  5. Comparative summary across all 4 configurations

Usage:
    cd ~/cfd_morph
    python3 post_process.py

Requires: numpy, matplotlib
Reads from: config_X/, config_O/, config_T/, config_H/ OpenFOAM case directories
"""

import os, json, glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm

# EDF diameter for normalization
D = 0.070  # 70mm

CONFIGS = {
    'X': {'color': '#3b82f6', 'label': 'X-Config (baseline)'},
    'O': {'color': '#22c55e', 'label': 'O-Config (compact)'},
    'T': {'color': '#f59e0b', 'label': 'T-Config (asymmetric)'},
    'H': {'color': '#a78bfa', 'label': 'H-Config (wide stance)'},
}


def read_openfoam_field(case_dir, field_name, time='500'):
    """Read an OpenFOAM vector or scalar field from the latest time directory."""
    # Find the latest time directory
    time_dirs = sorted([d for d in os.listdir(case_dir) 
                       if os.path.isdir(os.path.join(case_dir, d)) and d.replace('.', '').isdigit()],
                      key=lambda x: float(x))
    if not time_dirs:
        print(f"  WARNING: No time directories found in {case_dir}")
        return None, None
    
    latest = time_dirs[-1]
    field_path = os.path.join(case_dir, latest, field_name)
    
    if not os.path.exists(field_path):
        print(f"  WARNING: {field_path} not found")
        return None, None
    
    print(f"  Reading {field_name} from time={latest}")
    
    with open(field_path, 'r') as f:
        content = f.read()
    
    # Determine if vector or scalar
    is_vector = 'volVectorField' in content
    
    # Find the internalField data
    # Look for the pattern: internalField nonuniform List<vector> N ( ... )
    if 'nonuniform' in content:
        # Find the start of the data
        idx = content.find('internalField')
        # Find the count
        count_start = content.find('List<', idx)
        count_end = content.find('\n', count_start)
        # Find opening paren of the data block
        idx = content.find('\n(', count_start)
        if idx == -1:
            idx = content.find('(\n', count_start)
        end_idx = content.rfind(')')
        # Get everything between outer parens
        data_block = content[idx:end_idx+1]
        
        if is_vector:
            # Extract all (x y z) groups using simple parsing
            values = []
            i = 0
            while i < len(data_block):
                # Find opening paren
                start = data_block.find('(', i)
                if start == -1:
                    break
                # Find closing paren
                end = data_block.find(')', start)
                if end == -1:
                    break
                inner = data_block[start+1:end].strip()
                parts = inner.split()
                if len(parts) == 3:
                    try:
                        values.append([float(parts[0]), float(parts[1]), float(parts[2])])
                    except ValueError:
                        pass
                i = end + 1
            return np.array(values), 'vector'
        else:
            # Parse scalar field — just grab all numbers
            lines = data_block.strip().strip('()').split('\n')
            values = []
            for line in lines:
                line = line.strip()
                if line:
                    try:
                        values.append(float(line))
                    except ValueError:
                        pass
            return np.array(values), 'scalar'
    elif 'uniform' in content:
        idx = content.find('internalField')
        line = content[idx:content.find(';', idx)]
        if is_vector:
            start = line.find('(')
            end = line.find(')')
            parts = line[start+1:end].split()
            val = [float(parts[0]), float(parts[1]), float(parts[2])]
            return np.array(val), 'uniform_vector'
        else:
            parts = line.split()
            val = float(parts[-1])
            return np.array([val]), 'uniform_scalar'
    
    return None, None


def read_mesh_centers(case_dir):
    """Read cell center coordinates by running postProcess or from polyMesh."""
    # Try to read from a writeCellCentres output
    cc_path = os.path.join(case_dir, '0', 'C')
    
    # If cell centers file doesn't exist, we need to generate it
    if not os.path.exists(cc_path):
        # Check latest time dir
        time_dirs = sorted([d for d in os.listdir(case_dir) 
                           if os.path.isdir(os.path.join(case_dir, d)) and d.replace('.', '').isdigit()],
                          key=lambda x: float(x))
        for td in time_dirs:
            cc_path = os.path.join(case_dir, td, 'C')
            if os.path.exists(cc_path):
                break
    
    if not os.path.exists(cc_path):
        print(f"  Cell centers not found. Run: cd {case_dir} && postProcess -func writeCellCentres -time 0")
        return None
    
    data, dtype = read_openfoam_field(case_dir, 'C', '0')
    return data


def generate_cell_centers_from_mesh(case_dir):
    """Generate cell centers from blockMeshDict parameters directly."""
    # From our blockMeshDict: domain is (-0.6,-0.6,-0.8) to (0.6,0.6,0.5), 60x60x65 cells
    nx, ny, nz = 60, 60, 65
    x = np.linspace(-0.6, 0.6, nx+1)
    y = np.linspace(-0.6, 0.6, ny+1)
    z = np.linspace(-0.8, 0.5, nz+1)
    
    # Cell centers are at midpoints
    xc = 0.5 * (x[:-1] + x[1:])
    yc = 0.5 * (y[:-1] + y[1:])
    zc = 0.5 * (z[:-1] + z[1:])
    
    return xc, yc, zc, nx, ny, nz


def extract_slice_data(case_dir, field_name='U'):
    """Extract velocity field from unstructured mesh and interpolate onto a regular grid."""
    from scipy.interpolate import griddata
    
    # Read cell centers - first try writeCellCentres output
    time_dirs = sorted([d for d in os.listdir(case_dir)
                       if os.path.isdir(os.path.join(case_dir, d)) and d.replace('.', '').isdigit()],
                      key=lambda x: float(x))
    latest = time_dirs[-1] if time_dirs else '0'
    
    # Try reading C (cell centers) from latest time dir, then 0
    C = None
    for t in [latest, '0']:
        c_path = os.path.join(case_dir, t, 'C')
        if os.path.exists(c_path):
            C, _ = read_openfoam_field(case_dir, 'C')
            if C is not None and len(C.shape) == 2:
                print(f"    Read {len(C)} cell centers from time={t}")
                break
            C = None
    
    # Read the velocity field
    U_raw, dtype = read_openfoam_field(case_dir, field_name)
    if U_raw is None:
        return None, None, None, None
    
    if C is None:
        print(f"    Cell centers not found. Running writeCellCentres automatically...")
        import subprocess
        result = subprocess.run(
            ['postProcess', '-func', 'writeCellCentres', '-latestTime'],
            cwd=case_dir, capture_output=True, text=True
        )
        if result.returncode == 0:
            # Try reading Cx, Cy, Cz (writeCellCentres outputs these as separate scalar fields)
            for t in [latest, '0']:
                cx_path = os.path.join(case_dir, t, 'Cx')
                if os.path.exists(cx_path):
                    print(f"    Reading Cx/Cy/Cz from time={t}")
                    Cx, _ = read_openfoam_field(case_dir, 'Cx')
                    Cy, _ = read_openfoam_field(case_dir, 'Cy')
                    Cz, _ = read_openfoam_field(case_dir, 'Cz')
                    if Cx is not None and Cy is not None and Cz is not None:
                        C = np.column_stack([Cx, Cy, Cz])
                        print(f"    Loaded {len(C)} cell centers from Cx/Cy/Cz")
                    break
            # Also try C as vector field
            if C is None:
                for t in [latest, '0']:
                    c_path = os.path.join(case_dir, t, 'C')
                    if os.path.exists(c_path):
                        C, _ = read_openfoam_field(case_dir, 'C')
                        if C is not None and len(C.shape) == 2:
                            print(f"    Loaded {len(C)} cell centers from C")
                        else:
                            C = None
                        break
        
        if C is None:
            print(f"    Fallback: generating approximate centers from mesh dimensions...")
            C = _generate_approximate_centers(len(U_raw), case_dir)
            if C is None:
                return None, None, None, None
    
    if len(C) != len(U_raw):
        print(f"    WARNING: C has {len(C)} points but U has {len(U_raw)} — trimming to min")
        n = min(len(C), len(U_raw))
        C = C[:n]
        U_raw = U_raw[:n]
    
    # Compute velocity magnitude
    U_mag = np.sqrt(U_raw[:, 0]**2 + U_raw[:, 1]**2 + U_raw[:, 2]**2)
    
    # Create regular grids for XY and XZ slices
    nx_grid, ny_grid, nz_grid = 200, 200, 200
    
    x_min, x_max = C[:, 0].min(), C[:, 0].max()
    y_min, y_max = C[:, 1].min(), C[:, 1].max()
    z_min, z_max = C[:, 2].min(), C[:, 2].max()
    
    xg = np.linspace(x_min, x_max, nx_grid)
    yg = np.linspace(y_min, y_max, ny_grid)
    zg = np.linspace(z_min, z_max, nz_grid)
    
    # XY slice at z = -0.05 (below actuator disks)
    z_slice = -0.05
    z_tol = 0.015  # grab cells within ±15mm of the slice plane
    mask_xy = np.abs(C[:, 2] - z_slice) < z_tol
    
    if mask_xy.sum() < 50:
        print(f"    WARNING: Only {mask_xy.sum()} cells near z={z_slice}. Widening tolerance.")
        z_tol = 0.03
        mask_xy = np.abs(C[:, 2] - z_slice) < z_tol
    
    print(f"    XY slice: {mask_xy.sum()} cells within z={z_slice}±{z_tol}")
    
    XY_grid_x, XY_grid_y = np.meshgrid(xg, yg)
    if mask_xy.sum() > 10:
        U_xy = griddata(C[mask_xy, :2], U_mag[mask_xy], (XY_grid_x, XY_grid_y), method='linear', fill_value=0)
    else:
        U_xy = np.zeros_like(XY_grid_x)
    
    # XZ slice at y = 0 (side view)
    y_slice = 0.0
    y_tol = 0.015
    mask_xz = np.abs(C[:, 1] - y_slice) < y_tol
    
    print(f"    XZ slice: {mask_xz.sum()} cells within y={y_slice}±{y_tol}")
    
    XZ_grid_x, XZ_grid_z = np.meshgrid(xg, zg)
    if mask_xz.sum() > 10:
        U_xz = griddata(
            np.column_stack([C[mask_xz, 0], C[mask_xz, 2]]),
            U_mag[mask_xz],
            (XZ_grid_x, XZ_grid_z), method='linear', fill_value=0
        )
    else:
        U_xz = np.zeros_like(XZ_grid_x)
    
    # Also extract raw U vectors for the XY slice (for perturbation plots)
    if mask_xy.sum() > 10:
        U_xy_vec = np.zeros((ny_grid, nx_grid, 3))
        for comp in range(3):
            U_xy_vec[:, :, comp] = griddata(C[mask_xy, :2], U_raw[mask_xy, comp],
                                            (XY_grid_x, XY_grid_y), method='linear', fill_value=0)
    else:
        U_xy_vec = np.zeros((ny_grid, nx_grid, 3))
    
    return {
        'xg': xg, 'yg': yg, 'zg': zg,
        'U_xy': U_xy,         # velocity magnitude on XY plane
        'U_xz': U_xz,         # velocity magnitude on XZ plane
        'U_xy_vec': U_xy_vec,  # velocity vectors on XY plane
        'XY_grid_x': XY_grid_x, 'XY_grid_y': XY_grid_y,
        'XZ_grid_x': XZ_grid_x, 'XZ_grid_z': XZ_grid_z,
    }


def _generate_approximate_centers(n_cells, case_dir):
    """Fallback: generate approximate cell centers assuming mostly-structured mesh."""
    # We know the domain bounds from blockMeshDict
    # With snappyHexMesh, cells are added near the drone body
    # This is very approximate — real writeCellCentres is much better
    print(f"    Approximate centers for {n_cells} cells (inaccurate, use writeCellCentres instead)")
    # Just create a random sampling in the domain — this won't produce great plots
    # but at least won't crash
    rng = np.random.default_rng(42)
    C = np.zeros((n_cells, 3))
    C[:, 0] = rng.uniform(-0.6, 0.6, n_cells)
    C[:, 1] = rng.uniform(-0.6, 0.6, n_cells)
    C[:, 2] = rng.uniform(-0.8, 0.5, n_cells)
    return C


def plot_xy_slice(ax, xc, yc, U_slice_mag, edf_positions, config_name, color, vmax=50):
    """Plot a top-down (XY) velocity magnitude slice."""
    X, Y = np.meshgrid(xc / D, yc / D)
    
    # Velocity magnitude
    im = ax.pcolormesh(X, Y, U_slice_mag, cmap='RdBu_r', vmin=0, vmax=vmax, shading='auto')
    
    # Contour lines at percentage levels of peak
    if U_slice_mag.max() > 1:
        levels = [0.01 * vmax, 0.05 * vmax, 0.10 * vmax, 0.25 * vmax]
        cs = ax.contour(X, Y, U_slice_mag, levels=levels, colors='k', 
                        linewidths=[0.5, 0.8, 1.0, 1.5], linestyles=[':', '--', '-.', '-'])
        ax.clabel(cs, fmt='%.0f m/s', fontsize=7)
    
    # Mark EDF positions
    for i, (ex, ey, _) in enumerate(edf_positions):
        circle = plt.Circle((ex/D, ey/D), 0.5, fill=False, color=color, linewidth=2)
        ax.add_patch(circle)
        ax.plot(ex/D, ey/D, '+', color=color, markersize=8, markeredgewidth=2)
    
    ax.set_xlabel('x / D', fontsize=11)
    ax.set_ylabel('y / D', fontsize=11)
    ax.set_title(f'{config_name}', fontsize=14, fontweight='bold', color=color)
    ax.set_aspect('equal')
    ax.set_xlim(-8, 8)
    ax.set_ylim(-8, 8)
    
    return im


def plot_xz_slice(ax, xc, zc, U_slice_mag, edf_x_positions, config_name, color, vmax=50):
    """Plot a side-view (XZ) velocity magnitude slice showing downwash."""
    X, Z = np.meshgrid(xc / D, zc / D)
    
    im = ax.pcolormesh(X, Z, U_slice_mag, cmap='RdBu_r', vmin=0, vmax=vmax, shading='auto')
    
    # Contour lines
    if U_slice_mag.max() > 1:
        levels = [0.01 * vmax, 0.05 * vmax, 0.10 * vmax, 0.25 * vmax]
        cs = ax.contour(X, Z, U_slice_mag, levels=levels, colors='k',
                        linewidths=[0.5, 0.8, 1.0, 1.5], linestyles=[':', '--', '-.', '-'])
        ax.clabel(cs, fmt='%.0f m/s', fontsize=7)
    
    # Mark EDF positions
    for ex in edf_x_positions:
        ax.axvline(ex/D, color=color, linewidth=1.5, alpha=0.6, linestyle='--')
        ax.plot(ex/D, 0, 'v', color=color, markersize=10)
    
    ax.set_xlabel('x / D', fontsize=11)
    ax.set_ylabel('z / D', fontsize=11)
    ax.set_title(f'{config_name} — Side View (XZ plane)', fontsize=14, fontweight='bold', color=color)
    ax.set_xlim(-8, 8)
    ax.set_ylim(-12, 4)
    
    return im


def plot_velocity_perturbation(ax, xc, yc, U_slice, edf_positions, config_name, color):
    """Plot velocity perturbation (|U| - |U_background|) with contour lines
    similar to the reference paper style."""
    X, Y = np.meshgrid(xc / D, yc / D)
    
    # Background velocity is ~0 (hover, no freestream)
    # So perturbation = |U| itself, but we can show it differently
    U_mag = U_slice
    U_ref = np.median(U_mag)  # background reference
    U_pert = U_mag - U_ref
    
    vmax = np.percentile(np.abs(U_pert), 99)
    
    # Diverging colormap centered at 0
    im = ax.pcolormesh(X, Y, U_pert, cmap='RdBu_r', vmin=-vmax, vmax=vmax, shading='auto')
    
    # Contour lines at percentage levels
    if vmax > 0.1:
        pct_levels = [0.01, 0.05, 0.10]
        for pct in pct_levels:
            level = pct * vmax
            cs_pos = ax.contour(X, Y, U_pert, levels=[level], colors='k',
                               linewidths=0.5 + pct*10, linestyles='-')
            cs_neg = ax.contour(X, Y, U_pert, levels=[-level], colors='k',
                               linewidths=0.5 + pct*10, linestyles='--')
    
    # EDF markers
    for i, (ex, ey, _) in enumerate(edf_positions):
        circle = plt.Circle((ex/D, ey/D), 0.5, fill=False, color='k', linewidth=2)
        ax.add_patch(circle)
    
    ax.set_xlabel('x / D', fontsize=11)
    ax.set_ylabel('y / D', fontsize=11)
    ax.set_title(f'{config_name} — Velocity Perturbation', fontsize=13, fontweight='bold', color=color)
    ax.set_aspect('equal')
    ax.set_xlim(-8, 8)
    ax.set_ylim(-8, 8)
    
    return im


def plot_interrotor_profiles(ax, xc, yc, U_slice_mag, edf_positions, config_name, color):
    """Extract and plot velocity profiles along lines between adjacent rotor pairs."""
    labels = ['FR', 'FL', 'RL', 'RR']
    
    for i in range(4):
        for j in range(i+1, 4):
            x0, y0, _ = edf_positions[i]
            x1, y1, _ = edf_positions[j]
            dist = np.sqrt((x1-x0)**2 + (y1-y0)**2)
            
            # Only plot close pairs (< 350mm)
            if dist > 0.35:
                continue
            
            # Sample along the line
            n_samples = 100
            xs = np.linspace(x0, x1, n_samples)
            ys = np.linspace(y0, y1, n_samples)
            
            # Interpolate from grid
            velocities = []
            for s in range(n_samples):
                ix = np.argmin(np.abs(xc - xs[s]))
                iy = np.argmin(np.abs(yc - ys[s]))
                velocities.append(U_slice_mag[iy, ix])
            
            # Normalize distance along line
            d_along = np.linspace(0, dist/D, n_samples)
            
            ax.plot(d_along, velocities, color=color, linewidth=2,
                   label=f'{labels[i]}-{labels[j]} ({dist*1000:.0f}mm)')
    
    ax.set_xlabel('Distance along inter-rotor line (x/D)', fontsize=11)
    ax.set_ylabel('|U| (m/s)', fontsize=11)
    ax.set_title(f'{config_name} — Inter-Rotor Velocity Profile', fontsize=13, fontweight='bold', color=color)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(base_dir, 'plots')
    os.makedirs(output_dir, exist_ok=True)
    
    print("=" * 60)
    print("  Morphing Drone CFD Post-Processing")
    print("=" * 60)
    
    all_data = {}
    
    for config_name in ['X', 'O', 'T', 'H']:
        case_dir = os.path.join(base_dir, f'config_{config_name}')
        meta_path = os.path.join(case_dir, 'config_meta.json')
        
        if not os.path.exists(meta_path):
            print(f"\n  SKIP {config_name}: case not found")
            continue
        
        print(f"\n  Processing {config_name}-config...")
        
        with open(meta_path) as f:
            meta = json.load(f)
        
        result = extract_slice_data(case_dir, 'U')
        if result is None:
            print(f"  ERROR: Could not read velocity field for {config_name}")
            continue
        
        all_data[config_name] = {
            'slices': result,
            'meta': meta,
            'positions': meta['positions'],
        }
    
    if not all_data:
        print("\nERROR: No data loaded. Make sure simpleFoam has completed.")
        print("Check: ls config_X/500/  (should contain U, p, k, omega, nut)")
        return
    
    print(f"\n  Loaded {len(all_data)} configurations: {list(all_data.keys())}")
    
    # ═══ FIGURE 1: Top-down velocity slices ═══
    print("\n  Generating Figure 1: Top-down velocity slices...")
    fig, axes = plt.subplots(2, 2, figsize=(16, 16))
    fig.suptitle('Downwash Velocity Field — Horizontal Slice at z = -0.05m',
                fontsize=16, fontweight='bold', y=0.95)
    for idx, (cn, data) in enumerate(all_data.items()):
        ax = axes[idx // 2][idx % 2]
        cfg = CONFIGS[cn]
        s = data['slices']
        im = plot_xy_slice(ax, s['xg'], s['yg'], s['U_xy'], data['positions'], cfg['label'], cfg['color'])
    plt.tight_layout(rect=[0, 0, 0.88, 0.93])
    cbar_ax = fig.add_axes([0.90, 0.15, 0.02, 0.65])
    fig.colorbar(im, cax=cbar_ax, label='|U| (m/s)')
    fig.savefig(os.path.join(output_dir, 'fig1_topdown_velocity.png'), dpi=200, bbox_inches='tight')
    print("    Saved: plots/fig1_topdown_velocity.png")
    
    # ═══ FIGURE 2: Side-view slices ═══
    print("  Generating Figure 2: Side-view velocity slices...")
    fig, axes = plt.subplots(2, 2, figsize=(16, 14))
    fig.suptitle('Downwash Structure — Vertical Slice at y = 0', fontsize=16, fontweight='bold', y=0.95)
    for idx, (cn, data) in enumerate(all_data.items()):
        ax = axes[idx // 2][idx % 2]
        cfg = CONFIGS[cn]
        s = data['slices']
        edf_x = [p[0] for p in data['positions']]
        im = plot_xz_slice(ax, s['xg'], s['zg'], s['U_xz'], edf_x, cfg['label'], cfg['color'])
    plt.tight_layout(rect=[0, 0, 0.88, 0.93])
    cbar_ax = fig.add_axes([0.90, 0.15, 0.02, 0.65])
    fig.colorbar(im, cax=cbar_ax, label='|U| (m/s)')
    fig.savefig(os.path.join(output_dir, 'fig2_sideview_velocity.png'), dpi=200, bbox_inches='tight')
    print("    Saved: plots/fig2_sideview_velocity.png")
    
    # ═══ FIGURE 3: Velocity perturbation ═══
    print("  Generating Figure 3: Velocity perturbation fields...")
    fig, axes = plt.subplots(2, 2, figsize=(16, 16))
    fig.suptitle('Velocity Perturbation from Ambient', fontsize=16, fontweight='bold', y=0.95)
    for idx, (cn, data) in enumerate(all_data.items()):
        ax = axes[idx // 2][idx % 2]
        cfg = CONFIGS[cn]
        s = data['slices']
        im = plot_velocity_perturbation(ax, s['xg'], s['yg'], s['U_xy'], data['positions'], cfg['label'], cfg['color'])
    plt.tight_layout(rect=[0, 0, 0.92, 0.93])
    fig.savefig(os.path.join(output_dir, 'fig3_perturbation.png'), dpi=200, bbox_inches='tight')
    print("    Saved: plots/fig3_perturbation.png")
    
    # ═══ FIGURE 4: Inter-rotor profiles ═══
    print("  Generating Figure 4: Inter-rotor velocity profiles...")
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Velocity Profile Between Adjacent Rotors', fontsize=16, fontweight='bold', y=0.95)
    for idx, (cn, data) in enumerate(all_data.items()):
        ax = axes[idx // 2][idx % 2]
        cfg = CONFIGS[cn]
        s = data['slices']
        plot_interrotor_profiles(ax, s['xg'], s['yg'], s['U_xy'], data['positions'], cfg['label'], cfg['color'])
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(os.path.join(output_dir, 'fig4_interrotor_profiles.png'), dpi=200, bbox_inches='tight')
    print("    Saved: plots/fig4_interrotor_profiles.png")
    
    # ═══ FIGURE 5: Summary bars ═══
    print("  Generating Figure 5: Summary comparison...")
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('Cross-Configuration Aerodynamic Comparison', fontsize=16, fontweight='bold')
    cns = list(all_data.keys())
    clrs = [CONFIGS[c]['color'] for c in cns]
    peaks = [all_data[c]['slices']['U_xy'].max() for c in cns]
    axes[0].bar(cns, peaks, color=clrs, edgecolor='black', linewidth=0.5)
    axes[0].set_ylabel('Peak |U| (m/s)', fontsize=12)
    axes[0].set_title('Peak Downwash Velocity', fontsize=13, fontweight='bold')
    for i, v in enumerate(peaks):
        axes[0].text(i, v+0.5, f'{v:.1f}', ha='center', fontsize=11, fontweight='bold')
    min_dists = []
    for cn in cns:
        pos = all_data[cn]['positions']
        md = min(np.sqrt((pos[i][0]-pos[j][0])**2+(pos[i][1]-pos[j][1])**2) for i in range(4) for j in range(i+1,4))
        min_dists.append(md*1000)
    axes[1].bar(cns, min_dists, color=clrs, edgecolor='black', linewidth=0.5)
    axes[1].set_ylabel('Min Distance (mm)', fontsize=12)
    axes[1].set_title('Min Rotor Separation', fontsize=13, fontweight='bold')
    axes[1].axhline(D*1000*4, color='red', linestyle='--', linewidth=1.5, label=f'4xD ({D*4*1000:.0f}mm)')
    axes[1].legend(fontsize=10)
    for i, v in enumerate(min_dists):
        axes[1].text(i, v+3, f'{v:.0f}mm\n({v/(D*1000):.1f}xD)', ha='center', fontsize=10)
    mid_vels = []
    for cn in cns:
        s = all_data[cn]['slices']
        pos = all_data[cn]['positions']
        dists = [(np.sqrt((pos[i][0]-pos[j][0])**2+(pos[i][1]-pos[j][1])**2), i, j) for i in range(4) for j in range(i+1,4)]
        _, pi, pj = min(dists)
        mx = (pos[pi][0]+pos[pj][0])/2
        my = (pos[pi][1]+pos[pj][1])/2
        ix = np.argmin(np.abs(s['xg']-mx))
        iy = np.argmin(np.abs(s['yg']-my))
        mid_vels.append(s['U_xy'][iy, ix])
    axes[2].bar(cns, mid_vels, color=clrs, edgecolor='black', linewidth=0.5)
    axes[2].set_ylabel('|U| at midpoint (m/s)', fontsize=12)
    axes[2].set_title('Velocity Between Closest Pair', fontsize=13, fontweight='bold')
    for i, v in enumerate(mid_vels):
        axes[2].text(i, v+0.2, f'{v:.1f}', ha='center', fontsize=11, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(os.path.join(output_dir, 'fig5_summary.png'), dpi=200, bbox_inches='tight')
    print("    Saved: plots/fig5_summary.png")
    
    # ═══ Numerical summary ═══
    print("\n" + "=" * 70)
    print("  NUMERICAL SUMMARY")
    print("=" * 70)
    print(f"  {'Config':<8} {'Peak':>10} {'MinSep':>10} {'Sep/D':>8} {'MidVel':>10} {'Level':>14}")
    for i, cn in enumerate(cns):
        sr = min_dists[i] / (D*1000)
        lvl = "NONE" if sr > 5 else "LOW" if sr > 4 else "MODERATE" if sr > 3 else "HIGH"
        print(f"  {cn:<8} {peaks[i]:>8.1f} m/s {min_dists[i]:>7.0f} mm {sr:>7.1f}x {mid_vels[i]:>8.1f} m/s {lvl:>14}")
    print(f"\n  All plots saved to: {output_dir}/")
    print("=" * 70)


if __name__ == '__main__':
    main()
