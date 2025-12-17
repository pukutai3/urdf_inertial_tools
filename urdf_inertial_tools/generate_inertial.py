#!/usr/bin/env python3

import sys
import trimesh

MM_TO_M = 0.001

def main():
    if len(sys.argv) != 4:
        print("Usage: generate_inertial <model.stl> <mass> <g|kg>")
        sys.exit(1)

    stl_path = sys.argv[1]
    mass_input = float(sys.argv[2])
    unit = sys.argv[3]

    # Mass unit conversion
    if unit == "g":
        mass_kg = mass_input / 1000.0
    elif unit == "kg":
        mass_kg = mass_input
    else:
        raise ValueError("Unit must be 'g' or 'kg'")

    # Load STL (assumed mm)
    mesh = trimesh.load_mesh(stl_path)

    if not mesh.is_watertight:
        raise RuntimeError("STL mesh is not watertight")

    # Convert to meters
    mesh.apply_scale(MM_TO_M)

    # Compute density from mass and volume
    volume = mesh.volume  # m^3
    density = mass_kg / volume

    # Assign density
    mesh.density = density

    # Mass properties
    center = mesh.center_mass
    inertia = mesh.moment_inertia

    # Output URDF <inertial>
    print("<inertial>")
    print(f'  <origin xyz="{center[0]:.6f} {center[1]:.6f} {center[2]:.6f}" rpy="0 0 0"/>')
    print(f'  <mass value="{mass_kg:.6f}"/>')
    print("  <inertia")
    print(f'    ixx="{inertia[0][0]:.6f}" ixy="{inertia[0][1]:.6f}" ixz="{inertia[0][2]:.6f}"')
    print(f'    iyy="{inertia[1][1]:.6f}" iyz="{inertia[1][2]:.6f}"')
    print(f'    izz="{inertia[2][2]:.6f}"/>')
    print("</inertial>")

if __name__ == "__main__":
    main()

