#!/usr/bin/env python3
"""
Convert DOFBOT STL meshes to OBJ format using numpy-stl
"""

from stl import mesh
import numpy as np
import os


def stl_to_obj(stl_path, obj_path):
    """Convert a single STL file to OBJ format"""

    # Load the STL file
    stl_mesh = mesh.Mesh.from_file(stl_path)

    # Get vertices and faces
    vertices = stl_mesh.vectors.reshape(-1, 3)

    # Remove duplicate vertices
    unique_vertices, inverse_indices = np.unique(vertices, axis=0, return_inverse=True)

    # Create faces from inverse indices
    faces = inverse_indices.reshape(-1, 3) + 1  # OBJ is 1-indexed

    # Write OBJ file
    with open(obj_path, 'w') as f:
        # Write header
        f.write(f"# OBJ file generated from {os.path.basename(stl_path)}\n")
        f.write(f"# Vertices: {len(unique_vertices)}\n")
        f.write(f"# Faces: {len(faces)}\n\n")

        # Write vertices
        for v in unique_vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

        # Write faces
        f.write("\n")
        for face in faces:
            f.write(f"f {face[0]} {face[1]} {face[2]}\n")

    return len(unique_vertices), len(faces)


def convert_all_stl_to_obj(stl_dir, obj_dir):
    """Convert all STL files in directory to OBJ format"""

    # Create output directory
    os.makedirs(obj_dir, exist_ok=True)

    # Find all STL files
    stl_files = [f for f in os.listdir(stl_dir) if f.upper().endswith('.STL')]

    print(f"Found {len(stl_files)} STL files to convert\n")

    for stl_file in sorted(stl_files):
        stl_path = os.path.join(stl_dir, stl_file)
        obj_file = stl_file.replace('.STL', '.obj').replace('.stl', '.obj')
        obj_path = os.path.join(obj_dir, obj_file)

        try:
            vertices_count, faces_count = stl_to_obj(stl_path, obj_path)
            print(f"✓ {stl_file} -> {obj_file}")
            print(f"  Vertices: {vertices_count}, Faces: {faces_count}")
        except Exception as e:
            print(f"✗ Failed to convert {stl_file}: {e}")

    print(f"\n✓ Conversion complete! OBJ files saved to: {obj_dir}")


if __name__ == '__main__':
    stl_dir = "/root/dofbot_ws/src/dofbot_moveit/meshes"
    obj_dir = "/root/mujoco_models/meshes"

    print("Converting DOFBOT STL meshes to OBJ format...")
    print(f"Input:  {stl_dir}")
    print(f"Output: {obj_dir}\n")

    convert_all_stl_to_obj(stl_dir, obj_dir)
