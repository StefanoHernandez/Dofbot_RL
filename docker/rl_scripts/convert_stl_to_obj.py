#!/usr/bin/env python3
"""
Convert DOFBOT STL meshes to OBJ format for MuJoCo compatibility
"""

import trimesh
import os


def convert_stl_to_obj(stl_dir, obj_dir):
    """Convert all STL files in directory to OBJ format"""

    # Create output directory if it doesn't exist
    os.makedirs(obj_dir, exist_ok=True)

    # Find all STL files
    stl_files = [f for f in os.listdir(stl_dir) if f.endswith('.STL') or f.endswith('.stl')]

    print(f"Found {len(stl_files)} STL files to convert")
    print()

    for stl_file in stl_files:
        stl_path = os.path.join(stl_dir, stl_file)
        obj_file = stl_file.replace('.STL', '.obj').replace('.stl', '.obj')
        obj_path = os.path.join(obj_dir, obj_file)

        try:
            # Load STL mesh
            mesh = trimesh.load(stl_path)

            # Export as OBJ
            mesh.export(obj_path)

            print(f"✓ {stl_file} -> {obj_file}")
            print(f"  Vertices: {len(mesh.vertices)}")
            print(f"  Faces: {len(mesh.faces)}")

        except Exception as e:
            print(f"✗ Failed to convert {stl_file}: {e}")

    print()
    print(f"Conversion complete! OBJ files saved to: {obj_dir}")


if __name__ == '__main__':
    stl_dir = "/root/dofbot_ws/src/dofbot_moveit/meshes"
    obj_dir = "/root/mujoco_models/meshes"

    print("Converting DOFBOT STL meshes to OBJ format...")
    print(f"Input:  {stl_dir}")
    print(f"Output: {obj_dir}")
    print()

    convert_stl_to_obj(stl_dir, obj_dir)
