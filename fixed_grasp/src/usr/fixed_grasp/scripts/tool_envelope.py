"""Exact deduplication of the existing 46-opening CAD union; no simplification."""
import numpy as np
from geometry_msgs.msg import Point
from shape_msgs.msg import Mesh, MeshTriangle


def build_mesh_union(gripper, open_width_m, check=lambda: None):
    # Convert each original STL once. Mesh messages are immutable after assembly.
    mesh_by_link = {}
    for name, triangles in gripper.meshes.items():
        check()
        vertices, indices = np.unique(triangles['vertices'].reshape(-1, 3),
                                      axis=0, return_inverse=True)
        mesh_by_link[name] = Mesh(
            vertices=[Point(*map(float, p)) for p in vertices],
            triangles=[MeshTriangle(vertex_indices=list(map(int, ids)))
                       for ids in indices.reshape(-1, 3)])
    seen = {name: set() for name in mesh_by_link}
    meshes, transforms = [], []
    for width in np.linspace(0., open_width_m, 46):
        check()
        angle = gripper.at_width(float(width))['linkage_angle_rad']
        for name, mesh in mesh_by_link.items():
            transform = np.asarray(gripper.link(name, angle), dtype=np.float64)
            if transform.shape != (4, 4) or not np.isfinite(transform).all():
                raise ValueError('invalid gripper mesh transform: '+name)
            # Byte equality only: no rounding, tolerances, skipped openings or
            # removal of nonidentical overlapping geometry.
            key = transform.tobytes()
            if key in seen[name]:
                continue
            seen[name].add(key)
            meshes.append(mesh)
            transforms.append(transform.copy())
    counts = dict(tool_input_instances=46*len(mesh_by_link),
                  tool_unique_instances=len(meshes),
                  tool_mesh_conversions=len(mesh_by_link),
                  tool_input_triangles=46*sum(len(m.triangles) for m in mesh_by_link.values()),
                  tool_unique_triangles=sum(len(m.triangles) for m in meshes))
    return meshes, transforms, counts
