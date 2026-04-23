#!/usr/bin/env python3
"""Merge coplanar adjacent triangles in a binary STL.

Finds groups of coplanar adjacent faces via trimesh.facets(),
extracts the boundary polygon of each group, re-triangulates
with fan triangulation (fewer faces, no internal seam edges),
and writes the cleaned mesh.  Falls back to keeping original
faces for concave or multi-loop boundaries.

Usage: merge_coplanar.py INPUT.stl OUTPUT.stl
"""

import sys
from collections import Counter

import numpy as np
import trimesh


def _walk_boundary(boundary_edges):
    """Order boundary edges into a closed vertex loop.  Returns [] on failure."""
    if not boundary_edges:
        return []

    adj = {}
    for a, b in boundary_edges:
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)

    if any(len(ns) != 2 for ns in adj.values()):
        return []

    start = boundary_edges[0][0]
    loop = [start]
    prev = None
    cur = start

    for _ in range(len(boundary_edges)):
        ns = adj[cur]
        nxt = ns[0] if ns[0] != prev else ns[1]
        if nxt == start:
            break
        loop.append(nxt)
        prev = cur
        cur = nxt

    if len(loop) != len(boundary_edges):
        return []

    return loop


def _fan_triangulate(loop, vertices, expected_normal):
    """Fan triangulation from loop[0].  Returns face list or None if concave."""
    v0 = loop[0]
    faces = []

    for i in range(1, len(loop) - 1):
        vi, vj = loop[i], loop[i + 1]
        e1 = vertices[vi] - vertices[v0]
        e2 = vertices[vj] - vertices[v0]
        n = np.cross(e1, e2)
        mag = np.linalg.norm(n)

        if mag < 1e-12:
            continue

        if np.dot(n / mag, expected_normal) < 0.5:
            return None

        faces.append([v0, vi, vj])

    return faces if faces else None


def merge_coplanar(input_path, output_path):
    mesh = trimesh.load(input_path)
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.dump(concatenate=True)

    original_count = len(mesh.faces)
    groups = [g for g in mesh.facets if len(g) >= 2]

    if not groups:
        mesh.export(output_path, file_type="stl")
        return 0

    in_group = set()
    for g in groups:
        in_group.update(g.tolist())

    new_faces = [mesh.faces[i].tolist() for i in range(original_count) if i not in in_group]

    merged_count = 0

    for group in groups:
        group_faces = mesh.faces[group]
        normal = mesh.face_normals[group[0]]

        edge_list = []
        for face in group_faces:
            for i in range(3):
                edge_list.append(tuple(sorted((face[i], face[(i + 1) % 3]))))

        boundary = [e for e, c in Counter(edge_list).items() if c == 1]
        loop = _walk_boundary(boundary)

        if len(loop) < 3:
            new_faces.extend(group_faces.tolist())
            continue

        fan = _fan_triangulate(loop, mesh.vertices, normal)

        if fan is not None:
            new_faces.extend(fan)
            merged_count += len(group_faces) - len(fan)
        else:
            new_faces.extend(group_faces.tolist())

    result = trimesh.Trimesh(
        vertices=mesh.vertices,
        faces=np.array(new_faces),
        process=True,
    )
    result.export(output_path, file_type="stl")
    return merged_count


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} INPUT.stl OUTPUT.stl", file=sys.stderr)
        sys.exit(1)

    removed = merge_coplanar(sys.argv[1], sys.argv[2])
    print(f"  coplanar merge: eliminated {removed} redundant faces")
