"""Fair end-to-end COMPAS predicate comparison on a closed torus fixture.

Both workloads start from the same already-constructed COMPAS Mesh. The COMPAS
workload calls both public predicates. The Forge workload includes Python
topology packing, FFI and both Rust predicates; no cache is hidden.
"""

import math

import pyperf

from compas.datastructures import Mesh

import compas_forge as forge


def torus_mesh(major_segments=128, minor_segments=128):
    vertices = []
    faces = []
    major_radius = 2.0
    minor_radius = 0.5
    for i in range(major_segments):
        u = 2.0 * math.pi * i / major_segments
        for j in range(minor_segments):
            v = 2.0 * math.pi * j / minor_segments
            radius = major_radius + minor_radius * math.cos(v)
            vertices.append(
                [radius * math.cos(u), radius * math.sin(u), minor_radius * math.sin(v)]
            )
    for i in range(major_segments):
        next_i = (i + 1) % major_segments
        for j in range(minor_segments):
            next_j = (j + 1) % minor_segments
            a = i * minor_segments + j
            b = next_i * minor_segments + j
            c = next_i * minor_segments + next_j
            d = i * minor_segments + next_j
            faces.append([a, b, c, d])
    return Mesh.from_vertices_and_faces(vertices, faces)


MESH = torus_mesh()
VERTEX_COUNT, INDICES, OFFSETS = forge.compas_mesh_to_topology_buffers(MESH)


def compas_closed_and_manifold():
    return MESH.is_closed(), MESH.is_manifold()


def forge_closed_and_manifold():
    result = forge.mesh_topology_predicates(MESH)
    return result["is_closed"], result["is_manifold"]


def forge_prepacked_topology_kernel():
    result = forge.topology_predicates_buffers(VERTEX_COUNT, INDICES, OFFSETS)
    return result["is_closed"], result["is_manifold"]


if __name__ == "__main__":
    expected = compas_closed_and_manifold()
    observed = forge_closed_and_manifold()
    prepacked = forge_prepacked_topology_kernel()
    if expected != (True, True) or observed != expected or prepacked != expected:
        raise AssertionError(
            f"correctness gate failed: COMPAS={expected}, Forge={observed}, "
            f"prepacked={prepacked}"
        )

    runner = pyperf.Runner()
    runner.metadata["fixture"] = "closed 128x128 quad torus"
    runner.metadata["vertices"] = MESH.number_of_vertices()
    runner.metadata["faces"] = MESH.number_of_faces()
    runner.metadata["forge_scope"] = "Python packing + FFI + Rust closed/manifold predicates"
    runner.metadata["compas_scope"] = "public Mesh.is_closed + Mesh.is_manifold"
    runner.bench_func("compas-closed-and-manifold", compas_closed_and_manifold)
    runner.bench_func("forge-closed-and-manifold-end-to-end", forge_closed_and_manifold)
    runner.bench_func("forge-prepacked-topology-kernel", forge_prepacked_topology_kernel)
