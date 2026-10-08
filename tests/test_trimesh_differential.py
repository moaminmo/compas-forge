"""Optional semantic cross-checks against an independent mesh library."""

import math

import numpy as np
import pytest

trimesh = pytest.importorskip("trimesh")

from compas.datastructures import Mesh

import compas_forge as forge


CUBE_VERTICES = [
    [-0.5, -0.5, -0.5],
    [0.5, -0.5, -0.5],
    [0.5, 0.5, -0.5],
    [-0.5, 0.5, -0.5],
    [-0.5, -0.5, 0.5],
    [0.5, -0.5, 0.5],
    [0.5, 0.5, 0.5],
    [-0.5, 0.5, 0.5],
]
CUBE_QUADS = [
    [0, 3, 2, 1],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [1, 2, 6, 5],
    [2, 3, 7, 6],
    [3, 0, 4, 7],
]
CUBE_TRIANGLES = [
    [0, 3, 2], [0, 2, 1], [4, 5, 6], [4, 6, 7],
    [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
    [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7],
]


def test_closed_cube_agrees_with_trimesh_on_watertightness_winding_and_volume():
    forge_mesh = Mesh.from_vertices_and_faces(CUBE_VERTICES, CUBE_QUADS)
    reference = trimesh.Trimesh(
        vertices=np.asarray(CUBE_VERTICES),
        faces=np.asarray(CUBE_TRIANGLES),
        process=False,
    )

    preflight = forge.preflight_mesh(forge_mesh)

    assert preflight["is_watertight"] is bool(reference.is_watertight)
    assert preflight["winding_consistent"] is bool(reference.is_winding_consistent)
    assert preflight["volume_reliable"]
    assert math.isclose(preflight["volume_m3"], abs(reference.volume), abs_tol=1e-12)


def test_open_cube_agrees_with_trimesh_on_non_watertight_classification():
    open_quads = CUBE_QUADS[:-1]
    open_triangles = CUBE_TRIANGLES[:-2]
    forge_mesh = Mesh.from_vertices_and_faces(CUBE_VERTICES, open_quads)
    reference = trimesh.Trimesh(
        vertices=np.asarray(CUBE_VERTICES),
        faces=np.asarray(open_triangles),
        process=False,
    )

    preflight = forge.preflight_mesh(forge_mesh)

    assert not reference.is_watertight
    assert not preflight["is_watertight"]
    assert preflight["boundary_edges_count"] == 4
    assert not preflight["volume_reliable"]
