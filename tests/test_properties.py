"""Generated scientific checks for invariants, not hand-picked examples."""

from array import array
import math

import pytest

hypothesis = pytest.importorskip("hypothesis")
from hypothesis import given, settings, strategies as st

from compas.datastructures import Mesh

import compas_forge as forge


finite_coordinate = st.floats(
    min_value=-1.0e6,
    max_value=1.0e6,
    allow_nan=False,
    allow_infinity=False,
    width=64,
)


@settings(max_examples=75, deadline=None)
@given(st.lists(st.tuples(finite_coordinate, finite_coordinate, finite_coordinate), min_size=3, max_size=50))
def test_borrowed_scan_matches_python_bounds(vertices):
    flat = array("d", (coordinate for vertex in vertices for coordinate in vertex))
    indices = array("i", [0, 1, 2])
    offsets = array("i", [0, 3])

    report = forge.scan_mesh_buffers_borrowed(flat, indices, offsets)

    expected_min = [min(vertex[axis] for vertex in vertices) for axis in range(3)]
    expected_max = [max(vertex[axis] for vertex in vertices) for axis in range(3)]
    assert report["is_structurally_valid"]
    assert report["bounds_min"] == expected_min
    assert report["bounds_max"] == expected_max


@settings(max_examples=50, deadline=None)
@given(st.integers(min_value=3, max_value=500), st.integers(min_value=1, max_value=1000))
def test_borrowed_scan_counts_injected_out_of_range_index(vertex_count, excess):
    vertices = array("d", [0.0] * (vertex_count * 3))
    indices = array("i", [0, 1, vertex_count + excess])
    offsets = array("i", [0, 3])

    report = forge.scan_mesh_buffers_borrowed(vertices, indices, offsets)

    assert not report["is_structurally_valid"]
    assert report["invalid_index_count"] == 1


@settings(max_examples=40, deadline=None)
@given(
    finite_coordinate.filter(lambda value: abs(value) < 1.0e4),
    finite_coordinate.filter(lambda value: abs(value) < 1.0e4),
    finite_coordinate.filter(lambda value: abs(value) < 1.0e4),
)
def test_closed_tetrahedron_volume_is_translation_invariant(tx, ty, tz):
    vertices = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]
    faces = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]]
    base = Mesh.from_vertices_and_faces(vertices, faces)
    moved = Mesh.from_vertices_and_faces(
        [[x + tx, y + ty, z + tz] for x, y, z in vertices], faces
    )

    base_report = forge.preflight_mesh(base)
    moved_report = forge.preflight_mesh(moved)

    assert base_report["volume_reliable"]
    assert moved_report["volume_reliable"]
    assert math.isclose(
        base_report["volume_m3"], moved_report["volume_m3"], rel_tol=1e-9, abs_tol=1e-8
    )
