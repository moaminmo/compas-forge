import pytest

from compas.datastructures import Mesh

import compas_forge as forge


def topology_cases():
    return [
        Mesh(),
        Mesh.from_vertices_and_faces([[0, 0, 0]], []),
        Mesh.from_vertices_and_faces([[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0, 1, 2]]),
        Mesh.from_vertices_and_faces(
            [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
            [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]],
        ),
        Mesh.from_vertices_and_faces(
            [[0, 0, 0], [1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]],
            [[0, 1, 2], [0, 3, 4]],
        ),
    ]


def test_opt_in_accelerator_preserves_compas_predicate_semantics_and_restores_methods():
    originals = (Mesh.is_closed, Mesh.is_manifold)
    meshes = topology_cases()
    expected = [(mesh.is_closed(), mesh.is_manifold()) for mesh in meshes]

    with forge.mesh_acceleration():
        assert forge.accelerators_installed()
        assert (Mesh.is_closed, Mesh.is_manifold) != originals
        observed = [(mesh.is_closed(), mesh.is_manifold()) for mesh in meshes]

    assert observed == expected
    assert not forge.accelerators_installed()
    assert (Mesh.is_closed, Mesh.is_manifold) == originals


def test_opt_in_accelerator_is_nested_and_rejects_wrong_types():
    originals = (Mesh.is_closed, Mesh.is_manifold)
    with forge.mesh_acceleration():
        with forge.mesh_acceleration():
            assert forge.accelerators_installed()
        assert forge.accelerators_installed()
    assert (Mesh.is_closed, Mesh.is_manifold) == originals

    from compas_forge.plugin import is_mesh_closed, is_mesh_manifold

    with pytest.raises(TypeError):
        is_mesh_closed(object())
    with pytest.raises(TypeError):
        is_mesh_manifold(object())


def test_dedicated_topology_kernel_reports_evidence_without_geometry_work():
    tetrahedron = topology_cases()[3]
    report = forge.mesh_topology_predicates(tetrahedron)

    assert report["is_closed"]
    assert report["is_manifold"]
    assert report["boundary_edges_count"] == 0
    assert report["non_manifold_edges"] == []
    assert report["non_manifold_vertices"] == []
    assert report["isolated_vertices_count"] == 0
