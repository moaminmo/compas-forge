"""COMPAS integration points and an explicit, reversible Mesh accelerator.

COMPAS 2.15 does not expose ``Mesh.is_closed`` or ``Mesh.is_manifold`` as
pluggable extension points. The decorated functions remain available to plugin
aware callers, while direct method acceleration is deliberately opt-in through
``mesh_acceleration`` or ``install_mesh_accelerators``.
"""

from contextlib import contextmanager
from threading import RLock

from compas.datastructures import Mesh
from compas.plugins import plugin
import compas_forge


_ACCELERATOR_LOCK = RLock()
_ORIGINAL_METHODS = {}
_INSTALL_DEPTH = 0


@plugin(category="datastructures", requires=["compas_forge"])
def is_mesh_manifold(mesh):
    """
    Checks edge incidence and vertex links through the Rust validation API.
    Call this function directly; automatic Mesh-method replacement is not assumed.
    """
    if not isinstance(mesh, Mesh):
        raise TypeError("mesh must be a compas.datastructures.Mesh")
    if mesh.number_of_vertices() == 0:
        return False
    # COMPAS considers isolated vertices non-manifold. Empty index buffers are
    # also problematic for some Python buffer providers, so handle this exact
    # semantic boundary before entering Rust.
    if any(mesh.vertex_degree(vertex) == 0 for vertex in mesh.vertices()):
        return False
    return compas_forge.mesh_topology_predicates(mesh)["is_manifold"]

@plugin(category="datastructures", requires=["compas_forge"])
def is_mesh_closed(mesh):
    """
    Checks for a nonempty manifold surface without boundary edges.
    """
    if not isinstance(mesh, Mesh):
        raise TypeError("mesh must be a compas.datastructures.Mesh")
    if mesh.number_of_vertices() == 0:
        return False
    # Match COMPAS semantics: a nonempty mesh with no edges has no naked edge.
    if mesh.number_of_faces() == 0:
        return True
    return compas_forge.mesh_topology_predicates(mesh)["is_closed"]


def _forge_mesh_is_manifold(self):
    return is_mesh_manifold(self)


def _forge_mesh_is_closed(self):
    return is_mesh_closed(self)


def accelerators_installed():
    """Return whether the process-wide COMPAS Mesh methods are accelerated."""
    with _ACCELERATOR_LOCK:
        return bool(_INSTALL_DEPTH)


def install_mesh_accelerators():
    """Opt in to process-wide Rust-backed Mesh topology predicates.

    Installation is re-entrant and returns ``True`` only when this call changes
    the class. Do not install or uninstall while other threads mutate the
    ``Mesh`` class. Query execution itself remains safe for independent meshes.
    """
    global _INSTALL_DEPTH
    with _ACCELERATOR_LOCK:
        changed = _INSTALL_DEPTH == 0
        if changed:
            _ORIGINAL_METHODS["is_closed"] = Mesh.is_closed
            _ORIGINAL_METHODS["is_manifold"] = Mesh.is_manifold
            Mesh.is_closed = _forge_mesh_is_closed
            Mesh.is_manifold = _forge_mesh_is_manifold
        _INSTALL_DEPTH += 1
        return changed


def uninstall_mesh_accelerators():
    """Release one installation level and restore COMPAS at depth zero."""
    global _INSTALL_DEPTH
    with _ACCELERATOR_LOCK:
        if _INSTALL_DEPTH == 0:
            return False
        _INSTALL_DEPTH -= 1
        if _INSTALL_DEPTH == 0:
            if (
                Mesh.is_closed is not _forge_mesh_is_closed
                or Mesh.is_manifold is not _forge_mesh_is_manifold
            ):
                _INSTALL_DEPTH = 1
                raise RuntimeError(
                    "COMPAS Mesh methods changed after Forge installation; "
                    "refusing to overwrite them"
                )
            Mesh.is_closed = _ORIGINAL_METHODS.pop("is_closed")
            Mesh.is_manifold = _ORIGINAL_METHODS.pop("is_manifold")
            return True
        return False


@contextmanager
def mesh_acceleration():
    """Temporarily route ``Mesh.is_closed/is_manifold`` through Forge."""
    install_mesh_accelerators()
    try:
        yield
    finally:
        uninstall_mesh_accelerators()
