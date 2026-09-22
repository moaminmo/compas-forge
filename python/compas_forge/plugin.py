from compas.plugins import plugin
import compas_forge

@plugin(category='datastructures', requires=['compas_forge'])
def is_mesh_manifold(mesh):
    """
    Checks edge incidence and vertex links through the Rust validation API.
    Call this function directly; automatic Mesh-method replacement is not assumed.
    """
    try:
        report = compas_forge.verify_mesh_zero_copy(mesh)
        return not report.get("non_manifold_edges") and not report.get("non_manifold_vertices")
    except Exception:
        return False

@plugin(category='datastructures', requires=['compas_forge'])
def is_mesh_closed(mesh):
    """
    Checks for a nonempty manifold surface without boundary edges.
    """
    try:
        report = compas_forge.verify_mesh_zero_copy(mesh)
        return report.get("is_valid", False) and report.get("face_count", 0) > 0 and report.get("boundary_edges_count", 1) == 0
    except Exception:
        return False
