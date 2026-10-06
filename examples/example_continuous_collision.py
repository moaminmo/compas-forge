import time
import math
from compas.datastructures import Mesh
from compas.geometry import Frame
import compas_forge

def create_cube_geometry(size=1.0):
    """
    Generates a 3D manifold cube for swept-collision verification.
    """
    mesh = Mesh()
    h = size / 2.0
    v_coords = [
        [-h, -h, -h], [h, -h, -h], [h, h, -h], [-h, h, -h],
        [-h, -h, h], [h, -h, h], [h, h, h], [-h, h, h]
    ]
    for coord in v_coords:
        mesh.add_vertex(x=coord[0], y=coord[1], z=coord[2])
    mesh.add_face([0, 3, 2, 1])
    mesh.add_face([4, 5, 6, 7])
    mesh.add_face([0, 1, 5, 4])
    mesh.add_face([1, 2, 6, 5])
    mesh.add_face([2, 3, 7, 6])
    mesh.add_face([3, 0, 4, 7])
    return mesh

def run_example():
    print("=" * 60)
    print("  COMPAS FORGE - ROTATIONAL SWEPT-COLLISION (CCD) EXAMPLE")
    print("=" * 60)
    
    cube_a = create_cube_geometry(1.0)
    cube_b = create_cube_geometry(1.0)

    # Cube A moves from x=-3 to x=3 while rotating 90 degrees around Z.
    frame_a_start = Frame.worldXY()
    frame_a_start.point.x = -3.0
    frame_a_end = Frame.from_euler_angles(
        [0.0, 0.0, math.pi / 2.0], point=[3.0, 0.0, 0.0]
    )

    # Cube B remains static at the world origin.
    frame_b = Frame.worldXY()

    t0 = time.perf_counter_ns()
    result = compas_forge.sweep_collision(
        cube_a, frame_a_start, frame_a_end,
        cube_b, frame_b, frame_b
    )
    latency_ms = (time.perf_counter_ns() - t0) / 1_000_000.0

    print(f"\n[Execution Profiler] Evaluation Time: {latency_ms:.4f} ms")
    print(f"  Continuous Collision Detected: {result['has_collision']}")
    print(f"  Solver Method: {result['method']}")
    print(f"  Temporal Substeps: {result['substeps']}")
    print(f"  Normalized Time of Impact (TOI): {result['time_of_impact']:.6f} (0-1)")
    impact = result["impact"]
    print(f"  Solver Status: {impact['status']}")
    print(f"  Solver Converged: {impact['converged']}")
    print(f"  Conservative Estimate: {impact['conservative']}")
    print(f"  Impact Geometry Reliable: {impact['geometry_reliable']}")
    print(f"  Impact Contact Normal A (World): {impact['normal_a_world']}")
    print(f"  Contact Point A (World): {impact['witness_a_world']}")
    print(f"  Contact Point B (World): {impact['witness_b_world']}")

if __name__ == "__main__":
    run_example()
