"""Measure the actual Forge diagnostic API; write raw, machine-specific timings.

The measured stages perform different work. Their times are not a Python/Rust
speedup comparison. Run with the release-built extension in the active environment.
"""
import argparse
import importlib.metadata
import json
import platform
import statistics
import time
from pathlib import Path
from compas.datastructures import Mesh
import compas_forge as forge


def grid_mesh(n):
    vertices = [[float(x), float(y), 0.] for y in range(n + 1) for x in range(n + 1)]
    faces = [[y*(n+1)+x, y*(n+1)+x+1, (y+1)*(n+1)+x+1, (y+1)*(n+1)+x]
             for y in range(n) for x in range(n)]
    return Mesh.from_vertices_and_faces(vertices, faces)


def measure(call, repetitions):
    call()  # Warm-up is excluded.
    samples = []
    for _ in range(repetitions):
        start = time.perf_counter_ns()
        call()
        samples.append((time.perf_counter_ns()-start)/1e6)
    return {"samples_ms": samples, "median_ms": statistics.median(samples),
            "min_ms": min(samples), "max_ms": max(samples)}


def run(sizes, repetitions):
    results = []
    for n in sizes:
        mesh = grid_mesh(n)
        buffers = forge.compas_mesh_to_buffers(mesh)
        def diagnostics():
            report = json.loads(forge.validate_mesh_buffers(*buffers))
            assert report['face_count'] == n*n
            assert report['boundary_edges_count'] == 4*n
            assert report['is_valid']
        results.append({"grid_side": n, "faces": n*n, "vertices": (n+1)**2,
            "buffer_construction": measure(lambda: forge.compas_mesh_to_buffers(mesh), repetitions),
            "rust_diagnostics_with_owned_copy_and_json": measure(diagnostics, repetitions)})
    return {"python": platform.python_version(), "platform": platform.platform(),
            "processor": platform.processor(), "forge_version": importlib.metadata.version('compas-forge'),
            "repetitions": repetitions, "warmup_calls": 1,
            "build_profile": "Record the build command separately; runtime does not identify the Rust profile.",
            "comparison": "No equivalent baseline or cross-machine speedup is asserted.", "results": results}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sizes', type=int, nargs='+', default=[20, 40, 80])
    parser.add_argument('--repetitions', type=int, default=7)
    parser.add_argument('--output', type=Path, default=Path('benchmark-results.json'))
    args = parser.parse_args()
    if args.repetitions < 2 or any(n < 1 for n in args.sizes):
        parser.error('Use at least two repetitions and positive grid sizes.')
    payload = run(args.sizes, args.repetitions)
    args.output.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    print(f'Wrote {len(payload["results"])} measured workloads to {args.output}')
