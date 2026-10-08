"""Process-isolated performance evidence for the presentation.

Run after a release build:
    python -m maturin develop --release
    python benchmark_pyperf.py --rigorous -o pyperf-results.json

The three benchmarks intentionally name the work they perform. They must not
be collapsed into a universal "Rust speedup" ratio.
"""

from array import array
import json

import pyperf

import compas_forge as forge


def cube_buffers():
    vertices = array(
        "d",
        [
            -0.5, -0.5, -0.5, 0.5, -0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, -0.5,
            -0.5, -0.5, 0.5, 0.5, -0.5, 0.5, 0.5, 0.5, 0.5, -0.5, 0.5, 0.5,
        ],
    )
    indices = array(
        "i",
        [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7],
    )
    offsets = array("i", range(0, len(indices) + 1, 4))
    return vertices, indices, offsets


VERTICES, INDICES, OFFSETS = cube_buffers()
START = [-2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
END = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
FIXED = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
forge.clear_mesh_registry()
forge.register_mesh_buffers("pyperf-moving", VERTICES, INDICES, OFFSETS)
forge.register_mesh_buffers("pyperf-fixed", VERTICES, INDICES, OFFSETS)


def borrowed_ingress_scan():
    return forge.scan_mesh_buffers_borrowed(VERTICES, INDICES, OFFSETS)


def cached_ccd_native_dict():
    return forge.check_swept_collision_cached_native(
        "pyperf-moving", START, END, "pyperf-fixed", FIXED, FIXED
    )


def cached_ccd_json_compatibility():
    return json.loads(
        forge.check_swept_collision_cached(
            "pyperf-moving", START, END, "pyperf-fixed", FIXED, FIXED
        )
    )


if __name__ == "__main__":
    runner = pyperf.Runner()
    runner.metadata["build_profile"] = "release"
    runner.metadata["claim_scope"] = "same machine, same fixture, named work only"
    runner.bench_func("borrowed-ingress-structural-scan", borrowed_ingress_scan)
    runner.bench_func("cached-linear-ccd-native-dict", cached_ccd_native_dict)
    runner.bench_func("cached-linear-ccd-json-compatibility", cached_ccd_json_compatibility)
