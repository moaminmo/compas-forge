"""Reproducible, correctness-gated CCD latency measurements.

This is not a cross-library speedup benchmark. It separates cached native
queries from full COMPAS ingestion and records raw samples for independent
inspection. Run against a release build on an otherwise idle machine.
"""

import argparse
import gc
import importlib.metadata
import json
import math
import platform
import statistics
import time
from pathlib import Path

from compas.datastructures import Mesh

import compas_forge as forge


IDENTITY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


def cube_mesh(size=1.0):
    half = size / 2.0
    vertices = [
        [-half, -half, -half], [half, -half, -half],
        [half, half, -half], [-half, half, -half],
        [-half, -half, half], [half, -half, half],
        [half, half, half], [-half, half, half],
    ]
    faces = [
        [0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
        [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7],
    ]
    return Mesh.from_vertices_and_faces(vertices, faces)


def percentile(sorted_samples, quantile):
    index = max(0, math.ceil(quantile * len(sorted_samples)) - 1)
    return sorted_samples[index]


def measure(name, function, repetitions, warmups, expected_collision):
    for _ in range(warmups):
        result = function()
    if result["has_collision"] is not expected_collision:
        raise AssertionError(f"{name}: correctness gate failed before timing")

    samples_us = []
    gc_enabled = gc.isenabled()
    gc.disable()
    try:
        for _ in range(repetitions):
            start = time.perf_counter_ns()
            result = function()
            samples_us.append((time.perf_counter_ns() - start) / 1_000.0)
    finally:
        if gc_enabled:
            gc.enable()

    if result["has_collision"] is not expected_collision:
        raise AssertionError(f"{name}: correctness gate failed after timing")
    ordered = sorted(samples_us)
    return {
        "name": name,
        "repetitions": repetitions,
        "warmups": warmups,
        "expected_collision": expected_collision,
        "median_us": statistics.median(ordered),
        "p95_us": percentile(ordered, 0.95),
        "p99_us": percentile(ordered, 0.99),
        "min_us": ordered[0],
        "max_us": ordered[-1],
        "samples_us": samples_us,
    }


def run(repetitions, rotation_repetitions, warmups):
    mesh = cube_mesh()
    start = [-2.0, 0.0, 0.0, *IDENTITY[3:]]
    end = [2.0, 0.0, 0.0, *IDENTITY[3:]]
    fixed = IDENTITY
    rotation_end = [0.0, 0.0, 0.0, 0.0, 0.0,
                    math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)]

    forge.clear_mesh_cache()
    forge.register_mesh_to_cache("moving", mesh)
    forge.register_mesh_to_cache("fixed", mesh)

    cached_linear = lambda: forge.check_swept_collision_cached_poses(
        "moving", start, end, "fixed", fixed, fixed
    )
    cached_linear_json = lambda: json.loads(
        forge.check_swept_collision_cached(
            "moving", start, end, "fixed", fixed, fixed
        )
    )
    uncached_linear = lambda: forge.sweep_collision(
        mesh, start, end, mesh, fixed, fixed
    )
    cached_rotation = lambda: forge.check_swept_collision_cached_poses(
        "moving", start, rotation_end, "fixed", fixed, fixed
    )
    batch_queries = [
        ("moving", start, end, "fixed", fixed, fixed) for _ in range(32)
    ]

    def repeated_native_32():
        results = [
            forge.check_swept_collision_cached_poses(*query)
            for query in batch_queries
        ]
        return {"has_collision": all(result["has_collision"] for result in results)}

    def batch_native_32(parallel):
        results = forge.check_swept_collision_cached_batch_poses(
            batch_queries, parallel=parallel
        )
        return {"has_collision": all(result["has_collision"] for result in results)}

    workloads = [
        measure(
            "cached_linear_native_dict", cached_linear, repetitions, warmups, True
        ),
        measure(
            "cached_linear_json_compatibility",
            cached_linear_json,
            repetitions,
            warmups,
            True,
        ),
        measure("compas_ingestion_linear_hit", uncached_linear, repetitions, warmups, True),
        measure(
            "cached_nonlinear_rotation",
            cached_rotation,
            rotation_repetitions,
            min(warmups, 5),
            True,
        ),
        measure(
            "32_cached_linear_individual_calls",
            repeated_native_32,
            repetitions,
            warmups,
            True,
        ),
        measure(
            "32_cached_linear_native_serial_batch",
            lambda: batch_native_32(False),
            repetitions,
            warmups,
            True,
        ),
        measure(
            "32_cached_linear_native_parallel_batch",
            lambda: batch_native_32(True),
            repetitions,
            warmups,
            True,
        ),
    ]
    return {
        "schema_version": 1,
        "clock": "time.perf_counter_ns",
        "units": "microseconds",
        "python": platform.python_version(),
        "platform": platform.platform(),
        "processor": platform.processor(),
        "forge_version": importlib.metadata.version("compas-forge"),
        "methodology": {
            "release_build_required": True,
            "garbage_collection_disabled_during_samples": True,
            "raw_samples_retained": True,
            "claim_scope": "machine-specific latency; no comparative speedup claim",
        },
        "workloads": workloads,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repetitions", type=int, default=2_000)
    parser.add_argument("--rotation-repetitions", type=int, default=30)
    parser.add_argument("--warmups", type=int, default=100)
    parser.add_argument(
        "--output", type=Path, default=Path("benchmark-ccd-results.json")
    )
    arguments = parser.parse_args()
    if min(arguments.repetitions, arguments.rotation_repetitions, arguments.warmups) < 1:
        parser.error("repetitions and warmups must be positive")
    payload = run(
        arguments.repetitions, arguments.rotation_repetitions, arguments.warmups
    )
    arguments.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    for workload in payload["workloads"]:
        print(
            f"{workload['name']}: median={workload['median_us']:.3f} us, "
            f"p95={workload['p95_us']:.3f} us, p99={workload['p99_us']:.3f} us"
        )
    print(f"Raw evidence: {arguments.output}")
