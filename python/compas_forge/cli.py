import sys
import click
import os
import json
import time
from rich.console import Console
from rich.table import Table
from compas.datastructures import Mesh
from compas_forge import (
    verify_file, 
    check_assembly_clashes, 
    fix_geometry_file, 
    run_preflight_profile,
    verify_mesh_zero_copy,
    check_swept_collision_zero_copy,
    compute_assembly_contacts_zero_copy
)
from compas_forge.reporter import generate_html_report

console = Console()

def parse_pose_str(pose_str):
    try:
        parts = [float(x.strip()) for x in pose_str.split(',')]
        if len(parts) != 7:
            raise ValueError()
        return parts
    except Exception:
        raise click.BadParameter("Pose must be 7 comma-separated floats: x,y,z,qx,qy,qz,qw")

@click.group()
def main():
    """
    COMPAS Forge: Mesh diagnostics and experimental fabrication checks
    """
    pass

@main.command()
@click.argument('filepath', type=click.Path(exists=True))
def check(filepath):
    """
    Checks topology and geometry metrics in a serialized COMPAS JSON file.
    """
    console.print(f"[bold blue]Initiating structural scan on:[/bold blue] {filepath}")
    try:
        mesh = Mesh.from_json(filepath)
        report = verify_mesh_zero_copy(mesh)
        
        table = Table(title="[bold green]COMPAS Forge Diagnostics Summary[/bold green]")
        table.add_column("Property", style="cyan")
        table.add_column("Count / Value", style="magenta")
        table.add_column("Status", style="bold")

        table.add_row("Vertices Found", str(report["vertex_count"]), "[green]PASS[/green]")
        table.add_row("Faces Found", str(report["face_count"]), "[green]PASS[/green]")
        
        dup_color = "red" if report["duplicate_vertices"] > 0 else "green"
        dup_status = "FAIL" if report["duplicate_vertices"] > 0 else "PASS"
        table.add_row("Duplicate Vertices", str(report["duplicate_vertices"]), f"[{dup_color}]{dup_status}[/{dup_color}]")
        table.add_row("Duplicate Tolerance", f"{report['duplicate_tolerance']:.3e}", "[green]INFO[/green]")

        nm_count = len(report["non_manifold_edges"])
        nm_color = "red" if nm_count > 0 else "green"
        nm_status = "FAIL" if nm_count > 0 else "PASS"
        table.add_row("Non-Manifold Edges", str(nm_count), f"[{nm_color}]{nm_status}[/{nm_color}]")

        degenerate_count = report["degenerate_faces_count"]
        degenerate_status = "[red]FAIL[/red]" if degenerate_count else "[green]PASS[/green]"
        table.add_row("Degenerate Faces", str(degenerate_count), degenerate_status)

        intersection_count = len(report["self_intersections"])
        intersection_status = "[red]FAIL[/red]" if intersection_count else "[green]PASS[/green]"
        table.add_row("Self-Intersecting Face Pairs", str(intersection_count), intersection_status)
        winding_status = "[green]PASS[/green]" if report["winding_consistent"] else "[red]FAIL[/red]"
        table.add_row("Consistent Winding", str(report["winding_consistent"]), winding_status)

        bbox = report["bounding_box"]
        x_dim = bbox["max_x"] - bbox["min_x"]
        y_dim = bbox["max_y"] - bbox["min_y"]
        z_dim = bbox["max_z"] - bbox["min_z"]
        bounds_str = f"X: {x_dim:.3f} | Y: {y_dim:.3f} | Z: {z_dim:.3f}"
        table.add_row("Bounding Box Dimensions", bounds_str, "[green]INFO[/green]")

        console.print(table)
        
        if nm_count > 0:
            console.print("\n[bold yellow]Detected Non-Manifold Edges (Vertex Indices):[/bold yellow]")
            for edge in report["non_manifold_edges"]:
                console.print(f"  • Edge: {edge}")

        if not report["is_valid"]:
            sys.exit(1)
        else:
            sys.exit(0)
    except Exception as e:
        console.print(f"[bold red]Execution interrupted due to parsing failure:[/bold red] {e}")
        sys.exit(2)


@main.command()
@click.argument('filepath', type=click.Path(exists=True))
@click.option('--profile', '-p', type=str, default="kuka-timber")
@click.option('--report-out', '-r', type=click.Path(), required=False)
def preflight(filepath, profile, report_out):
    """
    Checks mesh metrics against a selected fabrication profile.
    """
    console.print(f"[bold blue]Executing Preflight validation pipeline on:[/bold blue] {filepath}")
    console.print(f"[bold blue]Target Fabrication Profile:[/bold blue] {profile}\n")
    try:
        t_start = time.perf_counter_ns()
        
        t0 = time.perf_counter_ns()
        diagnostics = verify_file(filepath)
        t_diag_ms = (time.perf_counter_ns() - t0) / 1_000_000.0

        t1 = time.perf_counter_ns()
        preflight_data = run_preflight_profile(filepath, profile)
        t_prof_ms = (time.perf_counter_ns() - t1) / 1_000_000.0

        t2 = time.perf_counter_ns()
        rep_data = fix_geometry_file(filepath)
        t_fix_ms = (time.perf_counter_ns() - t2) / 1_000_000.0

        table = Table(title=f"[bold green]Preflight Metrics: {profile}[/bold green]")
        table.add_column("Fabrication / Topological Parameter", style="cyan")
        table.add_column("Calculated Metric", style="magenta")
        table.add_column("Compliance Status", style="bold")

        volume_reliable = preflight_data["volume_reliable"]
        volume_status = "[green]VERIFIED[/green]" if volume_reliable else "[yellow]UNRELIABLE[/yellow]"
        table.add_row("Solid Mesh Volume", f"{preflight_data['volume_m3']:.6f} m³", volume_status)
        
        mass_limit_fail = not preflight_data["mass_within_limit"]
        mass_color = "yellow" if not volume_reliable else ("red" if mass_limit_fail else "green")
        mass_status = "UNRELIABLE" if not volume_reliable else ("FAIL" if mass_limit_fail else "PASS")
        table.add_row(f"Estimated Net Mass (limit {preflight_data['max_mass_kg']:g} kg)", f"{preflight_data['estimated_mass_kg']:.3f} kg", f"[{mass_color}]{mass_status}[/{mass_color}]")
        
        workspace_status = "[green]PASS[/green]" if preflight_data["fits_workspace"] else "[red]FAIL[/red]"
        bounds_str = f"X: {preflight_data['bounds_x_dim']:.3f} | Y: {preflight_data['bounds_y_dim']:.3f} | Z: {preflight_data['bounds_z_dim']:.3f}"
        table.add_row("Envelope Dimensions", bounds_str, workspace_status)

        wt_status = "[green]PASS[/green]" if preflight_data["is_watertight"] else "[red]FAIL[/red]"
        table.add_row("Boundary edges (not hole count)", str(preflight_data["boundary_edges_count"]), wt_status)

        table.add_row("Euler Characteristic (χ)", str(preflight_data["euler_characteristic"]), "[green]INFO[/green]")
        table.add_row("Geometric Genus (g)", str(preflight_data["genus"]), "[green]INFO[/green]")
        
        planarity_status = "[green]PASS[/green]" if preflight_data["max_planarity_deviation"] <= 0.005 else "[red]FAIL[/red]"
        table.add_row("Max Planarity Deviation", f"{preflight_data['max_planarity_deviation']:.6f} m", planarity_status)

        quality_status = "[green]PASS[/green]" if preflight_data["min_face_quality"] >= 0.1 else "[red]FAIL[/red]"
        table.add_row("Minimum Facet Quality (q)", f"{preflight_data['min_face_quality']:.4f}", quality_status)

        console.print(table)
        
        timing_profile = {
            "Parsing_and_mesh_checks": t_diag_ms,
            "Physical_Gauss_Evaluation": t_prof_ms,
            "Winding_Orientation_Weld_Repairs": t_fix_ms
        }

        if report_out:
            t3 = time.perf_counter_ns()
            generate_html_report(diagnostics, preflight_data, rep_data, timing_profile, report_out)
            t_rep_ms = (time.perf_counter_ns() - t3) / 1_000_000.0
            console.print(f"\n[bold green]✔ Interactive HTML preflight report generated successfully at:[/bold green] {report_out}")

        if preflight_data["is_compliant"]:
            console.print(f"\n[bold green]PREFLIGHT CHECKS PASSED for '{profile}'. Independent geometry and process validation is still required.[/bold green]")
            sys.exit(0)
        else:
            console.print(f"\n[bold red]❌ PREFLIGHT VIOLATION: Component violates workspace limits, payload thresholds, or watertightness criteria for '{profile}'.[/bold red]")
            sys.exit(1)

    except Exception as e:
        console.print(f"[bold red]Preflight processing failed:[/bold red] {e}")
        sys.exit(2)


@main.command()
@click.argument('filepath', type=click.Path(exists=True))
@click.option('--output', '-o', type=click.Path(), required=True, help="Path to save the fixed COMPAS JSON file")
def fix(filepath, output):
    """
    Welds duplicate vertices, unifies winding, and exports a repair log.
    """
    console.print(f"[bold blue]Initiating Auto-Fixer pipeline on:[/bold blue] {filepath}")
    try:
        report = fix_geometry_file(filepath)
        
        table = Table(title="[bold green]Auto-Fixer Execution Summary[/bold green]")
        table.add_column("Optimization Parameter", style="cyan")
        table.add_column("Count Affected", style="magenta")
        table.add_column("Repair Status", style="bold green")

        table.add_row("Merged Duplicate Vertices (Weld)", str(report["welded_count"]), "FIXED")
        table.add_row("Flipped Winding Normal Directions", str(report["flipped_count"]), "FIXED")
        console.print(table)

        fixed_data = json.loads(report["fixed_json"])
        with open(output, 'w', encoding='utf-8') as f:
            json.dump(fixed_data, f, indent=4)
            
        console.print(f"\n[bold green]✔ Repair pipeline completed. Restructured file exported to:[/bold green] {output}")
        sys.exit(0)
        
    except Exception as e:
        console.print(f"[bold red]Repair process failed:[/bold red] {e}")
        sys.exit(2)


@main.command()
@click.argument('files', nargs=-1, type=click.Path(exists=True), required=True)
@click.option('--clearance', '-c', type=float, default=0.0, help="Minimum safe clearance tolerance distance in meters")
def clash(files, clearance):
    """
    Queries mesh spatial intersections and clearance tolerance violations across multiple COMPAS files.
    """
    console.print(f"[bold blue]Loading and indexing {len(files)} assembly parts...[/bold blue]")
    if clearance > 0.0:
        console.print(f"[bold yellow]Clearance tolerance threshold set to:[/bold yellow] {clearance:.4f} meters")
    
    files_map = {}
    for filepath in files:
        filename = os.path.abspath(filepath)
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                files_map[filename] = f.read()
        except Exception as e:
            console.print(f"[bold red]Failed to read {filename}:[/bold red] {e}")
            sys.exit(1)

    try:
        collisions = check_assembly_clashes(files_map, clearance)
        
        if not collisions:
            console.print("\n[bold green]No intersections or clearance violations detected by this mesh query.[/bold green]")
            sys.exit(0)
        else:
            table = Table(title="[bold red]Spatial Clash & Clearance Report[/bold red]")
            table.add_column("Index", style="cyan")
            table.add_column("Element A", style="magenta")
            table.add_column("Element B", style="magenta")
            table.add_column("Min Distance (m)", style="bold yellow")
            table.add_column("Incident Type", style="bold red")

            unique_collisions = {}
            for report in collisions:
                pair = tuple(sorted([report["part_a"], report["part_b"]]))
                if pair not in unique_collisions:
                    unique_collisions[pair] = report

            for idx, (pair, rep) in enumerate(unique_collisions.items(), 1):
                inc_type = "Physical Mesh Collision" if rep["has_intersection"] else "Clearance Violation"
                table.add_row(
                    str(idx), 
                    rep["part_a"], 
                    rep["part_b"], 
                    f"{rep['minimum_distance']:.5f}", 
                    inc_type
                )

            console.print(table)
            console.print(f"\n[bold red]❌ Found {len(unique_collisions)} unique spatial violation(s). Adjust physical coordinates.[/bold red]")
            sys.exit(1)

    except Exception as e:
        console.print(f"[bold red]Clash processing failed:[/bold red] {e}")
        sys.exit(2)


@main.command()
@click.argument('mesh_a_path', type=click.Path(exists=True))
@click.argument('pose_a_start_str')
@click.argument('pose_a_end_str')
@click.argument('mesh_b_path', type=click.Path(exists=True))
@click.argument('pose_b_start_str')
@click.argument('pose_b_end_str')
def swept(mesh_a_path, pose_a_start_str, pose_a_end_str, mesh_b_path, pose_b_start_str, pose_b_end_str):
    """
    Continuous Collision Detection (CCD) between two moving COMPAS meshes.
    Poses format: x,y,z,qx,qy,qz,qw
    """
    console.print("[bold blue]Executing continuous swept trajectory intersection (CCD)...[/bold blue]")
    try:
        mesh_a = Mesh.from_json(mesh_a_path)
        mesh_b = Mesh.from_json(mesh_b_path)

        pose_a_start = parse_pose_str(pose_a_start_str)
        pose_a_end = parse_pose_str(pose_a_end_str)
        pose_b_start = parse_pose_str(pose_b_start_str)
        pose_b_end = parse_pose_str(pose_b_end_str)

        t0 = time.perf_counter_ns()
        result = check_swept_collision_zero_copy(
            mesh_a, pose_a_start, pose_a_end,
            mesh_b, pose_b_start, pose_b_end
        )
        t_ms = (time.perf_counter_ns() - t0) / 1_000_000.0

        table = Table(title="[bold green]Continuous Collision Detection (CCD) Report[/bold green]")
        table.add_column("Parameter", style="cyan")
        table.add_column("Calculated Metric", style="magenta")

        table.add_row("Evaluation Time", f"{t_ms:.4f} ms")
        table.add_row("Collision Detected", "[red]TRUE[/red]" if result["has_collision"] else "[green]FALSE[/green]")
        table.add_row("Method", result["method"])
        table.add_row("Temporal substeps", str(result["substeps"]))
        toi = result["time_of_impact"]
        impact = result["impact"]
        table.add_row("First Time of Impact (normalised 0-1)", f"{toi:.6f}" if toi is not None else "N/A")
        if impact is not None:
            table.add_row("Solver status", impact["status"])
            table.add_row("Converged", str(impact["converged"]))
            table.add_row("Conservative estimate", str(impact["conservative"]))
            table.add_row("Impact geometry reliable", str(impact["geometry_reliable"]))
            table.add_row("Verification distance", str(impact["verification_distance"]))
            table.add_row("Impact Normal A (world)", str(impact["normal_a_world"]))
            table.add_row("Impact Point A (world)", str(impact["witness_a_world"]))
            table.add_row("Impact Point B (world)", str(impact["witness_b_world"]))

        console.print(table)
        sys.exit(0)
    except Exception as e:
        console.print(f"[bold red]CCD computation failed:[/bold red] {e}")
        sys.exit(2)


@main.command()
@click.argument('files', nargs=-1, type=click.Path(exists=True), required=True)
@click.option('--tolerance', '-t', type=float, default=0.005, help="Contact tolerance distance in meters")
def contacts(files, tolerance):
    """
    Estimates face-to-face contact interfaces across multiple COMPAS meshes.
    """
    console.print(f"[bold blue]Indexing {len(files)} assembly parts for contact manifold analysis...[/bold blue]")
    try:
        meshes_dict = {}
        for filepath in files:
            name = os.path.abspath(filepath)
            meshes_dict[name] = Mesh.from_json(filepath)

        t0 = time.perf_counter_ns()
        interfaces = compute_assembly_contacts_zero_copy(meshes_dict, tolerance)
        t_ms = (time.perf_counter_ns() - t0) / 1_000_000.0

        table = Table(title="[bold green]Discrete Element Assembly Contacts[/bold green]")
        table.add_column("Index", style="cyan")
        table.add_column("Block A", style="magenta")
        table.add_column("Block B", style="magenta")
        table.add_column("Area (m²)", style="bold yellow")
        table.add_column("Centroid [X, Y, Z]", style="cyan")

        for idx, item in enumerate(interfaces, 1):
            centroid_str = f"[{item['centroid'][0]:.3f}, {item['centroid'][1]:.3f}, {item['centroid'][2]:.3f}]"
            table.add_row(
                str(idx),
                item['block_a'],
                item['block_b'],
                f"{item['area_m2']:.6f}",
                centroid_str
            )

        console.print(table)
        console.print(f"[bold green]✔ Analyzed assembly in {t_ms:.4f} ms. Found {len(interfaces)} contacts.[/bold green]")
        sys.exit(0)
    except Exception as e:
        console.print(f"[bold red]Assembly contact solver failed:[/bold red] {e}")
        sys.exit(2)


@main.command('trajectory')
@click.argument('filepath', type=click.Path(exists=True,dir_okay=False,readable=True))
@click.option('--clearance', required=True, type=float, help='Requested distance in input model length units.')
@click.option('--articulated-tolerance', type=float, default=None,
              help='Bound linear-joint interpolation error, including fixed-state attachments.')
@click.option('--solid', is_flag=True, help='Require the restricted single-shell solid contract.')
@click.option('--solid-rule', type=click.Choice(['single_shell','even_odd']), default='single_shell',
              help='Explicit shell fill rule; even_odd requires --solid.')
@click.option('--max-articulated-refinements', type=click.IntRange(0,8), default=2,
              help='Local bisection depth for unresolved bounded joint intervals.')
@click.option('--max-evaluations', type=click.IntRange(min=1), default=4096, show_default=True)
@click.option('--max-subdivisions', type=click.IntRange(min=1), default=128, show_default=True)
@click.option('--serial', is_flag=True, help='Disable native query parallelism.')
def trajectory_check(filepath,clearance,articulated_tolerance,solid,max_evaluations,max_subdivisions,serial,
                     solid_rule,max_articulated_refinements):
    """Check a trusted COMPAS JSON bundle containing cell, state and trajectory.

    Collision meshes must already be embedded/loaded; no implicit mesh downloads.
    JSON report is written to stdout. Exit 0=clear, 1=violation, 2=input/runtime
    error, 3=unknown. This command never sends motion to a robot/controller.
    """
    import hashlib
    import platform
    from pathlib import Path
    try:
        import compas
        import compas_fab
        import compas_forge
        from compas_forge import _core
        from compas_fab.robots import RobotCell,RobotCellState,JointTrajectory
        raw = Path(filepath).read_bytes()
        bundle = compas.json_loads(raw.decode('utf-8'))
        if not isinstance(bundle,dict):
            raise ValueError('bundle must contain cell, state and trajectory')
        unsupported = set(bundle)-{'cell','state','trajectory','scene_poses','tool_trajectories'}
        if unsupported:
            raise ValueError('unsupported bundle fields (not silently ignored): '+', '.join(sorted(unsupported)))
        for key,expected in [('cell',RobotCell),('state',RobotCellState),('trajectory',JointTrajectory)]:
            if not isinstance(bundle.get(key),expected):
                raise ValueError(f'{key} must be a serialized {expected.__name__}')
        with compas_forge.prepare_compas_fab_cell(bundle['cell'],bundle['state']) as prepared:
            result = prepared.verify_clearance(bundle['trajectory'],clearance,
                articulated_tolerance=articulated_tolerance,solid=solid,
                solid_rule=solid_rule,max_articulated_refinements=max_articulated_refinements,
                scene_poses=bundle.get('scene_poses'),
                tool_trajectories=bundle.get('tool_trajectories'),
                max_evaluations=max_evaluations,max_subdivisions=max_subdivisions,parallel=not serial)
        package_dir = Path(compas_forge.__file__).parent
        implementation_paths = {name:package_dir/name for name in ['__init__.py','cell.py','motion_bounds.py','cli.py']}
        implementation_paths['native_extension'] = Path(_core.__file__)
        report = dict(schema_version=1,input_sha256=hashlib.sha256(raw).hexdigest(),
            implementation_sha256={name:hashlib.sha256(path.read_bytes()).hexdigest()
                                   for name,path in implementation_paths.items()},
            versions=dict(python=platform.python_version(),compas=compas.__version__,
                          compas_fab=compas_fab.__version__,compas_forge=compas_forge.__version__),
            distance_units='input_model_length_units',robot_commands_sent=False,result=result)
        output = json.dumps(report,indent=2,allow_nan=False)
    except Exception as error:
        click.echo(f'Trajectory preflight failed: {error}',err=True)
        raise click.exceptions.Exit(2) from error
    click.echo(output)
    raise click.exceptions.Exit({'clear':0,'violation':1,'unknown':3}[result['status']])


if __name__ == '__main__':
    main()
