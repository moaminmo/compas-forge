# COMPAS Forge — reproducible publication example

![Executed example](evidence.png)

The retained tetrahedron demo reports three boundary edges when one face is missing and zero when it is restored. These are actual API results, not a rendered mockup.

## Reproduce the data

Run from the repository root after following its build instructions. The command writes into `publication/`; keep that directory present. Installed SDKs, built native libraries, model weights or Radiance are required only where named.

```powershell
python examples/publication_demo.py
```

Primary retained data: [demo.json](demo.json). The figure is a rendering of these retained results, not a screenshot of the host application. Fixture inputs are in the linked demo source. Results were recorded locally on Windows x64 on 2026-09-19; they are not remote CI badges.

## Regenerate the figure

```powershell
python -m pip install -r publication/requirements.txt
python publication/render_figure.py
```

The PNG is 1920 × 1080, suitable for README and LinkedIn use; the SVG remains editable. Both use the same data. The data-generation programs assert the fixture results; the renderer reads the retained output.

## Scope

Surface diagnostics and geometric queries; profiles are screening rules, not material or machine certification.

## Publication assets

- `evidence.png`: prepared social/README image.
- `evidence.svg`: editable vector figure.
- `linkedin.md`: English introduction draft and image description.
- `manifest.json`: SHA-256 identity of the demo source, data and graphics.

These files are prepared assets; no GitHub or LinkedIn publication is implied.

## Demo source

- [examples/publication_demo.py](../examples/publication_demo.py)
