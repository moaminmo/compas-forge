# COMPAS Forge — LinkedIn draft

## Post

I built COMPAS Forge to make geometry checks a practical part of a computational-design workflow.

The Rust core connects to COMPAS through Python and a CLI, covering topology diagnostics, repair operations, moving-mesh queries and fabrication-profile reports. The attached figure comes from the actual API: a tetrahedron with one missing face reports three boundary edges; restoring that face reduces the count to zero.

The useful engineering detail is the explicit contract: units, validation errors and diagnostic results travel with the geometry. A passing surface check is not a manufacturing certificate.

The repository includes source, tests, a runnable headless example and the data behind this figure. I would welcome reproducible geometry cases from design and fabrication workflows.

#ComputationalDesign #RustLang #COMPAS #ResearchSoftware

## Image

Attach `evidence.png` as the main image.

**Alt text:** The retained tetrahedron demo reports three boundary edges when one face is missing and zero when it is restored. These are actual API results, not a rendered mockup.

**Posting note:** Add the actual repository link after the repository has been created. No URL or release DOI has been invented.
