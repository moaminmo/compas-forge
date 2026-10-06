# Release check - 6 October 2026

Source distribution for research and supervised evaluation. The checks below were executed locally on Windows against this source snapshot. Passing tests do not certify all host workflows or production suitability. Original validation documents retain their historical dates.

- Python suite: 35 PASS on Windows x64 / CPython 3.14, including headless COMPAS FAB 1.1.4 UR5 integration.
- Rust quality gates: formatting PASS; Clippy with warnings denied PASS; 4 Cargo unit tests PASS.
- Release-mode native extension build: PASS.

Packaging excludes caches, generated build outputs, environments and compiled binaries. Build prerequisites and scope are described in README, CLAIMS and validation documents where present. No remote repository URL or publication status is invented.
