# Summary

[Introduction](introduction.md)

# Getting started

- [Build the kernel](getting-started/build.md)
- [Call it from Node](getting-started/node.md)
- [A first part](getting-started/first-part.md)

# The part format

- [Overview](format/overview.md)
- [Identifiers and references](format/references.md)
- [Variables and expressions](format/expressions.md)
- [Planes](format/planes.md)
- [Sketches](format/sketches.md)
  - [The sketch solver](format/sketch-solver.md)
- [Features](format/features.md)
  - [Solids from sketches](format/features-solids.md)
  - [Edge and face edits](format/features-edits.md)
  - [Patterns and transforms](format/features-patterns.md)
  - [Surfaces and curves](format/features-surfaces.md)
  - [Parts and assemblies](format/features-assemblies.md)
- [Versions and earlier files](format/versions.md)

# The protocol

- [Overview](protocol/overview.md)
- [Replies and buffers](protocol/reply.md)
- [Regeneration](protocol/regen.md)
- [Queries](protocol/queries.md)
- [Errors](protocol/errors.md)
- [The native service](protocol/native-service.md)

# Inside the kernel

- [Architecture](kernel/architecture.md)
- [The B-rep model](kernel/brep.md)
- [Geometry](kernel/geometry.md)
- [Operations](kernel/operations.md)
- [Tessellation](kernel/tessellation.md)
- [Regeneration and caching](kernel/regeneration.md)
- [Persistent names](kernel/naming.md)

# Validation

- [Overview](validation/overview.md)
- [Topology and geometry checks](validation/checks.md)
- [The validate request](validation/validate.md)
- [Tessellation checks](validation/tessellation.md)
- [Tolerances](validation/tolerances.md)
- [Healing](validation/healing.md)
- [Feature errors and diagnostics](validation/diagnostics.md)
- [Mass properties and interference](validation/analysis.md)

# Import and export

- [STEP](io/step.md)
- [SketchUp](io/skp.md)
- [SolidWorks](io/sldprt.md)
- [ACIS SAT](io/sat.md)
- [Feature recognition](io/recognition.md)
- [Text and fonts](io/text.md)
- [Drawings](io/drawings.md)

# Quality

- [Testing](quality/testing.md)
- [Corpora and sweeps](quality/corpora.md)
- [Robustness under edits](quality/robustness.md)
- [Known limits](quality/limits.md)

# Reference

- [Crate map](reference/crates.md)
- [Glossary](reference/glossary.md)
