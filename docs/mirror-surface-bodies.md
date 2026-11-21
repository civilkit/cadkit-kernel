# Mirror of surface bodies

Supported (kernel `exec_mirror`, tests `mirror_of_surface_bodies` in `crates/ck-model/tests/thicken_freeform.rs`): a seed that is a surface body, whether a planar
surface, a surface extrude or a freeform skin, named by its feature id or by any of its faces (`F2`, `F3`, `F2.P`). A surface has no tool body to repeat, so the
mirror does not go through the tool path: the body is reflected in the plane (geometry of every face and edge, orientation reversed so the normals still agree with
the reflected surface), stored as a new surface body under the mirror's id, and its faces are named `<face>@<mirror>.1` like the faces of a mirrored solid.
A mirror whose seeds are all surfaces leaves the solid untouched; a mix of surfaces and tools mirrors both.

The earlier refusal ("the seed F3 is a freeform and leaves no tool body") remains only for features that really leave nothing to reflect (fillet, chamfer, shell, ...).

Not covered: merging the image with the seed into one sheet, and mirroring a surface that crosses the plane (the image overlaps the seed as a second body).
