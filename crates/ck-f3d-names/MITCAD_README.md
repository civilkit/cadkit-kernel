# mitcad-f3d-names: names of B-rep entities in `.f3d` body blobs

`core/f3d-names` reads the `generic_tag_attrib_def` names that faces and
bodies of the ASM blobs carry, and finds among the faces and edges of a
history state the entities that the design's feature inputs name by them
(`NamedState`): faces, edges by the faces along them and at their ends
(`EdgeTail`, the tail of an edge recipe, mitcad#106), vertices, with
their geometry. `surface` finds points inside faces of any surface type.

It builds on `mitcad-asm` (the ASM bodies) and `mitcad-dump` (the names as
the dump IR's recipes give them). Until mitcad#776 it was
`mitcad_f3d::names`; that path still works (`mitcad-f3d` re-exports it),
and `mitcad_f3d::design::recipe` re-exports `EdgeTail` and `edge_tail`.
