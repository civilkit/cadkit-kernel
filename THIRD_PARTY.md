# Third-party code

Vendored or adapted code and its licence. Only permissively licensed sources.

| Path | Source | Licence |
|---|---|---|
| crates/ck-check/src/{shell,wire,edge,solid}.rs, LICENSE-MIT, LICENSE-APACHE | brepkit v2.129.15, crates/check/src/validate/ (check logic adapted onto ck-topo; no brepkit code linked) | MIT OR Apache-2.0 |
| crates/ck-heal/src/{vertex,small,wire,sew,orient}.rs, LICENSE-MIT, LICENSE-APACHE | brepkit v2.129.15, crates/heal/src/{fix/wire,fix/face,fix/shell,fix/solid,fix/small_face,fix/split_vertex,fix/wireframe,analysis/wire_order,upgrade/shell_sewing}.rs (fix catalogue and ordering adapted onto ck-topo / ck-geom; detection via ck-check; no brepkit code linked) | MIT OR Apache-2.0 |
