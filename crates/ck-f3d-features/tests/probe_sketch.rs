//! Decoder probe (diagnosis only, env-gated): `CK_F3D_PROBE=<text>` names the corpus files (path contains the text) whose sketch
//! entity classes are listed: per design, the classes of every sketch's entity list, the sketch classes of the class table, and the
//! sketch curves that no sketch lists. Without the variable (or `CK_F3D_CORPUS`) it passes at once.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use ck_f3d_design::{classes, sketch, stream::Segment};

fn find(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            find(&p, out);
        } else if matches!(p.extension().and_then(|x| x.to_str()), Some("f3d" | "f3z")) {
            out.push(p);
        }
    }
}

#[test]
fn sketch_entity_classes() {
    let (Ok(dir), Ok(want)) = (std::env::var("CK_F3D_CORPUS"), std::env::var("CK_F3D_PROBE")) else { return };
    let mut files = Vec::new();
    find(&PathBuf::from(dir), &mut files);
    files.sort();
    for path in files.iter().filter(|p| p.to_string_lossy().contains(&want)) {
        let Ok(docs) = ck_f3d_design::documents(path) else { continue };
        for (label, doc) in docs {
            let Ok(Some(s)) = ck_f3d_design::find_design_streams(&doc) else { continue };
            let Ok(seg) = Segment::parse(&s.meta, s.bulk) else { continue };
            eprintln!("== {label}");
            let mut listed = std::collections::HashSet::new();
            for sk in seg.objects_of(classes::SKETCH) {
                let ents = sketch::entity_list(&seg, sk);
                let mut by: BTreeMap<String, usize> = BTreeMap::new();
                for &e in &ents {
                    listed.insert(e);
                    let g = seg.guid_of(e).unwrap_or("?");
                    let chain: Vec<String> = seg.chain(g).iter().take(3).map(|c| c[..8.min(c.len())].to_string()).collect();
                    *by.entry(chain.join("<")).or_default() += 1;
                }
                eprintln!("  sketch {} entities {} {:?}", sk.id, ents.len(), by);
            }
            let mut orphans: BTreeMap<String, usize> = BTreeMap::new();
            for o in seg.objects_of_kind(classes::SKETCH_CURVE) {
                if !listed.contains(&o.id) {
                    *orphans.entry(seg.guid(o).unwrap_or("?").to_string()).or_default() += 1;
                }
            }
            eprintln!("  curves no sketch lists: {orphans:?}");
            for (i, c) in seg.meta.classes.iter().enumerate() {
                if c.module.contains("Sketch") || c.module.contains("Project") {
                    eprintln!("  class {} parent {} v{} module {} objects {}", c.guid, c.parent, c.version, c.module, c.ids.len());
                    let _ = i;
                }
            }
        }
    }
}

/// `CK_F3D_DUMP_DIR=<dir>` (with `CK_F3D_PROBE`): writes each matching design's dump as JSON (`<n>.json`) and an `index.txt`
/// naming them, for reading with other tools. `CK_F3D_STEPS=<text>` prints, per design whose file name contains the text and that has at most 80 features, the volume and lumps of the regenerated
/// history after each written feature (the first feature at which it departs from the file's body is where to look).
#[test]
fn dump_designs_and_steps() {
    let (Ok(dir), Ok(want)) = (std::env::var("CK_F3D_CORPUS"), std::env::var("CK_F3D_PROBE")) else { return };
    let out = std::env::var("CK_F3D_DUMP_DIR").ok();
    let steps = std::env::var("CK_F3D_STEPS").ok();
    if out.is_none() && steps.is_none() {
        return;
    }
    let mut files = Vec::new();
    find(&PathBuf::from(dir), &mut files);
    files.sort();
    let mut index = String::new();
    let mut n = 0;
    for path in files.iter().filter(|p| p.to_string_lossy().contains(&want)) {
        let Ok(bytes) = std::fs::read(path) else { continue };
        let name = path.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
        let Ok(designs) = ck_f3d_design::decode_bytes(bytes, &name) else { continue };
        for (di, d) in designs.into_iter().enumerate() {
            let Some(dump) = d.dump else { continue };
            n += 1;
            index += &format!("{n}\t{name}#{di}\n");
            if let Some(o) = &out {
                let _ = std::fs::create_dir_all(o);
                let _ = std::fs::write(format!("{o}/{n}.json"), serde_json::to_string(&dump).unwrap_or_default());
            }
            if let Some(only) = steps.as_ref().filter(|o| name.contains(o.as_str())) {
                let _ = only;
                let import = ck_f3d_features::timeline_to_features(&dump);
                if import.features.len() > 80 {
                    continue;
                }
                eprintln!("== {n} {name}#{di}: {} features", import.features.len());
                for k in 1..=import.features.len() {
                    let part = serde_json::json!({"format": "ck-part", "version": 1, "features": &import.features[..k], "seq": import.seq});
                    let f = &import.features[k - 1];
                    let label = format!("{} {} {}", f["id"].as_str().unwrap_or(""), f["type"].as_str().unwrap_or(""), f["op"].as_str().unwrap_or(""));
                    match ck_f3d_features::guard::regenerate(&part) {
                        Ok((m, errs)) => eprintln!("  {k:3} {label:30} {} {}", m.map_or("no body".to_string(), |m| format!("{:.1} mm3 / {} lumps", m.volume, m.solids)), errs.last().cloned().unwrap_or_default()),
                        Err(e) => eprintln!("  {k:3} {label}: {e}"),
                    }
                }
            }
        }
    }
    if let Some(o) = &out {
        let _ = std::fs::write(format!("{o}/index.txt"), index);
    }
}

/// `CK_F3D_EDGES=<file text>:<item index>` (with the corpus): for the blend item of the design(s) in the matching file, the
/// file's edges (middle point, length, ends, mm) and the nearest edges of the history written for the items before it.
#[test]
fn blend_edges_against_the_history() {
    let (Ok(dir), Ok(specs)) = (std::env::var("CK_F3D_CORPUS"), std::env::var("CK_F3D_EDGES")) else { return };
    let mut files = Vec::new();
    find(&PathBuf::from(dir), &mut files);
    files.sort();
    for spec in specs.split(';') {
    let (want, item_ix) = spec.rsplit_once(':').expect("text:item");
    let item_ix: i64 = item_ix.parse().expect("an item index");
    for path in files.iter().filter(|p| p.to_string_lossy().contains(want)) {
        let Ok(bytes) = std::fs::read(path) else { continue };
        let name = path.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
        let Ok(designs) = ck_f3d_design::decode_bytes(bytes, &name) else { continue };
        for (di, d) in designs.into_iter().enumerate() {
            let Some(dump) = d.dump else { continue };
            let import = ck_f3d_features::timeline_to_features(&dump);
            let Some(pos) = import.items.iter().position(|i| i.index == item_ix) else { continue };
            let ids: std::collections::HashSet<&str> = import.items[..pos].iter().flat_map(|i| i.features.iter().map(String::as_str)).collect();
            let prefix: Vec<&serde_json::Value> = import.features.iter().filter(|f| f["id"].as_str().is_some_and(|i| ids.contains(i))).collect();
            let part = serde_json::json!({"format": "ck-part", "version": 1, "features": prefix, "seq": import.seq});
            let Some(body) = ck_f3d_features::guard::regenerate_body(&part) else {
                eprintln!("{name}#{di}: the history before item {item_ix} builds no body");
                continue;
            };
            let item = &dump.timeline_items()[pos];
            let json = serde_json::to_value(item).unwrap_or_default();
            eprintln!("== {name}#{di} item {item_ix} {}: {} edges in the history before it", import.items[pos].object_type, body.edge_ids().count());
            let sets = json["detail"]["edgeSets"].as_array().cloned().unwrap_or_default();
            for set in sets {
                for e in set["edges"].as_array().cloned().unwrap_or_default() {
                    let v3 = |k: &str| e[k].as_array().map(|a| a.iter().map(|x| x.as_f64().unwrap_or(f64::NAN) * 10.0).collect::<Vec<_>>());
                    let mid = v3("mid_point").unwrap_or_default();
                    let len = e["length"].as_f64().unwrap_or(f64::NAN) * 10.0;
                    eprintln!("  file edge mid {mid:.3?} len {len:.3} start {:.3?} end {:.3?} curve {}", v3("start_point"), v3("end_point"), e["_f3d"]["curves"][0]["type"]);
                    if mid.len() == 3 {
                        let mut near: Vec<(f64, f64, [f64; 3], bool)> = body
                            .edge_ids()
                            .map(|k| {
                                let m = body.edge_mid(k);
                                let ed = body.edge(k);
                                (((m.x - mid[0]).powi(2) + (m.y - mid[1]).powi(2) + (m.z - mid[2]).powi(2)).sqrt(), body.edge_length(k), [m.x, m.y, m.z], ed.v0 == ed.v1)
                            })
                            .collect();
                        near.sort_by(|a, b| a.0.total_cmp(&b.0));
                        for n in near.iter().take(3) {
                            eprintln!("      kernel edge {:.4} away, len {:.3}, mid {:.3?}, closed {}", n.0, n.1, n.2, n.3);
                        }
                    }
                }
            }
        }
    }
    }
}

/// `CK_F3D_ATTRS=<file text>[:<sketch object id>]` (with the corpus): the root-part attributes of every entity of each sketch (or
/// the one), the ids profiles and entity inputs name curves by (`crv_primary_id`, `crv_secondary_id`, `pt_tag`).
#[test]
fn sketch_entity_attributes() {
    let (Ok(dir), Ok(spec)) = (std::env::var("CK_F3D_CORPUS"), std::env::var("CK_F3D_ATTRS")) else { return };
    let (want, only) = match spec.split_once(':') {
        Some((w, o)) => (w.to_string(), o.parse::<u64>().ok()),
        None => (spec.clone(), None),
    };
    let mut files = Vec::new();
    find(&PathBuf::from(dir), &mut files);
    files.sort();
    for path in files.iter().filter(|p| p.to_string_lossy().contains(&want)) {
        let Ok(docs) = ck_f3d_design::documents(path) else { continue };
        for (label, doc) in docs {
            let Ok(Some(s)) = ck_f3d_design::find_design_streams(&doc) else { continue };
            let Ok(seg) = Segment::parse(&s.meta, s.bulk) else { continue };
            eprintln!("== {label}");
            for sk in seg.objects_of(classes::SKETCH) {
                if only.is_some_and(|o| o != sk.id) {
                    continue;
                }
                eprintln!("  sketch {}", sk.id);
                for e in sketch::entity_list(&seg, sk) {
                    let Some(root) = seg.root_part(seg.data_of(e)) else { continue };
                    let attrs: Vec<String> = root.attrs.iter().map(|a| format!("{}={:?}", a.key, a.value)).collect();
                    eprintln!("    {e} {} {}", seg.guid_of(e).and_then(|g| g.get(..8)).unwrap_or("?"), attrs.join(" "));
                }
            }
        }
    }
}

/// `CK_F3D_ORPHANS=<file text>` (with the corpus): objects that carry `crv_primary_id` (the id profiles name curves by) and are
/// listed by no sketch: their class, ids, parent chain and the references in their data to sketches and curves.
#[test]
fn curve_ids_outside_the_sketch_lists() {
    let (Ok(dir), Ok(want)) = (std::env::var("CK_F3D_CORPUS"), std::env::var("CK_F3D_ORPHANS")) else { return };
    let mut files = Vec::new();
    find(&PathBuf::from(dir), &mut files);
    files.sort();
    for path in files.iter().filter(|p| p.to_string_lossy().contains(&want)) {
        let Ok(docs) = ck_f3d_design::documents(path) else { continue };
        for (label, doc) in docs {
            let Ok(Some(s)) = ck_f3d_design::find_design_streams(&doc) else { continue };
            let Ok(seg) = Segment::parse(&s.meta, s.bulk) else { continue };
            let mut listed = std::collections::HashSet::new();
            for sk in seg.objects_of(classes::SKETCH) {
                listed.extend(sketch::entity_list(&seg, sk));
            }
            eprintln!("== {label}");
            let mut by: BTreeMap<String, usize> = BTreeMap::new();
            for o in &seg.objects {
                let Some(root) = seg.root_part(seg.data(o)) else { continue };
                let attr = |k: &str| root.attrs.iter().find(|a| a.key == k).and_then(|a| match a.value {
                    ck_f3d_design::stream::AttrValue::U64(v) => Some(v),
                    _ => None,
                });
                let Some(primary) = attr("crv_primary_id") else { continue };
                if listed.contains(&o.id) {
                    continue;
                }
                let g = seg.guid(o).unwrap_or("?");
                let chain: Vec<String> = seg.chain(g).iter().take(4).map(|c| c.get(..8).unwrap_or(c).to_string()).collect();
                let mut refs: Vec<String> = Vec::new();
                for r in seg.ref_ids(seg.data(o)).into_iter().take(8) {
                    refs.push(format!("{r}:{}", seg.guid_of(r).and_then(|g| g.get(..8)).unwrap_or("?")));
                }
                *by.entry(chain.join("<")).or_default() += 1;
                eprintln!("  object {} {} primary {primary} secondary {:?} refs {refs:?}", o.id, chain.join("<"), attr("crv_secondary_id"));
            }
            eprintln!("  orphan classes {by:?}");
        }
    }
}
