//! Surface-body features -> ck-ops surfacing (knit, fill, trimSurface,
//! thicken, extend, deleteFace, freeform/rebuild, deleteBody).
//!
//! Body model (RECONSTRUCTION from oracle probes): besides the solid body a
//! regen keeps SURFACE bodies keyed by the feature id that made them
//! ([`Brep::surfs`], wire `surfaces: [[id, body], ..]` after the solid
//! body). A feature that consumes surfaces removes them; a result that is
//! a closed shell becomes the solid body (`fill {merge}` closing a knitted
//! skin: surface list empties, `body` appears). Face references
//! (`F13.S.e4`) resolve across the solid and all surface bodies.

use super::{Brep, FeatureKind};
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_geom::Vec3;
use ck_json::Json;
use ck_ops::surfacing as sf;
use ck_topo::body::{Body, EdgeId, FaceId};

/// Where a face reference lives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Slot {
    Solid,
    Surf(usize),
}

fn missing() -> FeatureError {
    FeatureError::new(codes::MISSING_REFERENCE)
}

fn coded(code: &str, detail: &str) -> FeatureError {
    let fe = FeatureError::new(code);
    if detail.is_empty() {
        fe
    } else {
        fe.with("detail", Json::Str(detail.to_string()))
    }
}

/// `true` for the feature types this module executes.
pub fn handles(f: &Feature) -> bool {
    matches!(
        f.type_name.as_str(),
        "surfacePlanar" | "surfaceRevolve" | "surfaceExtrude" | "knit" | "fill" | "trimSurface" | "thicken" | "extend" | "deleteFace" | "freeform" | "deleteBody"
    )
}

pub fn dispatch(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    use super::surface_extrude as se;
    match f.type_name.as_str() {
        "surfacePlanar" => se::exec_surface_planar(br, state, f),
        "surfaceRevolve" => se::exec_surface_revolve(br, state, f),
        "surfaceExtrude" => se::exec_surface_extrude(br, state, f),
        "knit" => exec_knit(br, state, f),
        "trimSurface" => exec_trim(br, state, f),
        "fill" => exec_fill(br, state, f),
        "thicken" => exec_thicken(br, state, f),
        "extend" => exec_extend(br, state, f),
        "deleteFace" => exec_delete_face(br, state, f),
        "freeform" => exec_freeform(br, state, f),
        "deleteBody" => super::delete_body::exec_delete_body(br, state, f),
        _ => unreachable!(),
    }
}

impl Brep {
    /// Register a surface body under a feature id (replacing a stale one).
    pub fn set_surface(&mut self, id: &str, b: Body) {
        if let Some(s) = self.surfs.iter_mut().find(|s| s.0 == id) {
            s.1 = b;
        } else {
            self.surfs.push((id.to_string(), b));
        }
    }

    pub fn surf_index(&self, id: &str) -> Option<usize> {
        self.surfs.iter().position(|s| s.0 == id)
    }

    /// The body holding the live face `face`.
    pub fn slot_of_face(&self, face: &str) -> Option<Slot> {
        if let Some(i) = self.surfs.iter().position(|(_, b)| b.find_face(face).is_some()) {
            return Some(Slot::Surf(i));
        }
        self.body.as_ref().filter(|b| b.find_face(face).is_some()).map(|_| Slot::Solid)
    }

    pub fn slot_body(&self, s: Slot) -> Option<&Body> {
        match s {
            Slot::Solid => self.body.as_ref(),
            Slot::Surf(i) => self.surfs.get(i).map(|s| &s.1),
        }
    }

    /// A feature reference to a surface body: a surface feature id, or any
    /// face id of it.
    pub fn surface_ref(&self, r: &str) -> Option<usize> {
        if let Some(i) = self.surf_index(r) {
            return Some(i);
        }
        match self.slot_of_face(r) {
            Some(Slot::Surf(i)) => Some(i),
            _ => None,
        }
    }

    /// Store `b` as the result of `id` in place of the body at `slot`
    /// (extend / rebuild / fill-merge keep the body's identity): a closed
    /// shell is the solid body, an open one stays a surface body.
    pub fn replace_slot(&mut self, slot: Slot, id: &str, b: Body) {
        match slot {
            Slot::Solid => self.body = Some(b),
            Slot::Surf(i) => {
                // pinned (extend probe): the result is appended after the
                // other surfaces; a closed shell becomes the solid body
                self.surfs.remove(i);
                if is_closed(&b) {
                    self.body = Some(b);
                } else {
                    self.surfs.push((id.to_string(), b));
                }
            }
        }
    }

    /// Drop the surface bodies at the given indices.
    pub fn drop_surfaces(&mut self, mut idx: Vec<usize>) {
        idx.sort_unstable();
        idx.dedup();
        for i in idx.into_iter().rev() {
            if i < self.surfs.len() {
                self.surfs.remove(i);
            }
        }
    }
}

/// Every edge has two coedges.
pub fn is_closed(b: &Body) -> bool {
    b.num_faces() > 0 && b.edge_ids().all(|e| b.edge_coedges(e).iter().filter(|&&c| b.coedge(c).alive).count() == 2)
}

fn strs(j: Option<&Json>) -> Vec<String> {
    j.and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect()).unwrap_or_default()
}

fn flag(f: &Feature, k: &str) -> bool {
    crate::sketch::jtruthy(f.raw.get(k))
}

/// Resolve an `edges[]` pick `{faces, at}` to (slot, edge): a one-face pick
/// is the open boundary edge of that face nearest `at`; with several faces
/// the edge they share (nearest `at`).
fn pick_edge(br: &Brep, faces: &[String], at: Option<Vec3>) -> Result<(Slot, EdgeId), FeatureError> {
    let first = faces.first().ok_or_else(missing)?;
    let slot = br.slot_of_face(first).ok_or_else(missing)?;
    let b = br.slot_body(slot).ok_or_else(missing)?;
    let fids: Vec<FaceId> = faces.iter().map(|n| b.find_face(n).ok_or_else(missing)).collect::<Result<_, _>>()?;
    let dist = |e: EdgeId| -> f64 {
        match at {
            Some(p) => {
                let ed = b.edge(e);
                ck_ops::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), p).1
            }
            None => 0.0,
        }
    };
    let cand: Vec<EdgeId> = b
        .edge_ids()
        .filter(|&e| {
            let cs: Vec<_> = b.edge_coedges(e).into_iter().filter(|&c| b.coedge(c).alive).collect();
            let fs = b.edge_faces(e);
            if fids.len() == 1 {
                // open boundary edges first
                cs.len() == 1 && fs.contains(&fids[0])
            } else {
                fids.iter().all(|f| fs.contains(f))
            }
        })
        .collect();
    let cand = if cand.is_empty() && fids.len() == 1 {
        b.face_edges(fids[0])
    } else {
        cand
    };
    cand.into_iter()
        .min_by(|&x, &y| dist(x).partial_cmp(&dist(y)).unwrap_or(std::cmp::Ordering::Equal))
        .map(|e| (slot, e))
        .ok_or_else(missing)
}

fn edge_picks(f: &Feature) -> Vec<(Vec<String>, Option<Vec3>)> {
    let FeatureKind::Fill { edges, .. } = &f.kind else {
        // extend: raw edges array
        return f
            .raw
            .get("edges")
            .and_then(|a| a.as_arr())
            .map(|a| a.iter().filter_map(crate::EdgeRef::from_json).map(|e| (e.faces, e.at.map(|p| Vec3::new(p[0], p[1], p[2])))).collect())
            .unwrap_or_default();
    };
    edges.iter().map(|e| (e.faces.clone(), e.at.map(|p| Vec3::new(p[0], p[1], p[2])))).collect()
}

// ------------------------------------------------------------------ knit

pub fn exec_knit(br: &mut Brep, _state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Knit { surfaces, solid } = &f.kind else { unreachable!() };
    if surfaces.is_empty() {
        return Err(missing());
    }
    // pinned: unresolved references are skipped (knit of [F2, F9] = F2)
    let mut idx: Vec<usize> = vec![];
    for r in surfaces {
        if let Some(i) = br.surface_ref(r.as_str()) {
            if !idx.contains(&i) {
                idx.push(i);
            }
        }
    }
    if idx.is_empty() {
        return Err(missing());
    }
    let bodies: Vec<Body> = idx.iter().map(|&i| br.surfs[i].1.clone()).collect();
    let out = sf::knit::knit(&bodies, *solid, super::TOL).map_err(|e| {
        if e.code == "knitNotClosed" {
            // pinned params: `{open: <n open edges>, points: [edge mids]}`
            if let Ok(sheet) = sf::knit::knit(&bodies, false, super::TOL) {
                let open: Vec<EdgeId> = sheet
                    .edge_ids()
                    .filter(|&x| sheet.edge_coedges(x).iter().filter(|&&c| sheet.coedge(c).alive).count() == 1)
                    .collect();
                let pts = open
                    .iter()
                    .map(|&x| {
                        let m = sheet.edge_mid(x);
                        Json::Arr(vec![Json::Num(m.x), Json::Num(m.y), Json::Num(m.z)])
                    })
                    .collect();
                return FeatureError::new("knitNotClosed")
                    .with("open", Json::Num(open.len() as f64))
                    .with("points", Json::Arr(pts));
            }
        }
        coded(e.code, &e.detail)
    })?;
    br.drop_surfaces(idx);
    if *solid {
        join_solid(br, &f.id, out)?;
    } else {
        br.set_surface(&f.id, out);
    }
    Ok(())
}

/// Add a solid result: the first one becomes the body, later ones unite.
fn join_solid(br: &mut Brep, id: &str, b: Body) -> Result<(), FeatureError> {
    let _ = id;
    br.combine(&b, super::extrude::Combine::Add, false)
}

// ------------------------------------------------------------------ trim

pub fn exec_trim(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::TrimSurface { surface, surfaces, tool_surface, plane, mutual, flip, .. } = &f.kind else { unreachable!() };
    // pinned: an absent `deleteTool` keeps the tool (the client writes true)
    let delete_tool = &flag(f, "deleteTool");
    let mut refs: Vec<&crate::Ref> = surfaces.iter().collect();
    if let Some(s) = surface {
        if refs.is_empty() {
            refs.push(s);
        }
    }
    if refs.is_empty() {
        return Err(missing());
    }
    let mut tidx = vec![];
    for r in refs {
        tidx.push(br.surface_ref(r.as_str()).ok_or_else(missing)?);
    }
    let targets: Vec<Body> = tidx.iter().map(|&i| br.surfs[i].1.clone()).collect();
    // tool: a surface body, or a plane (std / plane feature / planar face) as a bare plane face
    let (tool, tool_i) = match (tool_surface, plane) {
        (Some(t), _) => {
            let i = br.surface_ref(t.as_str()).ok_or_else(missing)?;
            (br.surfs[i].1.clone(), Some(i))
        }
        (None, Some(pl)) => {
            let fr = match pl {
                crate::GeomRef::Std(p) => p.frame(),
                crate::GeomRef::Feature(id) => state.plane(id).ok_or_else(missing)?,
                crate::GeomRef::Face(id) => {
                    let slot = br.slot_of_face(id).ok_or_else(missing)?;
                    crate::refs::face_frame(br.slot_body(slot).ok_or_else(missing)?, id).ok_or_else(missing)?
                }
                _ => return Err(missing()),
            };
            (plane_sheet(fr), None)
        }
        _ => return Err(missing()),
    };
    // ck-ops trim cuts by a PLANE tool only; surface x surface trimming (the
    // large sample's loft x barrel) is not reconstructed yet
    let planar_tool = tool.face_ids().all(|x| matches!(tool.face(x).surface, ck_geom::surface::Surface::Plane { .. }));
    let planar_targets = targets.iter().all(|t| t.face_ids().all(|x| matches!(t.face(x).surface, ck_geom::surface::Surface::Plane { .. })));
    if !planar_tool || !planar_targets {
        return Err(FeatureError::new(&super::extrude::todo("trimSurface of curved surfaces")));
    }
    let out = sf::trim::trim(&targets, &tool, *flip, *mutual, super::TOL).map_err(|e| coded(e.code, &e.detail))?;
    let mut gone = tidx;
    if let Some(ti) = tool_i {
        if *delete_tool || *mutual {
            gone.push(ti);
        }
    }
    br.drop_surfaces(gone);
    br.set_surface(&f.id, out);
    Ok(())
}

/// A bare plane face (no loops): only its plane matters to `trim`.
fn plane_sheet(fr: ck_geom::Frame) -> Body {
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    b.add_face(shell, ck_geom::surface::Surface::Plane { frame: fr }, true, ck_topo::body::FaceName::other("plane", "P"));
    b
}

// ------------------------------------------------------------------ fill

pub fn exec_fill(br: &mut Brep, _state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Fill { merge, continuity, fairing, .. } = &f.kind else { unreachable!() };
    let picks = edge_picks(f);
    if picks.is_empty() {
        return Err(missing());
    }
    let mut slot = None;
    let mut chain = vec![];
    for (faces, at) in &picks {
        let (s, e) = pick_edge(br, faces, *at)?;
        if *slot.get_or_insert(s) != s {
            // GUESS: a fill chain lives in one body
            return Err(missing());
        }
        chain.push(e);
    }
    let slot = slot.unwrap();
    let body = br.slot_body(slot).ok_or_else(missing)?.clone();
    let mode = fairing.as_deref().and_then(sf::fair::Fairing::parse).unwrap_or(sf::fair::Fairing::MeowNurbs);
    let strength = f.raw.get("guideStrength").and_then(|v| v.as_f64()).unwrap_or(0.5);
    let cont = match continuity {
        super::Continuity::G0 => "G0",
        super::Continuity::G1 => "G1",
        super::Continuity::G2 => "G2",
        super::Continuity::G3 => "G3",
    };
    let out = sf::fill::fill(&body, &chain, cont, mode, strength, &f.id, super::TOL).map_err(|e| {
        // pinned: an invalid patch face is reported as `fillInvalid`
        let code = if e.detail.contains("not a valid face") { "fillInvalid" } else { e.code };
        coded(code, &e.detail)
    })?;
    if *merge {
        br.replace_slot(slot, &f.id, out);
    } else {
        // the patch alone, as its own surface body
        let patch: Vec<FaceId> = out.face_ids().filter(|&x| out.face(x).name.feature == f.id).collect();
        let sheet = sf::sheet::sheet(&out, &patch);
        br.set_surface(&f.id, sheet);
    }
    Ok(())
}

// --------------------------------------------------------------- thicken

pub fn exec_thicken(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let r = f.raw.get("surface").and_then(|s| s.as_str()).ok_or_else(missing)?;
    let i = br.surface_ref(r).ok_or_else(missing)?;
    let t = state.num(f, "thickness", 2.0)?;
    let side = f.raw.get("side").and_then(|s| s.as_str()).and_then(sf::thicken::Side::parse).unwrap_or(sf::thicken::Side::Front);
    let out = sf::thicken::thicken(&br.surfs[i].1, &f.id, t, side, super::TOL).map_err(|e| coded(e.code, &e.detail))?;
    // pinned: the thickened surface is consumed
    br.drop_surfaces(vec![i]);
    join_solid(br, &f.id, out)
}

// ---------------------------------------------------------------- extend

pub fn exec_extend(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let picks = edge_picks(f);
    if picks.is_empty() {
        return Err(missing());
    }
    let mut slot = None;
    let mut edges = vec![];
    for (faces, at) in &picks {
        let (s, e) = pick_edge(br, faces, *at)?;
        // pinned: only surface bodies extend (a solid's edge is `missingReference`)
        if s == Slot::Solid || *slot.get_or_insert(s) != s {
            return Err(missing());
        }
        edges.push(e);
    }
    let slot = slot.unwrap();
    if let Some(sh) = f.raw.get("shape").and_then(|s| s.as_str()) {
        if sh != "same" {
            // oracle: linear / soft / reflective add a separate spline face `X<k>`
            return Err(FeatureError::new(&super::extrude::todo("extend shape other than same")));
        }
    }
    let shape = f.raw.get("shape").and_then(|s| s.as_str()).and_then(sf::extend::Shape::parse).unwrap_or(sf::extend::Shape::Same);
    let end_kind = f.raw.get("end").and_then(|s| s.as_str()).unwrap_or("distance");
    let end = if end_kind == "surface" {
        let id = f.raw.get("target").and_then(|t| t.get_str("id")).ok_or_else(missing)?;
        let ti = br.surface_ref(id).ok_or_else(missing)?;
        let tb = &br.surfs[ti].1;
        let face = tb.face_ids().next().ok_or_else(missing)?;
        sf::extend::End::Surface(tb.face(face).surface.clone())
    } else {
        sf::extend::End::Distance(state.num(f, "distance", 5.0)?)
    };
    let body = br.slot_body(slot).ok_or_else(missing)?.clone();
    let out = sf::extend::extend(&body, &edges, shape, &end, &f.id, super::TOL).map_err(|e| coded(e.code, &e.detail))?;
    // pinned: the wire edge numbers are compact (no dead arena slots)
    let all: Vec<FaceId> = out.face_ids().collect();
    let out = sf::sheet::sheet(&out, &all);
    br.replace_slot(slot, &f.id, out);
    Ok(())
}

// ------------------------------------------------------------ deleteFace

pub fn exec_delete_face(br: &mut Brep, _state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let names = strs(f.raw.get("faces"));
    if names.is_empty() {
        return Err(missing());
    }
    let slot = br.slot_of_face(&names[0]).ok_or_else(missing)?;
    let body = br.slot_body(slot).ok_or_else(missing)?.clone();
    let mut del = vec![];
    for n in &names {
        del.push(body.find_face(n).ok_or_else(missing)?);
    }
    let mode = f.raw.get("mode").and_then(|m| m.as_str()).unwrap_or("delete");
    if mode != "delete" {
        return Err(FeatureError::new(&super::extrude::todo("deleteFace fill/heal")));
    }
    let keep: Vec<FaceId> = body.face_ids().filter(|x| !del.contains(x)).collect();
    let out = sf::sheet::sheet(&body, &keep);
    match slot {
        Slot::Solid => br.body = None,
        Slot::Surf(i) => {
            br.surfs.remove(i);
        }
    }
    br.set_surface(&f.id, out);
    Ok(())
}

// -------------------------------------------------------------- freeform

pub fn exec_freeform(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let names = strs(f.raw.get("faces"));
    if names.is_empty() {
        return Err(missing());
    }
    let slot = br.slot_of_face(&names[0]).ok_or_else(missing)?;
    let body = br.slot_body(slot).ok_or_else(missing)?.clone();
    let mut faces = vec![];
    for n in &names {
        faces.push(body.find_face(n).ok_or_else(missing)?);
    }
    let has_edits = f.raw.get("edits").and_then(|e| e.as_arr()).map_or(false, |a| !a.is_empty());
    if has_edits {
        return Err(FeatureError::new(&super::extrude::todo("freeform edits")));
    }
    let n = |k: &str, d: f64| state.num(f, k, d);
    let (du, dv) = (n("degreeU", 3.0)?.max(1.0) as usize, n("degreeV", 3.0)?.max(1.0) as usize);
    let (su, sv) = (n("spansU", 2.0)?.max(1.0) as usize, n("spansV", 2.0)?.max(1.0) as usize);
    let out = sf::rebuild::rebuild(&body, &faces, du, dv, su, sv, super::TOL).map_err(|e| coded(e.code, &e.detail))?;
    // pinned marker: the refit control net of every face, `net[i * nv + j]`
    let num = |x: usize| Json::Num(x as f64);
    let mut mf = vec![];
    for (n, &fid) in names.iter().zip(&faces) {
        if let ck_geom::surface::Surface::BSpline(bs) = &out.face(fid).surface {
            let net = bs.ctrl.iter().map(|p| Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)])).collect();
            mf.push(Json::Obj(vec![
                ("face".into(), Json::Str(n.clone())),
                ("degree".into(), Json::Arr(vec![num(du), num(dv)])),
                ("spans".into(), Json::Arr(vec![num(su), num(sv)])),
                ("size".into(), Json::Arr(vec![num(bs.nu), num(bs.nv)])),
                ("net".into(), Json::Arr(net)),
            ]));
        }
    }
    state.result.markers.push(Json::Arr(vec![Json::Str(f.id.clone()), Json::Obj(vec![("faces".into(), Json::Arr(mf))])]));
    br.replace_slot(slot, &f.id, out);
    Ok(())
}
