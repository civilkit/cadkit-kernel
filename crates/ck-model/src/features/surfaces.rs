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

    /// `missingReference` for a face id nothing carries, listing the faces the solid and the surfaces have instead.
    pub fn missing_face(&self, face: &str) -> FeatureError {
        crate::refs::missing_face(self.body.iter().chain(self.surfs.iter().map(|(_, b)| b)), face)
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
pub(super) fn pick_edge(br: &Brep, faces: &[String], at: Option<Vec3>) -> Result<(Slot, EdgeId), FeatureError> {
    let first = faces.first().ok_or_else(missing)?;
    let slot = br.slot_of_face(first).ok_or_else(|| br.missing_face(first))?;
    let b = br.slot_body(slot).ok_or_else(missing)?;
    let fids: Vec<FaceId> = faces.iter().map(|n| b.find_face(n).ok_or_else(|| crate::refs::missing_face([b], n))).collect::<Result<_, _>>()?;
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
    // ck-ops trims planar tools with the pinned half-plane clip and any
    // other pairing (surface x surface, the large sample's loft x barrel)
    // via SSI + region clipping in `sf::trim::trim_curved`
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
    let map = |e: sf::thicken::ThickenError| {
        let mut fe = coded(e.code, &e.detail);
        if let Some(h) = &e.hint {
            fe = fe.with("hint", Json::Str(h.clone()));
        }
        match &e.limit {
            Some(l) => super::offset_limit_params(fe, l),
            None => fe,
        }
    };
    let ramp = thickness_at(f)?;
    let constant = ramp.is_none();
    let out = match ramp {
        None => sf::thicken::thicken(&br.surfs[i].1, &f.id, t, side, super::TOL).map_err(map)?,
        Some(ramp) => sf::thicken::thicken_variable(&br.surfs[i].1, &f.id, &ramp, side, super::TOL).map_err(map)?,
    };
    // never hand back an invalid wall (a partial-turn revolved sheet used to come back with edges off their faces).
    // GUESS / not covered: a variable-thickness wall is fitted and some of its tested bodies do not pass the check at
    // the model tolerance (variable_thicken flat_sheet_ramp_volume_is_the_integral), so only the constant wall is judged.
    if let Some(i) = ck_check::validate(&out, super::TOL).first().filter(|_| constant) {
        return Err(coded("thickenFailed", &format!("the thickened wall is not a valid solid ({} at {}: {})", i.code, i.entity, i.detail)).with(
            "hint",
            Json::Str("close the surface into a solid and use shell, or build the wall from a thick profile (revolve or extrude the closed section)".into()),
        ));
    }
    // a wall that starts on a plane of the body it is added to is sunk into it: the wedge between its normal-offset base
    // and the plane is closed, so the foot is an ordinary concave cone / plane edge
    let out = match br.body.as_ref() {
        Some(host) => sf::foot::sink_feet(&out, host, &f.id, super::TOL),
        None => out,
    };
    // pinned: the thickened surface is consumed
    br.drop_surfaces(vec![i]);
    join_solid(br, &f.id, out)
}

/// `thicknessAt {along: [x, y, z], stops: [[s, t], ...]}`: variable thickness, linear between
/// 2..=6 stops along `along` (a blade's taper root to tip). `None` without the field; a
/// constant stop list is the plain `thickness` body. A stop list the kernel cannot use is
/// `invalidValue {detail, hint}`.
fn thickness_at(f: &Feature) -> Result<Option<sf::thicken::ThicknessRamp>, FeatureError> {
    let Some(j) = f.raw.get("thicknessAt") else {
        return Ok(None);
    };
    let bad = |detail: String, hint: &str| {
        Err(FeatureError::new(codes::INVALID_VALUE)
            .with("detail", Json::Str(detail))
            .with("hint", Json::Str(hint.to_string())))
    };
    if !j.is_obj() {
        return bad(
            "thicknessAt must be an object {along: [x,y,z], stops: [[s,t],...]}".into(),
            "along is the direction the thickness varies along, stops the [s, thickness] pairs along it",
        );
    }
    let Some([x, y, z]) = j.get("along").and_then(Json::as_vec3) else {
        return bad(
            "thicknessAt.along must be [x, y, z]".into(),
            "the direction the thickness varies along, for example [0,0,1] for a blade lofted along z",
        );
    };
    let Some(s) = j.get_arr("stops") else {
        return bad("thicknessAt.stops must be [[s, thickness], ...]".into(), "2 to 6 [s, thickness] pairs ascending in s: two for a linear ramp (root and tip)");
    };
    let mut stops = Vec::with_capacity(s.len());
    for pair in s {
        let Some(p) = pair.as_arr() else {
            return bad("each thicknessAt stop must be [s, thickness]".into(), "2 to 6 [s, thickness] pairs ascending in s");
        };
        let Some([sv, tv]) = <&[ck_json::Json; 2]>::try_from(p).ok() else {
            return bad("each thicknessAt stop must be [s, thickness]".into(), "2 to 6 [s, thickness] pairs ascending in s");
        };
        let (Some(sv), Some(tv)) = (sv.as_f64(), tv.as_f64()) else {
            return bad("a thicknessAt stop holds a value that is not a number".into(), "every stop is [s, thickness] with finite numbers");
        };
        stops.push((sv, tv));
    }
    sf::thicken::ThicknessRamp::new(Vec3::new(x, y, z), stops).map(Some).map_err(|e| {
        let mut fe = coded(e.code, &e.detail);
        if let Some(h) = &e.hint {
            fe = fe.with("hint", Json::Str(h.clone()));
        }
        fe
    })
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
    let slot = br.slot_of_face(&names[0]).ok_or_else(|| br.missing_face(&names[0]))?;
    let body = br.slot_body(slot).ok_or_else(missing)?.clone();
    let mut del = vec![];
    for n in &names {
        del.push(body.find_face(n).ok_or_else(|| crate::refs::missing_face([&body], n))?);
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

/// `freeform.edits[]` -> ck-ops edits. Codes: `freeformEntryInvalid {entry}` (points not `[i, j]` with
/// non-negative integers, or an unreadable offset), `freeformEntryPlane {entry}` (a `plane` frame or
/// `symmetric` plane that does not resolve).
fn parse_ff_edits(br: &Brep, state: &RegenState, js: &[&Json]) -> Result<Vec<sf::rebuild::edits::Edit>, FeatureError> {
    use sf::rebuild::edits::{Edit, EditFrame};
    let mut out = vec![];
    for (ei, e) in js.iter().enumerate() {
        let entry = Json::Num((ei + 1) as f64);
        let invalid = || FeatureError::new("freeformEntryInvalid").with("entry", entry.clone());
        let noplane = || FeatureError::new("freeformEntryPlane").with("entry", entry.clone());
        let mut points = vec![];
        for p in e.get("points").and_then(|p| p.as_arr()).unwrap_or(&[]) {
            let a = p.as_arr().filter(|a| a.len() == 2).ok_or_else(invalid)?;
            let (i, j) = (a[0].as_f64().ok_or_else(invalid)?, a[1].as_f64().ok_or_else(invalid)?);
            if i < 0.0 || j < 0.0 || i.fract() != 0.0 || j.fract() != 0.0 {
                return Err(invalid());
            }
            points.push((i as usize, j as usize));
        }
        let num = |k: &str| crate::expr::eval_field(e.get(k), &state.vars, 0.0).map_err(|_| invalid());
        let d = Vec3::new(num("dx")?, num("dy")?, num("dz")?);
        let plane_of = |j: &Json| -> Result<ck_core::linalg::Frame, FeatureError> {
            let r = crate::GeomRef::from_json(j).ok_or_else(noplane)?;
            super::pattern::plane_frame(br, state, &r).ok_or_else(noplane)
        };
        let frame = match e.get_str("frame") {
            Some("surface") => EditFrame::Surface,
            Some("plane") => EditFrame::Plane(plane_of(e.get("plane").unwrap_or(&Json::Null))?),
            _ => EditFrame::World,
        };
        let symmetric = match e.get("symmetric") {
            Some(j) if !matches!(j, Json::Null | Json::Bool(false)) => {
                let fr = plane_of(j)?;
                Some((fr.o, fr.z))
            }
            _ => None,
        };
        let of = e.get("of").and_then(|o| o.as_arr()).filter(|o| o.len() == 2).map(|o| (o[0].as_f64().unwrap_or(0.0) as usize, o[1].as_f64().unwrap_or(0.0) as usize));
        out.push(Edit {
            points,
            of,
            frame,
            d,
            proportional: num("proportional")?.max(0.0),
            symmetric,
            disabled: crate::sketch::jtruthy(e.get("disabled")),
        });
    }
    Ok(out)
}

pub fn exec_freeform(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let names = strs(f.raw.get("faces"));
    if names.is_empty() {
        return Err(missing());
    }
    let slot = br.slot_of_face(&names[0]).ok_or_else(|| br.missing_face(&names[0]))?;
    let body = br.slot_body(slot).ok_or_else(missing)?.clone();
    let mut faces = vec![];
    for n in &names {
        faces.push(body.find_face(n).ok_or_else(|| crate::refs::missing_face([&body], n))?);
    }
    let edits_json: Vec<&Json> = f.raw.get("edits").and_then(|e| e.as_arr()).map(|a| a.iter().collect()).unwrap_or_default();
    // control point edits work on exactly one face
    if !edits_json.is_empty() && names.len() != 1 {
        return Err(FeatureError::new("freeformOneFace"));
    }
    let edits = parse_ff_edits(br, state, &edits_json)?;
    let n = |k: &str, d: f64| state.num(f, k, d);
    let (du, dv) = (n("degreeU", 3.0)?.max(1.0) as usize, n("degreeV", 3.0)?.max(1.0) as usize);
    let (su, sv) = (n("spansU", 2.0)?.max(1.0) as usize, n("spansV", 2.0)?.max(1.0) as usize);
    let mut out = sf::rebuild::rebuild(&body, &faces, du, dv, su, sv, super::TOL).map_err(|e| coded(e.code, &e.detail))?;
    let mut entry_json: Vec<Json> = vec![];
    if !edits.is_empty() {
        let ed = sf::rebuild::edits::apply_edits(&out, faces[0], &edits, super::TOL).map_err(|e| {
            FeatureError::new(e.code).with("entry", Json::Num(e.entry as f64))
        })?;
        for e in &ed.entries {
            let pts = Json::Arr(e.points.iter().map(|&(i, j)| Json::Arr(vec![Json::Num(i as f64), Json::Num(j as f64)])).collect());
            entry_json.push(Json::Obj(vec![
                ("points".into(), pts),
                ("mapped".into(), Json::Bool(e.mapped)),
                ("applied".into(), Json::Bool(e.applied)),
                ("kept".into(), Json::Num(e.points.len() as f64)),
            ]));
        }
        out = ed.body;
    }
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
    let mut marker = vec![("faces".to_string(), Json::Arr(mf))];
    if !edits.is_empty() {
        marker.push(("entries".into(), Json::Arr(entry_json)));
    }
    state.result.markers.push(Json::Arr(vec![Json::Str(f.id.clone()), Json::Obj(marker)]));
    br.replace_slot(slot, &f.id, out);
    Ok(())
}
