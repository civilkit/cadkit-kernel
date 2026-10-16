//! Regenerate a recognised feature list into a body, and compare it with the original.

use ck_json::{Json, Obj};
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use ck_topo::body::Body;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}

struct Exec {
    brep: Brep,
    labels: Noop,
}

impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn edge_line(&mut self, s: &regen::RegenState, faces: &[String], at: Option<[f64; 3]>) -> Option<(ck_core::linalg::Vec3, ck_core::linalg::Vec3)> {
        self.brep.edge_line(s, faces, at)
    }
    fn edge_seg(&mut self, s: &regen::RegenState, faces: &[String], at: Option<ck_core::linalg::Vec3>) -> Option<ck_ops::sweep::PathSeg> {
        self.brep.edge_seg(s, faces, at)
    }
    fn vertex_point(&mut self, s: &regen::RegenState, faces: &[String], at: Option<ck_core::linalg::Vec3>) -> Option<ck_core::linalg::Vec3> {
        self.brep.vertex_point(s, faces, at)
    }
    fn face_surf(&mut self, s: &regen::RegenState, face: &str) -> Option<regen::FaceSurf> {
        self.brep.face_surf(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

/// Regenerate `features`; returns the body (if any) and the error codes (`feature: code`).
pub fn rebuild(features: &[Json]) -> Result<(Option<Body>, Vec<String>), String> {
    let part_json = Obj::new().put("format", "ck-part").put("version", 1.0).put("features", Json::Arr(features.to_vec())).build();
    let part = Part::from_json(&part_json)?;
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.code)).collect();
    Ok((ex.brep.body, errs))
}

pub struct Verify {
    pub vol_orig: f64,
    pub vol_new: f64,
    pub sym_diff: Option<f64>,
    pub rel: f64,
    pub bbox_dev: f64,
    pub area_rel: f64,
    pub exact: bool,
    pub close: bool,
}

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn cut_volume(a: &Body, b: &Body) -> Option<f64> {
    match ck_ops::boolean::boolean(a, b, ck_ops::boolean::Op::Cut, 1e-6) {
        Ok(o) => {
            if o.no_effect {
                Some(vol(a))
            } else if o.body.num_faces() == 0 {
                Some(0.0)
            } else {
                Some(vol(&o.body).abs())
            }
        }
        Err(_) => None,
    }
}

pub fn verify(orig: &Body, new: &Body) -> Verify {
    let (vo, vn) = (vol(orig), vol(new));
    let (ao, an) = (ck_topo::mass::mass_props(orig).area, ck_topo::mass::mass_props(new).area);
    let sym = match (cut_volume(orig, new), cut_volume(new, orig)) {
        (Some(x), Some(y)) => Some(x + y),
        _ => None,
    };
    let (b0, b1) = (orig.bbox(), new.bbox());
    let bbox_dev = (b0.0 - b1.0).len().max((b0.1 - b1.1).len());
    let area_rel = (ao - an).abs() / ao.abs().max(1e-12);
    let dv = (vo - vn).abs() / vo.abs().max(1e-12);
    let rel = match sym {
        Some(s) => s / vo.abs().max(1e-12),
        None => dv.max(area_rel),
    };
    let ok = |t: f64| rel < t && dv < t.max(1e-6) && bbox_dev < 1e-3 * (1.0 + (b0.1 - b0.0).len());
    Verify { vol_orig: vo, vol_new: vn, sym_diff: sym, rel, bbox_dev, area_rel, exact: ok(1e-6) && area_rel < 1e-6, close: ok(1e-3) }
}

impl Verify {
    pub fn json(&self) -> Json {
        Obj::new()
            .put("volume", self.vol_new)
            .put("volumeOriginal", self.vol_orig)
            .put("symDiffVolume", self.sym_diff.map_or(Json::Null, Json::Num))
            .put("relativeError", self.rel)
            .put("areaRelativeError", self.area_rel)
            .put("bboxDeviation", self.bbox_dev)
            .put("exact", self.exact)
            .put("close", self.close)
            .build()
    }
}
