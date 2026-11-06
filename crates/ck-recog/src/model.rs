//! Face classification of the imported body: exact surface kind, outward sense, adjacency helpers.

use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId};

/// Coincidence tolerance (model units).
pub const TOL: f64 = 1e-5;

#[derive(Debug, Clone)]
pub enum Kind {
    /// Outward unit normal `n` and offset `d` (`n . p = d`).
    Plane { n: Vec3, d: f64 },
    /// Axis point, unit axis, radius; `convex` = the outward normal points away from the axis.
    Cyl { o: Vec3, a: Vec3, r: f64, convex: bool },
    /// Frame origin (radius `r0` there), unit axis, half angle, convex flag.
    Cone { o: Vec3, a: Vec3, r0: f64, half: f64, convex: bool },
    Sphere { c: Vec3, r: f64, convex: bool },
    Torus { c: Vec3, a: Vec3, major: f64, minor: f64, convex: bool },
    Other(&'static str),
}

impl Kind {
    pub fn name(&self) -> &'static str {
        match self {
            Kind::Plane { .. } => "plane",
            Kind::Cyl { .. } => "cylinder",
            Kind::Cone { .. } => "cone",
            Kind::Sphere { .. } => "sphere",
            Kind::Torus { .. } => "torus",
            Kind::Other(s) => s,
        }
    }
}

pub struct Model<'a> {
    pub b: &'a Body,
    pub faces: Vec<FaceId>,
    kinds: Vec<Option<Kind>>,
}

impl<'a> Model<'a> {
    /// A copy of the model in which the surface of the given faces is another (a face as it was before a later
    /// feature moved it); the body, and so the edges, stay the same.
    pub fn with_kinds(&self, over: &[(FaceId, Kind)]) -> Model<'a> {
        let mut kinds = self.kinds.clone();
        for (f, k) in over {
            kinds[f.idx()] = Some(k.clone());
        }
        Model { b: self.b, faces: self.faces.clone(), kinds }
    }

    pub fn new(b: &'a Body) -> Model<'a> {
        let faces: Vec<FaceId> = b.face_ids().collect();
        let mut kinds: Vec<Option<Kind>> = vec![None; b.faces.len()];
        for &f in &faces {
            let face = b.face(f);
            let k = match &face.surface {
                Surface::Plane { frame } => {
                    let n = b.planar_normal(f).normalize();
                    Kind::Plane { n, d: n.dot(frame.o) }
                }
                Surface::Cylinder { frame, r } => Kind::Cyl { o: frame.o, a: frame.z.normalize(), r: *r, convex: face.sense },
                Surface::Cone { frame, r, semi_angle } => {
                    Kind::Cone { o: frame.o, a: frame.z.normalize(), r0: *r, half: *semi_angle, convex: face.sense }
                }
                Surface::Sphere { frame, r } => Kind::Sphere { c: frame.o, r: *r, convex: face.sense },
                Surface::Torus { frame, major, minor } => {
                    Kind::Torus { c: frame.o, a: frame.z.normalize(), major: *major, minor: *minor, convex: face.sense }
                }
                s => {
                    if s.is_planar(1e-7) {
                        let n = b.planar_normal(f).normalize();
                        let p = b.vertex(b.edge(b.face_edges(f)[0]).v0).p;
                        Kind::Plane { n, d: n.dot(p) }
                    } else {
                        Kind::Other(match s {
                            Surface::BSpline(_) => "spline",
                            Surface::Extrusion { .. } => "extrusion",
                            Surface::Revolution { .. } => "revolution",
                            Surface::Offset { .. } => "offset",
                            _ => "other",
                        })
                    }
                }
            };
            kinds[f.idx()] = Some(k);
        }
        Model { b, faces, kinds }
    }

    pub fn kind(&self, f: FaceId) -> &Kind {
        self.kinds[f.idx()].as_ref().expect("live face")
    }

    /// Non-seam edges of a face.
    pub fn edges(&self, f: FaceId) -> Vec<EdgeId> {
        self.b.face_edges(f).into_iter().filter(|&e| !self.b.is_seam(e)).collect()
    }

    /// The other face across `e` (None for seams, open or non-manifold edges).
    pub fn across(&self, e: EdgeId, f: FaceId) -> Option<FaceId> {
        let fs = self.b.edge_faces(e);
        if fs.len() == 2 {
            Some(if fs[0] == f { fs[1] } else { fs[0] })
        } else {
            None
        }
    }

    pub fn area(&self, f: FaceId) -> f64 {
        ck_topo::mass::faces_moments(self.b, &[f], Vec3::ZERO).area.abs()
    }

    pub fn p0(&self, e: EdgeId) -> Vec3 {
        self.b.vertex(self.b.edge(e).v0).p
    }
    pub fn p1(&self, e: EdgeId) -> Vec3 {
        self.b.vertex(self.b.edge(e).v1).p
    }
    pub fn mid(&self, e: EdgeId) -> Vec3 {
        self.b.edge_mid(e)
    }
    pub fn curve(&self, e: EdgeId) -> &Curve {
        &self.b.edge(e).curve
    }
}

/// `Some((index, sign))` when `v` is a unit principal axis (+-X, +-Y, +-Z).
pub fn principal(v: Vec3) -> Option<(usize, f64)> {
    let a = [v.x, v.y, v.z];
    for i in 0..3 {
        if a[i].abs() > 1.0 - 1e-7 {
            return Some((i, a[i].signum()));
        }
    }
    None
}

pub fn par(a: Vec3, b: Vec3) -> bool {
    a.cross(b).len() < 1e-6
}

pub fn dist_to_line(p: Vec3, o: Vec3, a: Vec3) -> f64 {
    (p - o).cross(a).len()
}

pub fn axis_vec(i: usize) -> Vec3 {
    match i {
        0 => Vec3::X,
        1 => Vec3::Y,
        _ => Vec3::Z,
    }
}

pub fn comp(v: Vec3, i: usize) -> f64 {
    match i {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

/// Sketch plane description for a principal normal axis: `std` plane name and the world directions of the
/// sketch x, y and normal.
#[derive(Debug, Clone, Copy)]
pub struct PlaneSpec {
    pub std: &'static str,
    pub xs: Vec3,
    pub ys: Vec3,
    pub zs: Vec3,
}

impl PlaneSpec {
    /// The std plane whose normal is the positive world axis `i`.
    pub fn for_axis(i: usize) -> PlaneSpec {
        match i {
            0 => PlaneSpec { std: "right", xs: Vec3::new(0.0, 0.0, -1.0), ys: Vec3::Y, zs: Vec3::X },
            1 => PlaneSpec { std: "top", xs: Vec3::X, ys: Vec3::new(0.0, 0.0, -1.0), zs: Vec3::Y },
            _ => PlaneSpec { std: "front", xs: Vec3::X, ys: Vec3::Y, zs: Vec3::Z },
        }
    }
    pub fn xy(&self, p: Vec3) -> (f64, f64) {
        (p.dot(self.xs), p.dot(self.ys))
    }
    pub fn level(&self, p: Vec3) -> f64 {
        p.dot(self.zs)
    }
}
