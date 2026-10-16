//! Phase B (part 1): Parasolid XT neutral-binary transmit reader.
//!
//! Physical layout (see the open sources listed in THIRD_PARTY.md; the structure of the reader, the base schema
//! SCH_13006 field tables and the edit-script rules follow BlinkingSun/sldprt2step, Apache-2.0):
//!
//! * header `PS 00 00`, short-prefixed modeller string, int-prefixed schema name `SCH_<modeller>_<schema>_<base>`, a
//!   short (max node types), an int (user field size, must be 0);
//! * nodes: u16 type; on the first occurrence of a type the embedded schema info (`0xFF` = identical to the base
//!   schema, else an edit script `C`opy / `D`elete / `I`nsert / `A`ppend ... `Z` against the base layout, or a full
//!   definition for types the base does not know); for variable-length types an i32 element count; the node index;
//!   then the fields;
//! * terminator: type 1 followed by index 0;
//! * pointers and positive integers: i16 `r`; if negative a second i16 `q` follows; value = q*32767 + |r| - 1.
//!   Null markers: int -32764, double -3.14158e13.
//!
//! Every read is bounds-checked; errors are structured (`xt.*`).

use crate::error::{Limits, SldprtError, R};
use std::collections::HashMap;
use std::rc::Rc;

pub const VAR: i32 = -1;

#[derive(Debug, Clone)]
pub struct Field {
    pub name: String,
    pub kind: u8,
    pub n: i32,
}

fn f(name: &str, kind: char, n: i32) -> Field {
    Field { name: name.to_string(), kind: kind as u8, n }
}
fn f1(name: &str, kind: char) -> Field {
    f(name, kind, 1)
}

pub type Layout = Vec<Field>;

#[derive(Debug, Clone, PartialEq)]
pub enum Val {
    I(i64),
    F(f64),
    B(bool),
    C(u8),
    V3([f64; 3]),
    V2([f64; 2]),
    V6([f64; 6]),
    AI(Vec<i64>),
    AF(Vec<f64>),
    AV(Vec<[f64; 3]>),
    AB(Vec<bool>),
    AS(Vec<u8>),
}

#[derive(Debug, Clone)]
pub struct Node {
    pub index: i64,
    pub ty: u16,
    layout: Rc<Layout>,
    vals: Vec<Val>,
}

impl Node {
    pub fn val(&self, name: &str) -> Option<&Val> {
        self.layout.iter().position(|x| x.name == name).map(|i| &self.vals[i])
    }
    /// Integer / pointer field (0 when absent).
    pub fn i(&self, name: &str) -> i64 {
        match self.val(name) {
            Some(Val::I(v)) => *v,
            Some(Val::C(c)) => *c as i64,
            Some(Val::B(b)) => *b as i64,
            _ => 0,
        }
    }
    pub fn f(&self, name: &str) -> f64 {
        match self.val(name) {
            Some(Val::F(v)) => *v,
            _ => 0.0,
        }
    }
    pub fn b(&self, name: &str) -> bool {
        matches!(self.val(name), Some(Val::B(true)))
    }
    pub fn c(&self, name: &str) -> u8 {
        match self.val(name) {
            Some(Val::C(c)) => *c,
            _ => 0,
        }
    }
    /// `+` sense chars.
    pub fn plus(&self, name: &str) -> bool {
        self.c(name) == b'+'
    }
    pub fn v3(&self, name: &str) -> [f64; 3] {
        match self.val(name) {
            Some(Val::V3(v)) => *v,
            _ => [0.0; 3],
        }
    }
    pub fn af(&self, name: &str) -> &[f64] {
        match self.val(name) {
            Some(Val::AF(v)) => v,
            _ => &[],
        }
    }
    pub fn ai(&self, name: &str) -> &[i64] {
        match self.val(name) {
            Some(Val::AI(v)) => v,
            _ => &[],
        }
    }
    pub fn av(&self, name: &str) -> &[[f64; 3]] {
        match self.val(name) {
            Some(Val::AV(v)) => v,
            _ => &[],
        }
    }
    pub fn pair(&self, name: &str) -> [f64; 2] {
        match self.val(name) {
            Some(Val::V2(v)) => *v,
            Some(Val::AF(v)) if v.len() >= 2 => [v[0], v[1]],
            _ => [0.0; 2],
        }
    }
    /// All fields with their names (debugging / census tools).
    pub fn dump(&self) -> Vec<(&str, &Val)> {
        self.layout.iter().zip(self.vals.iter()).map(|(f, v)| (f.name.as_str(), v)).collect()
    }
    pub fn name(&self) -> String {
        node_name(self.ty).map(|s| s.to_string()).unwrap_or_else(|| format!("TYPE_{}", self.ty))
    }
}

#[derive(Debug, Default)]
pub struct XtFile {
    pub modeller: String,
    pub schema_id: String,
    pub partition: bool,
    pub nodes: HashMap<i64, Node>,
    pub root_index: i64,
    pub trailing_bytes: usize,
    pub schema_log: Vec<String>,
}

impl XtFile {
    pub fn get(&self, index: i64) -> Option<&Node> {
        if index <= 0 {
            None
        } else {
            self.nodes.get(&index)
        }
    }
    /// Node referenced by pointer field `field` of `n`.
    pub fn deref(&self, n: &Node, field: &str) -> Option<&Node> {
        self.get(n.i(field))
    }
    /// Follow `next_field` from `head` (cycle- and length-safe).
    pub fn chain<'a>(&'a self, head: Option<&'a Node>, next_field: &str) -> Vec<&'a Node> {
        let mut out = vec![];
        let mut seen = std::collections::HashSet::new();
        let mut cur = head;
        while let Some(n) = cur {
            if !seen.insert(n.index) || out.len() > 5_000_000 {
                break;
            }
            out.push(n);
            cur = self.deref(n, next_field);
        }
        out
    }
    pub fn census(&self) -> Vec<(String, usize)> {
        let mut m: std::collections::BTreeMap<String, usize> = Default::default();
        for n in self.nodes.values() {
            *m.entry(n.name()).or_default() += 1;
        }
        m.into_iter().collect()
    }
}

pub fn node_name(t: u16) -> Option<&'static str> {
    Some(match t {
        10 => "ASSEMBLY", 11 => "INSTANCE", 12 => "BODY", 13 => "SHELL", 14 => "FACE", 15 => "LOOP", 16 => "EDGE", 17 => "HALFEDGE",
        18 => "VERTEX", 19 => "REGION", 29 => "POINT", 30 => "LINE", 31 => "CIRCLE", 32 => "ELLIPSE", 38 => "INTERSECTION", 40 => "CHART",
        41 => "LIMIT", 45 => "BSPLINE_VERTICES", 50 => "PLANE", 51 => "CYLINDER", 52 => "CONE", 53 => "SPHERE", 54 => "TORUS",
        56 => "BLENDED_EDGE", 59 => "BLEND_BOUND", 60 => "OFFSET_SURF", 67 => "SWEPT_SURF", 68 => "SPUN_SURF", 70 => "LIST",
        74 => "POINTER_LIS_BLOCK", 79 => "ATT_DEF_ID", 80 => "ATTRIB_DEF", 81 => "ATTRIBUTE", 82 => "INT_VALUES", 83 => "REAL_VALUES",
        84 => "CHAR_VALUES", 85 => "POINT_VALUES", 86 => "VECTOR_VALUES", 87 => "AXIS_VALUES", 88 => "TAG_VALUES",
        89 => "DIRECTION_VALUES", 90 => "FEATURE", 91 => "MEMBER_OF_FEATURE", 98 => "UNICODE_VALUES", 99 => "FIELD_NAMES",
        100 => "TRANSFORM", 101 => "WORLD", 102 => "KEY", 120 => "PE_SURF", 121 => "INT_PE_DATA", 122 => "EXT_PE_DATA",
        124 => "B_SURFACE", 125 => "SURFACE_DATA", 126 => "NURBS_SURF", 127 => "KNOT_MULT", 128 => "KNOT_SET", 130 => "PE_CURVE",
        133 => "TRIMMED_CURVE", 134 => "B_CURVE", 135 => "CURVE_DATA", 136 => "NURBS_CURVE", 137 => "SP_CURVE", 141 => "GEOMETRIC_OWNER",
        163 => "HELIX_SU_FORM", 176 => "PART_XMT_BLOCK", 184 => "HELIX_CU_FORM", 185 => "POLYLINE_DATA", 189 => "PSM_MESH",
        200 => "POLYLINE", 201 => "MESH", 204 => "INTERSECTION_DATA", 229 => "TRANSFORM_PRECISION",
        _ => return None,
    })
}

fn common() -> Layout {
    vec![f1("node_id", 'd'), f1("attributes_features", 'p'), f1("owner", 'p'), f1("next", 'p'), f1("previous", 'p'), f1("geometric_owner", 'p'), f1("sense", 'c')]
}

fn cat(mut a: Layout, b: Layout) -> Layout {
    a.extend(b);
    a
}

/// Base schema SCH_13006 layouts. Types not listed are carried in full by the file. Fields added after V13 are not here:
/// files deliver them as embedded edits.
pub fn base_layout(t: u16) -> Option<Layout> {
    let p = |n: &str| f1(n, 'p');
    let v = |n: &str| f1(n, 'v');
    let fl = |n: &str| f1(n, 'f');
    let d = |n: &str| f1(n, 'd');
    Some(match t {
        10 => vec![d("highest_node_id"), p("attributes_features"), p("attribute_chains"), p("list"), p("surface"), p("curve"), p("point"), p("key"), fl("res_size"), fl("res_linear"), p("ref_instance"), p("next"), p("previous"), f1("state", 'u'), p("owner"), f1("type", 'u'), p("sub_instance")],
        11 => vec![d("node_id"), p("attributes_features"), f1("type", 'u'), p("part"), p("transform"), p("assembly"), p("next_in_part"), p("prev_in_part"), p("next_of_part"), p("prev_of_part")],
        12 => vec![d("highest_node_id"), p("attributes_features"), p("attribute_chains"), p("surface"), p("curve"), p("point"), p("key"), fl("res_size"), fl("res_linear"), p("ref_instance"), p("next"), p("previous"), f1("state", 'u'), p("owner"), f1("body_type", 'u'), f1("nom_geom_state", 'u'), p("shell"), p("boundary_surface"), p("boundary_curve"), p("boundary_point"), p("region"), p("edge"), p("vertex")],
        13 => vec![d("node_id"), p("attributes_features"), p("body"), p("next"), p("face"), p("edge"), p("vertex"), p("region"), p("front_face")],
        14 => vec![d("node_id"), p("attributes_features"), fl("tolerance"), p("next"), p("previous"), p("loop"), p("shell"), p("surface"), f1("sense", 'c'), p("next_on_surface"), p("previous_on_surface"), p("next_front"), p("previous_front"), p("front_shell")],
        15 => vec![d("node_id"), p("attributes_features"), p("halfedge"), p("face"), p("next")],
        16 => vec![d("node_id"), p("attributes_features"), fl("tolerance"), p("halfedge"), p("previous"), p("next"), p("curve"), p("next_on_curve"), p("previous_on_curve"), p("owner")],
        17 => vec![p("attributes_features"), p("loop"), p("forward"), p("backward"), p("vertex"), p("other"), p("edge"), p("curve"), p("next_at_vx"), f1("sense", 'c')],
        18 => vec![d("node_id"), p("attributes_features"), p("halfedge"), p("previous"), p("next"), p("point"), fl("tolerance"), p("owner")],
        19 => vec![d("node_id"), p("attributes_features"), p("body"), p("next"), p("previous"), p("shell"), f1("type", 'c')],
        29 => vec![d("node_id"), p("attributes_features"), p("owner"), p("next"), p("previous"), v("pvec")],
        30 => cat(common(), vec![v("pvec"), v("direction")]),
        31 => cat(common(), vec![v("centre"), v("normal"), v("x_axis"), fl("radius")]),
        32 => cat(common(), vec![v("centre"), v("normal"), v("x_axis"), fl("major_radius"), fl("minor_radius")]),
        38 => cat(common(), vec![f("surface", 'p', 2), p("chart"), p("start"), p("end")]),
        40 => vec![fl("base_parameter"), fl("base_scale"), d("chart_count"), fl("chordal_error"), fl("angular_error"), f("parameter_error", 'f', 2), f("hvec", 'h', VAR)],
        41 => vec![f1("type", 'c'), f("hvec", 'h', VAR)],
        45 => vec![f("vertices", 'f', VAR)],
        50 => cat(common(), vec![v("pvec"), v("normal"), v("x_axis")]),
        51 => cat(common(), vec![v("pvec"), v("axis"), fl("radius"), v("x_axis")]),
        52 => cat(common(), vec![v("pvec"), v("axis"), fl("radius"), fl("sin_half_angle"), fl("cos_half_angle"), v("x_axis")]),
        53 => cat(common(), vec![v("centre"), fl("radius"), v("axis"), v("x_axis")]),
        54 => cat(common(), vec![v("centre"), v("axis"), fl("major_radius"), fl("minor_radius"), v("x_axis")]),
        56 => cat(common(), vec![f1("blend_type", 'c'), f("surface", 'p', 2), p("spine"), f("range", 'f', 2), f("thumb_weight", 'f', 2), f("boundary", 'p', 2), p("start"), p("end")]),
        59 => cat(common(), vec![f1("boundary", 'n'), p("blend")]),
        60 => cat(common(), vec![f1("check", 'c'), f1("true_offset", 'l'), p("surface"), fl("offset"), fl("scale")]),
        67 => cat(common(), vec![p("section"), v("sweep"), fl("scale")]),
        68 => cat(common(), vec![p("profile"), v("base"), v("axis"), v("start"), v("end"), fl("start_param"), fl("end_param"), v("x_axis"), fl("scale")]),
        70 => vec![d("node_id"), p("owner"), p("next"), p("previous"), f1("list_type", 'u'), d("list_length"), d("block_length"), d("size_of_entry"), p("list_block")],
        74 => vec![d("n_entries"), p("next_block"), f("entries", 'p', VAR)],
        79 => vec![f("string", 'c', VAR)],
        80 => vec![p("next"), p("identifier"), d("type_id"), f("actions", 'u', 8), p("field_names"), f("legal_owners", 'l', 14), f("fields", 'u', VAR)],
        81 => vec![d("node_id"), p("definition"), p("owner"), p("next"), p("previous"), p("next_of_type"), p("previous_of_type"), f("fields", 'p', VAR)],
        82 => vec![f("values", 'd', VAR)],
        83 => vec![f("values", 'f', VAR)],
        84 => vec![f("values", 'c', VAR)],
        85 | 86 | 87 | 89 => vec![f("values", 'v', VAR)],
        88 => vec![f("values", 't', VAR)],
        90 => vec![d("node_id"), p("attributes_features"), p("owner"), p("next"), p("previous"), f1("type", 'u'), p("first_member")],
        91 => vec![d("dummy_node_id"), p("owning_feature"), p("owner"), p("next"), p("previous"), p("next_member"), p("previous_member")],
        98 => vec![f("values", 'w', VAR)],
        99 => vec![f("names", 'p', VAR)],
        100 => vec![d("node_id"), p("owner"), p("next"), p("previous"), f("rotation_matrix", 'f', 9), v("translation_vector"), fl("scale"), d("flag"), v("perspective_vector")],
        101 => vec![p("assembly"), p("attribute"), p("body"), p("transform"), p("surface"), p("curve"), p("point"), f1("alive", 'l'), p("attrib_def"), d("highest_id"), d("current_id")],
        102 => vec![f("string", 'c', VAR)],
        120 => cat(common(), vec![f1("type", 'c'), p("data"), p("tf"), f("internal_geom", 'p', VAR)]),
        121 => vec![d("geom_type"), p("real_array"), p("int_array")],
        122 => vec![p("key"), p("real_array"), p("int_array")],
        124 => cat(common(), vec![p("nurbs"), p("data")]),
        125 => vec![f1("original_uint", 'i'), f1("original_vint", 'i'), f1("extended_uint", 'i'), f1("extended_vint", 'i'), f1("self_int", 'u'), f1("original_u_start", 'c'), f1("original_u_end", 'c'), f1("original_v_start", 'c'), f1("original_v_end", 'c'), f1("extended_u_start", 'c'), f1("extended_u_end", 'c'), f1("extended_v_start", 'c'), f1("extended_v_end", 'c'), f1("analytic_form_type", 'c'), f1("swept_form_type", 'c'), f1("spun_form_type", 'c'), f1("blend_form_type", 'c'), p("analytic_form"), p("swept_form"), p("spun_form"), p("blend_form")],
        126 => vec![f1("u_periodic", 'l'), f1("v_periodic", 'l'), f1("u_degree", 'n'), f1("v_degree", 'n'), d("n_u_vertices"), d("n_v_vertices"), f1("u_knot_type", 'u'), f1("v_knot_type", 'u'), d("n_u_knots"), d("n_v_knots"), f1("rational", 'l'), f1("u_closed", 'l'), f1("v_closed", 'l'), f1("surface_form", 'u'), f1("vertex_dim", 'n'), p("bspline_vertices"), p("u_knot_mult"), p("v_knot_mult"), p("u_knots"), p("v_knots")],
        127 => vec![f("mult", 'n', VAR)],
        128 => vec![f("knots", 'f', VAR)],
        130 => cat(common(), vec![f1("type", 'c'), p("data"), p("tf"), f("internal_geom", 'p', VAR)]),
        133 => cat(common(), vec![p("basis_curve"), v("point_1"), v("point_2"), fl("parm_1"), fl("parm_2")]),
        134 => cat(common(), vec![p("nurbs"), p("data")]),
        135 => vec![f1("self_int", 'u'), p("analytic_form")],
        136 => vec![f1("degree", 'n'), d("n_vertices"), f1("vertex_dim", 'n'), d("n_knots"), f1("knot_type", 'u'), f1("periodic", 'l'), f1("closed", 'l'), f1("rational", 'l'), f1("curve_form", 'u'), p("bspline_vertices"), p("knot_mult"), p("knots")],
        137 => cat(common(), vec![p("surface"), p("b_curve"), p("original"), fl("tolerance_to_original")]),
        141 => vec![p("owner"), p("next"), p("previous"), p("shared_geometry")],
        163 => vec![v("axis_pt"), v("axis_dir"), f1("hand", 'c'), f1("turns", 'i'), fl("pitch"), fl("gap"), fl("tol")],
        184 => vec![v("axis_pt"), v("axis_dir"), v("point"), f1("hand", 'c'), f1("turns", 'i'), fl("pitch"), fl("tol")],
        _ => return None,
    })
}

fn is_variable(l: &Layout) -> bool {
    l.last().map_or(false, |x| x.n == VAR)
}

fn xerr(msg: impl Into<String>) -> SldprtError {
    SldprtError::new("xt.stream", msg)
}

struct Rd<'a> {
    b: &'a [u8],
    pos: usize,
}

impl<'a> Rd<'a> {
    fn raw(&mut self, k: usize) -> R<&'a [u8]> {
        let end = self.pos.checked_add(k).filter(|&e| e <= self.b.len()).ok_or_else(|| xerr(format!("truncated stream at offset {} (need {k} bytes)", self.pos)))?;
        let s = &self.b[self.pos..end];
        self.pos = end;
        Ok(s)
    }
    fn u8(&mut self) -> R<u8> {
        Ok(self.raw(1)?[0])
    }
    fn i16(&mut self) -> R<i16> {
        let s = self.raw(2)?;
        Ok(i16::from_be_bytes([s[0], s[1]]))
    }
    fn u16(&mut self) -> R<u16> {
        let s = self.raw(2)?;
        Ok(u16::from_be_bytes([s[0], s[1]]))
    }
    fn i32(&mut self) -> R<i32> {
        let s = self.raw(4)?;
        Ok(i32::from_be_bytes([s[0], s[1], s[2], s[3]]))
    }
    fn f64(&mut self) -> R<f64> {
        let s = self.raw(8)?;
        let mut a = [0u8; 8];
        a.copy_from_slice(s);
        Ok(f64::from_be_bytes(a))
    }
    fn index(&mut self) -> R<i64> {
        let mut r = self.i16()? as i64;
        let mut q = 0i64;
        if r < 0 {
            q = self.i16()? as i64;
            r = -r;
        }
        Ok(q * 32767 + r - 1)
    }
    fn short_string(&mut self) -> R<String> {
        let k = self.u8()? as usize;
        Ok(self.raw(k)?.iter().map(|&c| c as char).collect())
    }
}

fn read_field_def(r: &mut Rd) -> R<Field> {
    let name = r.short_string()?;
    let ptr_class = r.u16()?;
    let n_elts = r.index()?;
    let kind = if ptr_class == 0 {
        let t = r.short_string()?;
        t.bytes().last().unwrap_or(b'd')
    } else {
        b'p'
    };
    let n = if n_elts == 1 {
        r.u8()?;
        VAR
    } else if n_elts == 0 {
        1
    } else if n_elts < 0 || n_elts > 1_000_000 {
        return Err(xerr(format!("bad field element count {n_elts}")));
    } else {
        n_elts as i32
    };
    Ok(Field { name, kind, n })
}

fn read_schema_info(r: &mut Rd, ty: u16, log: &mut Vec<String>) -> R<Layout> {
    let first = r.u8()?;
    let tname = node_name(ty).map(String::from).unwrap_or_else(|| format!("TYPE_{ty}"));
    let base = base_layout(ty);
    if first == 0xFF {
        let b = base.ok_or_else(|| xerr(format!("node type {ty} ({tname}) flagged identical to the base schema but unknown")))?;
        log.push(format!("{tname}: base"));
        return Ok(b);
    }
    let nfields = first as usize;
    let nxt = *r.b.get(r.pos).ok_or_else(|| xerr("truncated schema info"))?;
    if let (Some(base), true) = (&base, matches!(nxt, b'C' | b'D' | b'I' | b'A' | b'Z')) {
        let mut out: Layout = vec![];
        let mut bi = 0usize;
        loop {
            let op = r.u8()?;
            match op {
                b'Z' => break,
                b'C' => {
                    out.push(base.get(bi).ok_or_else(|| xerr(format!("{tname}: edit script copies past end of base layout")))?.clone());
                    bi += 1;
                }
                b'D' => {
                    if bi >= base.len() {
                        return Err(xerr(format!("{tname}: edit script deletes past end of base layout")));
                    }
                    bi += 1;
                }
                b'I' | b'A' => out.push(read_field_def(r)?),
                o => return Err(xerr(format!("{tname}: bad edit op 0x{o:02x} at offset {}", r.pos - 1))),
            }
            if out.len() > 4096 {
                return Err(xerr("edit script too long"));
            }
        }
        if out.len() != nfields {
            return Err(xerr(format!("{tname}: edit script yields {} fields, header says {nfields}", out.len())));
        }
        log.push(format!("{tname}: edited ({} fields)", out.len()));
        return Ok(out);
    }
    let _name = r.short_string()?;
    let _desc = r.short_string()?;
    let mut out = vec![];
    for _ in 0..nfields {
        out.push(read_field_def(r)?);
    }
    log.push(format!("{tname}: full definition ({} fields)", out.len()));
    Ok(out)
}

fn read_value(r: &mut Rd, fld: &Field, count: usize) -> R<Val> {
    let scalar = fld.n == 1;
    let n = if scalar { 1 } else { count };
    // guard absurd counts before allocating
    if n > r.b.len() {
        return Err(xerr(format!("array of {n} elements in field {}", fld.name)));
    }
    let k = fld.kind;
    Ok(match k {
        b'p' => {
            if scalar { Val::I(r.index()?) } else { Val::AI((0..n).map(|_| r.index()).collect::<R<_>>()?) }
        }
        b'd' | b't' => {
            if scalar { Val::I(r.i32()? as i64) } else { Val::AI((0..n).map(|_| r.i32().map(|x| x as i64)).collect::<R<_>>()?) }
        }
        b'n' => {
            if scalar { Val::I(r.i16()? as i64) } else { Val::AI((0..n).map(|_| r.i16().map(|x| x as i64)).collect::<R<_>>()?) }
        }
        b'w' => {
            if scalar { Val::I(r.u16()? as i64) } else { Val::AI((0..n).map(|_| r.u16().map(|x| x as i64)).collect::<R<_>>()?) }
        }
        b'u' => {
            if scalar { Val::I(r.u8()? as i64) } else { Val::AI(r.raw(n)?.iter().map(|&b| b as i64).collect()) }
        }
        b'f' => {
            if scalar { Val::F(r.f64()?) } else { Val::AF((0..n).map(|_| r.f64()).collect::<R<_>>()?) }
        }
        b'v' | b'h' => {
            let one = |r: &mut Rd| -> R<[f64; 3]> { Ok([r.f64()?, r.f64()?, r.f64()?]) };
            if scalar { Val::V3(one(r)?) } else { Val::AV((0..n).map(|_| one(r)).collect::<R<_>>()?) }
        }
        b'i' => {
            if scalar { Val::V2([r.f64()?, r.f64()?]) } else { Val::AF((0..2 * n).map(|_| r.f64()).collect::<R<_>>()?) }
        }
        b'b' => {
            if scalar { Val::V6([r.f64()?, r.f64()?, r.f64()?, r.f64()?, r.f64()?, r.f64()?]) } else { Val::AF((0..6 * n).map(|_| r.f64()).collect::<R<_>>()?) }
        }
        b'l' => {
            if scalar { Val::B(r.u8()? != 0) } else { Val::AB(r.raw(n)?.iter().map(|&b| b != 0).collect()) }
        }
        b'c' => {
            if scalar { Val::C(r.u8()?) } else { Val::AS(r.raw(n)?.to_vec()) }
        }
        o => return Err(xerr(format!("unknown field kind {:?} for field {}", o as char, fld.name))),
    })
}

/// Parse one neutral-binary XT transmit (part or partition) stream.
pub fn read_xt(buf: &[u8], limits: &Limits) -> R<XtFile> {
    let mut r = Rd { b: buf, pos: 0 };
    if r.raw(2)? != b"PS" {
        return Err(SldprtError::new("xt.header", "not a binary XT stream (no PS flag)"));
    }
    if r.raw(2)? != [0, 0] {
        return Err(SldprtError::new("xt.header", "unsupported XT binary flavour (only neutral binary is supported)"));
    }
    let mut xt = XtFile::default();
    let mlen = r.i16()?;
    if mlen < 0 {
        return Err(SldprtError::new("xt.header", "negative modeller string length"));
    }
    xt.modeller = r.raw(mlen as usize)?.iter().map(|&c| c as char).collect();
    xt.partition = xt.modeller.contains("partition");
    let slen = r.i32()?;
    if !(0..4096).contains(&slen) {
        return Err(SldprtError::new("xt.header", "bad schema name length"));
    }
    xt.schema_id = r.raw(slen as usize)?.iter().map(|&c| c as char).collect();
    let parts: Vec<&str> = xt.schema_id.split('_').collect();
    if parts.len() >= 4 && parts[0] == "SCH" {
        let base: i64 = parts[3].parse().unwrap_or(0);
        r.u16()?; // maximum number of node types
        if base != 13006 {
            return Err(SldprtError::new("xt.schema", format!("unsupported base schema {base} (only SCH_13006 embedding is supported)")));
        }
    } else {
        return Err(SldprtError::new("xt.schema", format!("XT stream without embedded schema ({}) is not supported", xt.schema_id)));
    }
    if r.i32()? != 0 {
        return Err(SldprtError::new("xt.schema", "XT user fields are not supported"));
    }
    let mut layouts: HashMap<u16, Rc<Layout>> = HashMap::new();
    let mut first = true;
    loop {
        let ty = r.u16()?;
        if ty == 1 {
            if r.u16()? != 1 {
                return Err(xerr(format!("bad terminator at offset {}", r.pos - 4)));
            }
            break;
        }
        let layout = match layouts.get(&ty) {
            Some(l) => l.clone(),
            None => {
                let l = Rc::new(read_schema_info(&mut r, ty, &mut xt.schema_log)?);
                layouts.insert(ty, l.clone());
                l
            }
        };
        let mut count = 0usize;
        if is_variable(&layout) {
            let c = r.i32()?;
            if c < 0 {
                return Err(xerr(format!("negative variable length at offset {}", r.pos - 4)));
            }
            count = c as usize;
        }
        let idx = r.index()?;
        let mut vals = Vec::with_capacity(layout.len());
        for fld in layout.iter() {
            let v = read_value(&mut r, fld, if fld.n == VAR { count } else { fld.n as usize }).map_err(|e| xerr(format!("while reading {} #{idx}: {}", node_name(ty).unwrap_or("?"), e.message)))?;
            vals.push(v);
        }
        if xt.nodes.len() >= limits.max_nodes {
            return Err(SldprtError::new("limit.nodes", "too many XT nodes"));
        }
        if xt.nodes.insert(idx, Node { index: idx, ty, layout, vals }).is_some() {
            return Err(xerr(format!("duplicate node index {idx}")));
        }
        if first {
            xt.root_index = idx;
            first = false;
        }
    }
    xt.trailing_bytes = buf.len() - r.pos;
    Ok(xt)
}
