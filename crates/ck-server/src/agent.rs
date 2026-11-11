//! Agent-interface extensions (not in the original kernel): `check`,
//! `resolve`, `describe` (body queries) and `refDiff` (stateless, two parts).
//! Wire shapes: docs/agent-queries.md.

use crate::session::{check_part, Cx};
use ck_json::Json;

fn null_or_query(cx: &mut Cx, ty: &str) -> Result<Json, String> {
    if !cx.state()?.has_body {
        return Ok(Json::Null);
    }
    cx.query(ty)
}

/// `{type:"check", tag, expect:[{kind, equals|min|max, tol?}]}`
pub fn check(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "check")
}

/// `{type:"recognize", tag, opts?:{apply?}}`: feature recognition of an imported body (ck-recog).
pub fn recognize(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "recognize")
}

/// `{type:"fitQuadrics", tag, opts?}`: refit facet fans of the body as cylinders and cones (ck-ops fit_quadrics).
pub fn fit_quadrics(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "fitQuadrics")
}

/// `{type:"resolve", tag, descriptions:[{kind:"face", ...}]}`
pub fn resolve(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "resolve")
}

/// `{type:"describe", tag, names:["F2.C1", "F2.C1|F2.S.e1#3", ...]}`
pub fn describe(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "describe")
}

/// `{type:"section", tag?, plane:{origin, normal}}`: exact cross-section loops of the body.
pub fn section(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "section")
}

/// `{type:"sections", tag?, axis, n | positions}`: a stack of sections and the volume integrated from it.
pub fn sections(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "sections")
}

/// `{type:"probe", tag?, points?, rays?}`: inside / signed distance of points, hit lists of rays.
pub fn probe(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "probe")
}

/// `{type:"distance", tag?, ...}`: minimum distance of two entities (face | edge | vertex | point | lump | body).
pub fn distance(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "distance")
}

/// `{type:"maxDistance", tag?, ...}`: one-sided Hausdorff distance a -> b with an error bound.
pub fn max_distance(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "maxDistance")
}

/// `{type:"thickness", tag?, ...}`: minimum wall thickness and the thinnest regions.
pub fn thickness(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "thickness")
}

/// `{type:"thicknessAt", tag?, ...}`: wall thickness at a point or over a face.
pub fn thickness_at(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "thicknessAt")
}

/// `{type:"closestPoint", tag?, ...}`: closest entity of the body to many points.
pub fn closest_point(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "closestPoint")
}

/// `{type:"angles", tag?, ...}`: dihedral angles of edges / faces and draft angles against a pull direction.
pub fn angles(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "angles")
}

/// `{type:"clearance", tag?, ...}`: face pairs closer than `min`.
pub fn clearance(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "clearance")
}

/// `{type:"symmetry", tag?, ...}`: mirror-symmetry planes and the deviation of the mirrored body.
pub fn symmetry(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "symmetry")
}

/// `{type:"extrema", tag?, dir, of?}`: lowest and highest point of the body (or `of`) along a direction.
pub fn extrema(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "extrema")
}

/// `{type:"refDiff", base:<part>, part:<part>}`: regenerates both privately
/// and reports references that now point at a different entity.
pub fn ref_diff(cx: &mut Cx) -> Result<Json, String> {
    check_part(cx.msg.get("base"))?;
    check_part(cx.msg.get("part"))?;
    cx.query("refDiff")
}

/// `{type:"textMetrics", string, font?, height, spacing?}`: advance width and ink box of a string (stateless).
pub fn text_metrics(cx: &mut Cx) -> Result<Json, String> {
    cx.query("textMetrics")
}

/// `{type:"textOutline", string, font?, height, at?, anchor?, angle?, spacing?, mirror?, arc?}`: the loops of a
/// text entity as polylines (stateless, for the viewport preview).
pub fn text_outline(cx: &mut Cx) -> Result<Json, String> {
    cx.query("textOutline")
}

/// `{type:"equationCurve", x, y, t:[start, end], tol?, variables?}`: the points of the spline an equation curve
/// fits, with `closed`, `count`, `deviation` and `warnings` (stateless; docs/equation-curve.md).
pub fn equation_curve(cx: &mut Cx) -> Result<Json, String> {
    cx.query("equationCurve")
}

/// `{type:"loadFont", name, data:<base64 TTF/OTF>, replace?}`: register a font for the session; reply `{name, hash,
/// glyphs, supported, coverage, warnings, ..}`. Stateless w.r.t. tags (the registry is the kernel's).
pub fn load_font(cx: &mut Cx) -> Result<Json, String> {
    cx.query("loadFont")
}

/// `{type:"unloadFont", name}`: `{name, unloaded}`.
pub fn unload_font(cx: &mut Cx) -> Result<Json, String> {
    cx.query("unloadFont")
}

/// `{type:"listFonts"}`: `{fonts:[..]}`, the built-in fonts then the loaded ones.
pub fn list_fonts(cx: &mut Cx) -> Result<Json, String> {
    cx.query("listFonts")
}

/// `{type:"subsetFont", name, text}`: `{name, hash, data:<base64 TTF>, bytes, glyphs, warnings}`.
pub fn subset_font(cx: &mut Cx) -> Result<Json, String> {
    cx.query("subsetFont")
}

/// `{type:"threadData", standard?, size?}`: the thread tables (stateless).
/// No `standard`: the list of standards with their size designations; with
/// it: that standard's sizes with all numbers (mm); with `size` too: the one
/// entry. Unknown standard / size are wire errors that list the choices.
pub fn thread_data(cx: &mut Cx) -> Result<Json, String> {
    cx.query("threadData")
}

/// `{type:"sheetDxf", flatPattern, version?, units?, bendLines?, tangentLines?, bendNotes?, noteHeight?, layers?, layerNames?}`:
/// the flat pattern of a regen result's `sheetMetal` entry as DXF text (stateless; docs/sheet-metal.md "Export").
pub fn sheet_dxf(cx: &mut Cx) -> Result<Json, String> {
    cx.query("sheetDxf")
}

/// `{type:"sheetBendTable", flatPattern, format?:"csv"|"json", body?}`: the bend table as text (stateless).
pub fn sheet_bend_table(cx: &mut Cx) -> Result<Json, String> {
    cx.query("sheetBendTable")
}

/// `{type:"checkProfile", tag? | part?, profile:"<id>"|{rules:[..]}, opts?}`:
/// run a design-rule profile (fdm-print, cnc-3axis, injection or inline) on
/// the tag's body, or on `part` regenerated privately. Read-only. No body: null.
pub fn check_profile(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get("part").is_some() {
        check_part(cx.msg.get("part"))?;
        return cx.query("checkProfile");
    }
    null_or_query(cx, "checkProfile")
}

/// `{type:"profiles"}`: built-in profiles and the rule catalogue (stateless).
pub fn profiles(cx: &mut Cx) -> Result<Json, String> {
    cx.query("profiles")
}

/// `{type:"sketchAnalyze", featureId, tolerance?, tiny?, tag? | part?}`:
/// structural findings of one sketch (open ends with gap sizes,
/// near-coincident points, duplicates, overlaps, zero-length and tiny
/// segments, branches, isolated points, self-intersections), its chains and
/// the reason there is no closed profile. Read-only. With `part` the part is
/// regenerated privately (the `checkProfile` pattern — also the way to
/// analyse a document whose regen produced no body); with neither a body nor
/// a `part`: null.
pub fn sketch_analyze(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get("part").is_some() {
        check_part(cx.msg.get("part"))?;
        return cx.query("sketchAnalyze");
    }
    null_or_query(cx, "sketchAnalyze")
}

/// `{type:"sketchFreedoms", featureId, tag? | part?}`: degrees of freedom
/// of one sketch per point (0 / 1 with a direction / 2), per radius and
/// per entity — the same build and rank as the regen `status` block
/// (`dof` agrees with it). Read-only; `part` and null semantics exactly as
/// `sketchAnalyze`.
pub fn sketch_freedoms(cx: &mut Cx) -> Result<Json, String> {
    if cx.msg.get("part").is_some() {
        check_part(cx.msg.get("part"))?;
        return cx.query("sketchFreedoms");
    }
    null_or_query(cx, "sketchFreedoms")
}

/// `{type:"dfmMinWall", threshold, tag?, opts?:{samples}}`: the thinnest wall against a minimum, every location below it
/// with both faces and points. Read-only. No body: null.
pub fn dfm_min_wall(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "dfmMinWall")
}

/// `{type:"dfmMinInternalRadius", toolRadius, direction?, tag?}`: the smallest concave corner radius against the cutter
/// radius, with location. Read-only. No body: null.
pub fn dfm_min_internal_radius(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "dfmMinInternalRadius")
}

/// `{type:"dfmPocketRatio", maxRatio, tag?}`: depth / width of every closed pocket against a limit. Read-only. No body: null.
pub fn dfm_pocket_ratio(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "dfmPocketRatio")
}

/// `{type:"dfmOverhang", direction, angle, minArea?, tag?}`: faces steeper than `angle` from vertical against the build
/// direction, their area and a rough support volume. Read-only. No body: null.
pub fn dfm_overhang(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "dfmOverhang")
}

/// `{type:"dfmConstantSection", axis, tag?}`: whether the body is a constant extrusion along `axis`, else the first station
/// where the section changes. Read-only. No body: null.
pub fn dfm_constant_section(cx: &mut Cx) -> Result<Json, String> {
    null_or_query(cx, "dfmConstantSection")
}
