//! Lettering entities: `{"type":"text","id":"t1","string":"RPM x100","font":"gauge","height":4,"at":[0,0],
//! "anchor":["center","middle"],"angle":0,"spacing":1,"mirror":false,"arc":{..},"polyline":false}`.
//!
//! A text entity is fixed geometry: it takes no constraints and is not intersected with the other
//! curves of the sketch. Its outlines (ck-text) become profile regions of their own, appended after
//! the regions of the curve arrangement, so letters inside a rectangle of the same sketch do not
//! become holes of it: keep lettering in a sketch of its own (docs/text.md). `p` names a sketch
//! point used as the anchor (so the position can be constrained or dragged); without it `at` is used.

use super::profile::{Geom, Region, Seg, SegGeom};
use super::{jnum, jnum_opt, jstr, jtruthy, Sketch};
use ck_json::Json;

/// A parsed `text` entity.
#[derive(Debug, Clone)]
pub struct TextEntity {
    pub spec: ck_text::Spec,
    /// Anchor point id (`p`), when given.
    pub p: Option<String>,
    /// Centre point id of an arc layout (`arc.c`), when given.
    pub arc_c: Option<String>,
}

fn pair(j: Option<&Json>) -> Option<[f64; 2]> {
    let a = j?.as_arr()?;
    Some([jnum_opt(a.first())?, jnum_opt(a.get(1))?])
}

fn words(j: Option<&Json>) -> Vec<String> {
    match j {
        Some(Json::Str(s)) => s.split(|c: char| c == ' ' || c == ',' || c == 'x').map(|w| w.to_ascii_lowercase()).filter(|w| !w.is_empty()).collect(),
        Some(Json::Arr(a)) => a.iter().filter_map(|v| v.as_str().map(|s| s.to_ascii_lowercase())).collect(),
        _ => vec![],
    }
}

impl TextEntity {
    pub fn from_json(j: &Json) -> TextEntity {
        let mut spec = ck_text::Spec {
            string: jstr(j, "string"),
            font: j.get("font").and_then(|v| v.as_str()).unwrap_or("gauge").to_string(),
            font_hash: j.get("fontHash").and_then(|v| v.as_str()).filter(|h| !h.is_empty()).map(str::to_string),
            height: jnum_opt(j.get("height")).unwrap_or(5.0),
            spacing: jnum_opt(j.get("spacing")).unwrap_or(1.0),
            at: pair(j.get("at")).unwrap_or([0.0, 0.0]),
            angle: jnum(j.get("angle")),
            mirror: jtruthy(j.get("mirror")),
            polyline: jtruthy(j.get("polyline")),
            run_spans: jnum_opt(j.get("runSpans")).map_or(ck_text::Spec::default().run_spans, |v| v.max(1.0).min(16.0) as usize),
            kern: jtruthy(j.get("kern")),
            liga: jtruthy(j.get("liga")),
            merge: jtruthy(j.get("merge")),
            ..ck_text::Spec::default()
        };
        for w in words(j.get("anchor")).into_iter().chain(words(j.get("halign"))).chain(words(j.get("valign"))) {
            match w.as_str() {
                "left" => spec.h = ck_text::HAnchor::Left,
                "center" | "centre" => spec.h = ck_text::HAnchor::Center,
                "right" => spec.h = ck_text::HAnchor::Right,
                "baseline" | "bottom" => spec.v = ck_text::VAnchor::Baseline,
                "middle" => spec.v = ck_text::VAnchor::Middle,
                "top" => spec.v = ck_text::VAnchor::Top,
                _ => {}
            }
        }
        let mut arc_c = None;
        if let Some(a @ Json::Obj(_)) = j.get("arc") {
            let side = match a.get("side").and_then(|v| v.as_str()) {
                Some("inside") => ck_text::Side::Inside,
                _ => ck_text::Side::Outside,
            };
            if let Some(Json::Str(s)) = a.get("c").or(a.get("centre")) {
                arc_c = Some(s.clone());
            }
            spec.arc = Some(ck_text::ArcSpec {
                centre: pair(a.get("centre").or(a.get("c"))).unwrap_or([0.0, 0.0]),
                radius: jnum(a.get("radius")),
                angle_centre: jnum_opt(a.get("angleCentre")).unwrap_or(90.0),
                side,
            });
        }
        TextEntity { spec, p: j.get("p").and_then(|v| v.as_str()).map(str::to_string), arc_c }
    }

    /// The spec with the anchor / centre points resolved against the sketch.
    pub fn resolved(&self, sk: &Sketch) -> ck_text::Spec {
        let mut s = self.spec.clone();
        if let Some(q) = self.p.as_deref().and_then(|p| sk.point(p)) {
            s.at = [q.x, q.y];
        }
        if let (Some(a), Some(q)) = (s.arc.as_mut(), self.arc_c.as_deref().and_then(|p| sk.point_xy(p))) {
            a.centre = q;
        }
        s
    }
}

/// One text entity's layout (or why it has none).
pub type TextLayout = (String, Result<ck_text::Layout, ck_text::Error>);

/// Lay out every non-construction text entity of the sketch.
pub fn layout_all(sk: &Sketch) -> Vec<TextLayout> {
    sk.entities
        .iter()
        .filter_map(|e| match e {
            super::SketchEntity::Text { id, text, construction } if !construction => Some((id.clone(), ck_text::layout(&text.resolved(sk)))),
            _ => None,
        })
        .collect()
}

fn seg(tag: &str, e: &ck_text::Edge) -> Seg {
    match e {
        ck_text::Edge::Line(l) => Seg {
            kind: "line",
            tag: tag.to_string(),
            a: l[0],
            b: l[1],
            curve: SegGeom { geom: Geom::Line { p: l[0], d: [l[1][0] - l[0][0], l[1][1] - l[0][1]] }, ta: 0.0, tb: 1.0 },
        },
        ck_text::Edge::Quad(q) => {
            let m = (q.len() - 2) as f64;
            Seg {
                kind: "spline",
                tag: tag.to_string(),
                a: q[0],
                b: q[q.len() - 1],
                curve: SegGeom { geom: Geom::BSpline { cv: q.clone(), degree: 2, knots: e.knots() }, ta: 0.0, tb: m },
            }
        }
    }
}

/// Profile regions of the laid-out text.
pub fn regions(layouts: &[TextLayout]) -> Vec<Region> {
    let mut out = vec![];
    for (id, l) in layouts {
        let Ok(l) = l else { continue };
        for r in &l.regions {
            let lp = |l: &Vec<ck_text::Edge>| l.iter().map(|e| seg(id, e)).collect::<Vec<_>>();
            out.push(Region { outer: lp(&r.outer), holes: r.holes.iter().map(lp).collect() });
        }
    }
    out
}

/// `(code, params)` of the first layout error, if any.
pub fn first_error(layouts: &[TextLayout]) -> Option<(String, &ck_text::Error)> {
    layouts.iter().find_map(|(id, l)| l.as_ref().err().map(|e| (id.clone(), e)))
}

/// The warnings of all layouts as `(entity id, warning)`.
pub fn warnings(layouts: &[TextLayout]) -> Vec<(&str, &ck_text::Warning)> {
    layouts.iter().filter_map(|(id, l)| l.as_ref().ok().map(|l| (id.as_str(), l))).flat_map(|(id, l)| l.warnings.iter().map(move |w| (id, w))).collect()
}

fn jpt(p: [f64; 2]) -> Json {
    Json::Arr(vec![Json::Num(p[0]), Json::Num(p[1])])
}

fn warn_json(w: &ck_text::Warning) -> Json {
    Json::Obj(vec![("code".into(), Json::Str(w.code.into())), ("detail".into(), Json::Str(w.detail.clone()))])
}

fn err_text(e: &ck_text::Error) -> String {
    format!("{}: {}", e.code(), e.message())
}

/// `{type:"textMetrics", string, font?, height, spacing?, kern?, liga?}`: advance width and ink box
/// (relative to the pen start on the baseline) so a script can centre and size text. `kern` and
/// `liga` mirror the text entity's opt-in flags. Stateless.
pub fn metrics_reply(msg: &Json) -> Result<Json, String> {
    let string = msg.get("string").and_then(|v| v.as_str()).unwrap_or("");
    let font = msg.get("font").and_then(|v| v.as_str()).unwrap_or("gauge");
    let height = jnum_opt(msg.get("height")).unwrap_or(5.0);
    let spacing = jnum_opt(msg.get("spacing")).unwrap_or(1.0);
    let pin = msg.get("fontHash").and_then(|v| v.as_str());
    let kern = jtruthy(msg.get("kern"));
    let liga = jtruthy(msg.get("liga"));
    let m = ck_text::metrics(string, font, pin, height, spacing, kern, liga).map_err(|e| err_text(&e))?;
    Ok(Json::Obj(vec![
        ("advance".into(), Json::Num(m.advance)),
        ("bbox".into(), Json::Arr(m.bbox.iter().map(|v| Json::Num(*v)).collect())),
        ("width".into(), Json::Num(m.bbox[2] - m.bbox[0])),
        ("capHeight".into(), Json::Num(m.cap_height)),
        ("chars".into(), Json::Num(m.chars as f64)),
        ("advances".into(), Json::Arr(m.advances.iter().map(|v| Json::Num(*v)).collect())),
        ("warnings".into(), Json::Arr(m.warnings.iter().map(warn_json).collect())),
    ]))
}

/// `{type:"textOutline", <text entity fields>}`: the loops a text entity gives, as polylines, for a
/// viewport preview. Stateless.
pub fn outline_reply(msg: &Json) -> Result<Json, String> {
    let t = TextEntity::from_json(msg);
    let l = ck_text::layout(&t.spec).map_err(|e| err_text(&e))?;
    let poly = |lp: &Vec<ck_text::Edge>| Json::Arr(ck_text::loop_polygon(lp, 8).into_iter().map(jpt).collect());
    let regions: Vec<Json> = l
        .regions
        .iter()
        .map(|r| Json::Obj(vec![("outer".into(), poly(&r.outer)), ("holes".into(), Json::Arr(r.holes.iter().map(poly).collect()))]))
        .collect();
    // `edges: true`: the exact edges of every loop (`[[x,y],[x,y]]` for a line, the control points of a
    // quadratic B-spline otherwise), for debugging and for drawing the exact outline
    let edges_json = |lp: &Vec<ck_text::Edge>| {
        Json::Arr(
            lp.iter()
                .map(|e| match e {
                    ck_text::Edge::Line(l) => Json::Obj(vec![("line".into(), Json::Arr(l.iter().map(|p| jpt(*p)).collect()))]),
                    ck_text::Edge::Quad(q) => Json::Obj(vec![("bspline2".into(), Json::Arr(q.iter().map(|p| jpt(*p)).collect()))]),
                })
                .collect(),
        )
    };
    let with_edges = jtruthy(msg.get("edges"));
    let regions = if with_edges {
        l.regions
            .iter()
            .zip(regions)
            .map(|(r, mut j)| {
                if let Json::Obj(m) = &mut j {
                    m.push(("outerEdges".into(), edges_json(&r.outer)));
                    m.push(("holeEdges".into(), Json::Arr(r.holes.iter().map(edges_json).collect())));
                }
                j
            })
            .collect()
    } else {
        regions
    };
    Ok(Json::Obj(vec![
        ("regions".into(), Json::Arr(regions)),
        ("bbox".into(), Json::Arr(l.bbox.iter().map(|v| Json::Num(*v)).collect())),
        ("area".into(), Json::Num(l.area())),
        ("warnings".into(), Json::Arr(l.warnings.iter().map(warn_json).collect())),
    ]))
}

/// `inspect` entry of one text entity: `{id, string, font, height, ok, bbox, area, glyphs, error?}`.
pub fn info_json(sk: &Sketch, t: &TextLayout) -> Json {
    let (id, l) = t;
    let e = sk.entity(id);
    let spec = match e {
        Some(super::SketchEntity::Text { text, .. }) => Some(text.resolved(sk)),
        _ => None,
    };
    let mut o = vec![("id".to_string(), Json::Str(id.clone()))];
    if let Some(s) = &spec {
        o.push(("string".into(), Json::Str(s.string.clone())));
        o.push(("font".into(), Json::Str(s.font.clone())));
        if let Some(e) = ck_text::font_entry(&s.font) {
            o.push(("fontHash".into(), Json::Str(ck_text::sha256_hex(&e.hash))));
        }
        o.push(("height".into(), Json::Num(s.height)));
    }
    match l {
        Ok(l) => {
            o.push(("ok".into(), Json::Bool(true)));
            o.push(("bbox".into(), Json::Arr(l.bbox.iter().map(|v| Json::Num(*v)).collect())));
            o.push(("area".into(), Json::Num(l.area())));
            o.push(("glyphs".into(), Json::Num(l.regions.iter().map(|r| r.glyph + 1).max().unwrap_or(0) as f64)));
            if !l.warnings.is_empty() {
                o.push(("warnings".into(), Json::Arr(l.warnings.iter().map(warn_json).collect())));
            }
        }
        Err(e) => {
            o.push(("ok".into(), Json::Bool(false)));
            o.push(("error".into(), Json::Str(e.code().into())));
        }
    }
    Json::Obj(o)
}

// ---------------------------------------------------------------------------------------------
// fonts: loadFont / unloadFont / listFonts / subsetFont

fn font_err(e: &ck_text::FontError) -> String {
    format!("{}: {}", e.code, e.message)
}

fn font_warn(w: &ck_text::FontWarn) -> Json {
    Json::Obj(vec![("code".into(), Json::Str(w.code.into())), ("detail".into(), Json::Str(w.detail.clone()))])
}

fn font_info(i: &ck_text::FontInfo) -> Json {
    let mut o = vec![
        ("name".to_string(), Json::Str(i.name.clone())),
        ("hash".into(), Json::Str(i.hash.clone())),
        ("builtin".into(), Json::Bool(i.builtin)),
        ("family".into(), i.family.clone().map_or(Json::Null, Json::Str)),
        ("bytes".into(), Json::Num(i.bytes as f64)),
        ("glyphs".into(), Json::Num(i.glyphs as f64)),
        ("chars".into(), Json::Num(i.chars as f64)),
        ("unitsPerEm".into(), Json::Num(i.units_per_em)),
        ("capHeight".into(), Json::Num(i.cap_height)),
        ("outlines".into(), Json::Str(i.outlines.into())),
        ("supported".into(), Json::Arr(i.coverage.iter().map(|c| Json::Str(c.0.into())).collect())),
        (
            "coverage".into(),
            Json::Arr(
                i.coverage
                    .iter()
                    .map(|c| Json::Obj(vec![("block".into(), Json::Str(c.0.into())), ("mapped".into(), Json::Num(c.1 as f64)), ("of".into(), Json::Num(c.2 as f64))]))
                    .collect(),
            ),
        ),
        ("warnings".into(), Json::Arr(i.warnings.iter().map(font_warn).collect())),
    ];
    if !i.builtin {
        o.push(("alreadyLoaded".into(), Json::Bool(i.already_loaded)));
        o.push(("replaced".into(), Json::Bool(i.replaced)));
    }
    Json::Obj(o)
}

/// `{type:"loadFont", name, data:<base64 TTF/OTF>, replace?}`: register a font for the session.
pub fn load_font_reply(msg: &Json) -> Result<Json, String> {
    let name = msg.get("name").and_then(|v| v.as_str()).unwrap_or("");
    let data = msg.get("data").and_then(|v| v.as_str()).ok_or("fontBadData: data must be the base64 text of a TTF or OTF file")?;
    let bytes = ck_text::base64_decode(data).map_err(|m| format!("fontBadData: {m}"))?;
    let info = ck_text::load_font(name, bytes, jtruthy(msg.get("replace"))).map_err(|e| font_err(&e))?;
    Ok(font_info(&info))
}

/// `{type:"unloadFont", name}`: `{name, unloaded}`; built-in names are refused.
pub fn unload_font_reply(msg: &Json) -> Result<Json, String> {
    let name = msg.get("name").and_then(|v| v.as_str()).unwrap_or("");
    let gone = ck_text::unload_font(name).map_err(|e| font_err(&e))?;
    Ok(Json::Obj(vec![("name".into(), Json::Str(name.into())), ("unloaded".into(), Json::Bool(gone))]))
}

/// `{type:"listFonts"}`: `{fonts: [info..]}`, built-ins first.
pub fn list_fonts_reply() -> Json {
    Json::Obj(vec![("fonts".into(), Json::Arr(ck_text::list_fonts().iter().map(font_info).collect()))])
}

/// `{type:"subsetFont", name, text}`: `{name, hash, data:<base64 TTF>, bytes, glyphs, chars, warnings}`, a
/// plain TrueType font with just the glyphs of `text` (plus `.notdef` and `H`).
pub fn subset_font_reply(msg: &Json) -> Result<Json, String> {
    let name = msg.get("name").and_then(|v| v.as_str()).unwrap_or("");
    let text = msg.get("text").and_then(|v| v.as_str()).ok_or("subsetFont: text must be a string")?;
    let (bytes, warnings) = ck_text::subset_font(name, text).map_err(|e| font_err(&e))?;
    let hash = ck_text::sha256_hex(&ck_text::sha256(&[&bytes]));
    let f = ck_text::Font::parse(std::sync::Arc::from(bytes.clone())).map_err(|m| format!("fontSubsetFailed: {m}"))?;
    Ok(Json::Obj(vec![
        ("name".into(), Json::Str(name.into())),
        ("hash".into(), Json::Str(hash)),
        ("bytes".into(), Json::Num(bytes.len() as f64)),
        ("glyphs".into(), Json::Num(f.num_glyphs() as f64)),
        ("data".into(), Json::Str(ck_text::base64_encode(&bytes))),
        ("warnings".into(), Json::Arr(warnings.iter().map(font_warn).collect())),
    ]))
}
