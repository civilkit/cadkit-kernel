use super::*;

fn lay(s: &str, f: &str, h: f64) -> Layout {
    layout(&Spec { string: s.into(), font: f.into(), height: h, ..Spec::default() }).unwrap()
}

fn holes(l: &Layout) -> usize {
    l.regions.iter().map(|r| r.holes.len()).sum()
}

#[test]
fn fonts_load() {
    for f in FONT_NAMES {
        let font = font(f).unwrap();
        assert!(font.cap_height() > 500.0, "{f}");
    }
    assert!(font("nope").is_none());
}

#[test]
fn letter_i_is_a_rectangle() {
    for f in FONT_NAMES {
        let l = lay("I", f, 10.0);
        if f == "gauge" {
            assert_eq!(l.regions.len(), 1);
            assert!(l.regions[0].outer.iter().all(|e| matches!(e, Edge::Line(_))), "{:?}", l.regions[0].outer);
            let b = l.bbox;
            let rect = (b[2] - b[0]) * (b[3] - b[1]);
            assert!((l.area() - rect).abs() / rect < 0.02, "area {} vs {}", l.area(), rect);
        }
        // cap height: the glyph is as tall as asked
        assert!((l.bbox[3] - l.bbox[1] - 10.0).abs() < 0.05, "{f}: {:?}", l.bbox);
    }
}

#[test]
fn nesting_of_glyphs() {
    for f in FONT_NAMES {
        // (string, regions, holes)
        for (s, r, h) in [("O", 1, 1), ("B", 1, 2), ("8", 1, 2), ("0", 1, 1), ("A", 1, 1), ("D", 1, 1), ("P", 1, 1), ("R", 1, 1), ("6", 1, 1), ("9", 1, 1), ("L", 1, 0), ("i", 2, 0), ("%", 3, 2), ("\u{b0}", 1, 1), ("RPM", 3, 2)] {
            let l = lay(s, f, 10.0);
            assert_eq!((l.regions.len(), holes(&l)), (r, h), "{f} {s:?}");
            for reg in &l.regions {
                for lp in reg.loops() {
                    for w in lp.windows(2) {
                        let (a, b) = (w[0].end(), w[1].start());
                        assert!((a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9, "open loop {s:?}");
                    }
                    let (a, b) = (lp.last().unwrap().end(), lp[0].start());
                    assert!((a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9, "open loop {s:?}");
                }
                assert!(reg.area() > 0.0);
            }
        }
    }
}

#[test]
fn orientation_outer_ccw_holes_cw() {
    let l = lay("B8", "gauge", 10.0);
    for r in &l.regions {
        assert!(signed_area(&loop_polygon(&r.outer, 16)) > 0.0);
        for h in &r.holes {
            assert!(signed_area(&loop_polygon(h, 16)) < 0.0);
        }
    }
}

#[test]
fn ring_area_matches_super_ellipse_estimate() {
    // O: outer minus inner. The bbox ellipse bounds it: area between 0.8 and 1.0 of the ellipse ring
    let l = lay("O", "gauge", 10.0);
    let o = &l.regions[0];
    let bb = |lp: &Vec<Edge>| {
        let p = loop_polygon(lp, 16);
        let (mut x0, mut x1, mut y0, mut y1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for q in p {
            x0 = x0.min(q[0]);
            x1 = x1.max(q[0]);
            y0 = y0.min(q[1]);
            y1 = y1.max(q[1]);
        }
        (x1 - x0, y1 - y0)
    };
    let (w, h) = bb(&o.outer);
    let (iw, ih) = bb(&o.holes[0]);
    let ring = std::f64::consts::FRAC_PI_4 * (w * h - iw * ih);
    let rect = w * h - iw * ih;
    assert!(o.area() > ring * 0.95 && o.area() < rect, "{} vs ring {} rect {}", o.area(), ring, rect);
}

#[test]
fn polyline_mode_has_the_same_area() {
    for f in FONT_NAMES {
        for s in ["0123456789", "RPM x100", "KNOTS", "OIL PSI", "TEMP \u{b0}F", "%&@Qgs"] {
            let a = lay(s, f, 10.0);
            let b = layout(&Spec { string: s.into(), font: f.into(), height: 10.0, polyline: true, ..Spec::default() }).unwrap();
            assert!((a.area() - b.area()).abs() / a.area() < 0.01, "{f} {s}: {} vs {}", a.area(), b.area());
            assert!(b.regions.iter().all(|r| r.loops().all(|l| l.iter().all(|e| matches!(e, Edge::Line(_))))));
        }
    }
}

#[test]
fn area_scales_with_height() {
    let a = lay("KNOTS", "engrave", 5.0).area();
    let b = lay("KNOTS", "engrave", 10.0).area();
    assert!((b / a - 4.0).abs() < 1e-9);
}

fn cross_free(poly: &[P]) -> bool {
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        for j in i + 2..n {
            if i == 0 && j == n - 1 {
                continue;
            }
            let (c, d) = (poly[j], poly[(j + 1) % n]);
            let o = |p: P, q: P, r: P| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
            let (d1, d2, d3, d4) = (o(a, b, c), o(a, b, d), o(c, d, a), o(c, d, b));
            if d1 * d2 < 0.0 && d3 * d4 < 0.0 {
                return false;
            }
        }
    }
    true
}

/// No glyph of either font has crossing contours (the loops are then disjoint or nested, which the
/// nesting rule needs).
#[test]
fn glyph_contours_do_not_cross() {
    for f in FONT_NAMES {
        let font = font(f).unwrap();
        for cp in 0x21u32..0x100 {
            let Some(ch) = char::from_u32(cp) else { continue };
            let Some(g) = font.glyph_index(ch) else { continue };
            let ls = glyph_loops(font, g, None, RUN_SPANS);
            let polys: Vec<Vec<P>> = ls.iter().map(|l| loop_polygon(l, 10)).collect();
            for (i, p) in polys.iter().enumerate() {
                assert!(cross_free(p), "{f} {ch:?} self crossing");
                for q in &polys[i + 1..] {
                    let both: Vec<P> = p.iter().chain(q.iter()).copied().collect();
                    let _ = both;
                    // loops must not cross each other
                    for a in 0..p.len() {
                        for b in 0..q.len() {
                            let (a0, a1, b0, b1) = (p[a], p[(a + 1) % p.len()], q[b], q[(b + 1) % q.len()]);
                            let o = |p: P, q: P, r: P| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
                            let (d1, d2, d3, d4) = (o(a0, a1, b0), o(a0, a1, b1), o(b0, b1, a0), o(b0, b1, a1));
                            assert!(!(d1 * d2 < 0.0 && d3 * d4 < 0.0), "{f} {ch:?} loops cross");
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn errors_and_warnings() {
    assert_eq!(layout(&Spec { string: "".into(), ..Spec::default() }).unwrap_err(), Error::Empty);
    assert_eq!(layout(&Spec { string: "   ".into(), ..Spec::default() }).unwrap_err(), Error::NothingToDraw);
    assert!(matches!(layout(&Spec { string: "A".into(), font: "zz".into(), ..Spec::default() }), Err(Error::UnknownFont(_))));
    assert_eq!(layout(&Spec { string: "A".into(), height: 0.0, ..Spec::default() }).unwrap_err(), Error::BadHeight);
    let l = lay("A\u{65e5}B", "gauge", 5.0);
    assert!(l.warnings.iter().any(|w| w.code == "textMissingGlyph" && w.detail.contains('\u{65e5}')));
    assert_eq!(l.regions.len(), 2);
    let l = lay("AB", "gauge", 0.5);
    assert!(l.warnings.iter().any(|w| w.code == "textTooSmall"));
    let long: String = "A".repeat(600);
    let l = lay(&long, "gauge", 1.0);
    assert!(l.warnings.iter().any(|w| w.code == "textTruncated"));
    assert_eq!(l.regions.len(), 500);
}

#[test]
fn anchors_and_angle() {
    let h = 10.0;
    let base = |hh, vv, ang: f64| {
        layout(&Spec { string: "HIHI".into(), height: h, at: [100.0, 50.0], h: hh, v: vv, angle: ang, ..Spec::default() }).unwrap().bbox
    };
    let b = base(HAnchor::Left, VAnchor::Baseline, 0.0);
    assert!((b[0] - 100.0).abs() < 1e-9 && b[1].abs() > 0.0 && (b[1] - 50.0).abs() < 1e-6);
    let c = base(HAnchor::Center, VAnchor::Middle, 0.0);
    assert!(((c[0] + c[2]) / 2.0 - 100.0).abs() < 1e-6, "{c:?}");
    assert!(((c[1] + c[3]) / 2.0 - 50.0).abs() < 1e-6);
    let r = base(HAnchor::Right, VAnchor::Top, 0.0);
    assert!((r[2] - 100.0).abs() < 1e-6 && (r[3] - 50.0).abs() < 1e-6);
    // 90 degrees: text runs up the page
    let u = base(HAnchor::Left, VAnchor::Baseline, 90.0);
    assert!(u[3] - u[1] > u[2] - u[0]);
}

#[test]
fn text_reads_left_to_right_unmirrored() {
    // 'L' has its stem on the left: more ink on the left half than on the right at mid-height
    let l = lay("L", "gauge", 10.0);
    let r = &l.regions[0];
    let poly = loop_polygon(&r.outer, 8);
    let mid = (l.bbox[0] + l.bbox[2]) / 2.0;
    let y = l.bbox[1] + 8.0;
    assert!(inside(&poly, [l.bbox[0] + 0.5, y]) && !inside(&poly, [mid + 0.5, y]));
    // mirrored: stem on the right
    let m = layout(&Spec { string: "L".into(), height: 10.0, mirror: true, ..Spec::default() }).unwrap();
    let poly = loop_polygon(&m.regions[0].outer, 8);
    let mid = (m.bbox[0] + m.bbox[2]) / 2.0;
    assert!(inside(&poly, [m.bbox[2] - 0.5, y]) && !inside(&poly, [mid - 0.5, y]));
    assert!(signed_area(&poly) > 0.0, "mirrored loops are re-oriented");
}

#[test]
fn arc_text_sits_on_the_circle() {
    let spec = |side, ac| Spec {
        string: "KNOTS".into(),
        height: 4.0,
        h: HAnchor::Center,
        arc: Some(ArcSpec { centre: [10.0, 20.0], radius: 30.0, angle_centre: ac, side }),
        ..Spec::default()
    };
    // outside, centred at 90 degrees: above the centre, glyph tops outwards, ink between R and R + 4
    let l = layout(&spec(Side::Outside, 90.0)).unwrap();
    let mut rmin = f64::MAX;
    let mut rmax = f64::MIN;
    for r in &l.regions {
        for p in loop_polygon(&r.outer, 6) {
            let d = (p[0] - 10.0).hypot(p[1] - 20.0);
            rmin = rmin.min(d);
            rmax = rmax.max(d);
        }
    }
    assert!(rmin > 29.9 && rmax < 34.3, "{rmin} {rmax}");
    // reads left to right at the top: the first glyph is left of the last
    let cx = |r: &Region| loop_polygon(&r.outer, 4).iter().map(|p| p[0]).sum::<f64>() / loop_polygon(&r.outer, 4).len() as f64;
    assert!(cx(&l.regions[0]) < cx(&l.regions[l.regions.len() - 1]));
    assert!(l.bbox[1] > 20.0 + 29.0);
    // inside, centred at 270: below the centre, tops pointing at the centre, reads left to right
    let l = layout(&spec(Side::Inside, 270.0)).unwrap();
    let mut rmin = f64::MAX;
    let mut rmax = f64::MIN;
    for r in &l.regions {
        for p in loop_polygon(&r.outer, 6) {
            let d = (p[0] - 10.0).hypot(p[1] - 20.0);
            rmin = rmin.min(d);
            rmax = rmax.max(d);
        }
    }
    assert!(rmin > 25.6 && rmax < 30.1, "{rmin} {rmax}");
    assert!(cx(&l.regions[0]) < cx(&l.regions[l.regions.len() - 1]));
    assert!(l.bbox[3] < 20.0 - 25.0);
}

#[test]
fn metrics_match_layout() {
    let m = metrics("RPM x100", "gauge", 8.0, 1.0).unwrap();
    let l = layout(&Spec { string: "RPM x100".into(), height: 8.0, ..Spec::default() }).unwrap();
    assert!((l.bbox[2] - l.bbox[0] - (m.bbox[2] - m.bbox[0])).abs() < 1e-6);
    assert!(m.advance > m.bbox[2] - m.bbox[0]);
    assert_eq!(m.chars, 8);
    assert_eq!(m.advances.len(), 8);
    let wide = metrics("RPM x100", "gauge", 8.0, 1.5).unwrap();
    assert!((wide.advance / m.advance - 1.5).abs() < 1e-9);
}

#[test]
fn twenty_characters_are_quick_and_small() {
    let l = lay("OIL PSI TEMP 0123456", "gauge", 6.0);
    let edges: usize = l.regions.iter().flat_map(|r| r.loops()).map(|lp| lp.len()).sum();
    assert!(edges < 400, "{edges} edges");
}

#[test]
fn tight_arc_widens_and_tight_spacing_warns() {
    let spec = Spec {
        string: "OIL PSI".into(),
        height: 5.0,
        h: HAnchor::Center,
        arc: Some(ArcSpec { centre: [0.0, 0.0], radius: 24.0, angle_centre: 270.0, side: Side::Inside }),
        ..Spec::default()
    };
    let l = layout(&spec).unwrap();
    assert!(collision(&l.regions, 0.0).is_none(), "{:?}", l.warnings);
    let l = layout(&Spec { string: "HIM".into(), height: 10.0, spacing: 0.1, ..Spec::default() }).unwrap();
    // letters that would overlap are pushed apart
    assert!(l.warnings.iter().any(|w| w.code == "textSpacingAdjusted"), "{:?}", l.warnings);
    assert!(collision(&l.regions, 0.03).is_none());
    let l = layout(&Spec { string: "HIM".into(), height: 10.0, ..Spec::default() }).unwrap();
    assert!(l.warnings.is_empty());
    // the serif font's touching serifs
    let l = layout(&Spec { string: "vvVV".into(), font: "engrave".into(), height: 3.0, ..Spec::default() }).unwrap();
    assert!(collision(&l.regions, 0.03).is_none());
}
