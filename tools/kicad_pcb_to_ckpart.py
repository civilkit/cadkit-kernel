#!/usr/bin/env python3
"""Convert a KiCad .kicad_pcb (text, KiCad 6..10) into a ck-part document of simple bodies.

No KiCad install needed: the board file is parsed directly.

What it builds (one named body per item, `op: newBody`):
  BOARD      the board slab from Edge.Cuts (lines, arcs, rects, polys, circles; inner loops are cutouts)
             with every non-plated hole drilled (and plated ones with --pth-holes)
  <REF>      one box per footprint, sized from its courtyard (else pads, else fab layer), or a cylinder
             when the footprint's fab layer has a circle that covers it (radial caps, round parts)
  KEEPOUT_*  each rule-area keep-out zone as a 0.1 mm film on the board face it applies to
  extras     optional envelopes from --extras (modules on headers, etc., see below)

Heights: "height=X mm" in the footprint description wins, then the HEIGHTS table (regex on the
footprint name), then DEFAULT_HEIGHT. The source of every number is written to --report.

Frame (as KiCad's own STEP export): the board lies in XY with its bottom face at z = 0 and its top face at
z = thickness; KiCad x -> x, KiCad y -> -y. Top-side parts grow +Z from the top face, bottom-side parts
grow -Z from z = 0.

--extras JSON: {"bodies": [{"name": "MOD_X", "over": ["P3", "P4"] | "at": [x, y],
  "size": [w, l], "rot": deg, "z0": mm above the board face, "height": mm, "side": "top"|"bottom",
  "shape": "box"|"cyl", "margin": mm, "note": "..."}]}
  "over": the envelope is centred on the union of those footprints' outlines; "size" defaults to
  that union plus "margin".

usage: python3 kicad_pcb_to_ckpart.py board.kicad_pcb out.ckpart [--thickness 1.6] [--extras x.json]
       [--report report.json] [--name NAME] [--pth-holes] [--skip REGEX]
"""
import argparse
import json
import math
import re
import sys

DEFAULT_HEIGHT = 2.0
# (footprint-name regex, height mm, shape hint). First match wins. Heights are typical catalogue values.
HEIGHTS = [
    (r"^MountingHole", None, "skip"),
    (r"^Fiducial", None, "skip"),
    (r"^(NetTie|Logo|Symbol)", None, "skip"),
    (r"SolderJumper", 0.05, "box"),
    (r"^TestPoint", 1.0, None),
    (r"^(R|C|L|LED|D)_0402", 0.5, "box"),
    (r"^(R|C|L|LED|D)_0603", 0.8, "box"),
    (r"^(R|C|L|LED|D)_0805", 1.0, "box"),
    (r"^(R|C|L|LED|D)_1206", 1.1, "box"),
    (r"^(R|C|L|LED|D)_12\d\d|^(R|C|L)_2\d\d\d", 1.3, "box"),
    (r"^D_SOD-123", 1.1, "box"),
    (r"^D_SOD-323", 1.0, "box"),
    (r"^D_SMA", 2.3, "box"),
    (r"^D_SMB", 2.3, "box"),
    (r"^D_SMC", 2.5, "box"),
    (r"^D_DO-201", 5.3, "box"),
    (r"^D_DO-41", 2.7, "box"),
    (r"^SOT-23", 1.1, "box"),
    (r"^SOT-223", 1.8, "box"),
    (r"^(SOIC|SOP|TSSOP|MSOP|SSOP)", 1.75, "box"),
    (r"^(QFN|DFN|VQFN|WQFN)", 0.9, "box"),
    (r"^(LQFP|TQFP)", 1.6, "box"),
    (r"^TO-220.*Vertical", 19.0, "box"),
    (r"^TO-220.*Horizontal", 4.5, "box"),
    (r"^TO-252|^TO-263", 2.4, "box"),
    (r"^PinSocket_\dx\d+_P2\.54mm_Vertical", 8.5, "box"),
    (r"^PinSocket_\dx\d+_P2\.54mm_Horizontal", 2.54, "box"),
    (r"^PinHeader_\dx\d+_P2\.54mm_Vertical", 8.5, "box"),
    (r"^PinHeader_\dx\d+_P2\.54mm_Horizontal", 2.54, "box"),
    (r"^PinSocket_\dx\d+_P1\.27mm", 4.4, "box"),
    (r"^JST_XH_.*Vertical", 7.0, "box"),
    (r"^JST_XH_.*Horizontal", 6.0, "box"),
    (r"^JST_PH_", 6.0, "box"),
    (r"^JST_SH_", 4.3, "box"),
    (r"^TerminalBlock_Phoenix_MKDS-1,5-.*5\.08mm", 10.0, "box"),
    (r"^TerminalBlock", 10.0, "box"),
    (r"XT30", 8.4, "box"),
    (r"XT60", 8.6, "box"),
    (r"^Fuseholder_Clip-5x20mm", 8.5, "box"),
    (r"^Fuseholder", 10.0, "box"),
    (r"^CP_Radial|^C_Radial", 12.0, "cyl"),
    (r"^C_Disc", 5.0, "box"),
    (r"^USB_C", 3.3, "box"),
    (r"^SW_", 5.0, "box"),
    (r"^Crystal", 4.0, "box"),
    (r"^Buzzer", 9.5, "cyl"),
    (r"^Relay", 15.0, "box"),
]


# ---------------------------------------------------------------- s-expression reader
def parse_sexpr(text):
    tok = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+')
    stack, cur = [], []
    for m in tok.finditer(text):
        t = m.group(0)
        if t == "(":
            stack.append(cur)
            cur = []
        elif t == ")":
            done = cur
            cur = stack.pop()
            cur.append(done)
        elif t[0] == '"':
            cur.append(bytes(t[1:-1], "utf-8").decode("unicode_escape") if "\\" in t else t[1:-1])
        else:
            cur.append(t)
    return cur[0]


def kids(node, name):
    return [c for c in node[1:] if isinstance(c, list) and c and c[0] == name]


def kid(node, name):
    k = kids(node, name)
    return k[0] if k else None


def num(v):
    return float(v)


def xy(node):
    return (num(node[1]), num(node[2]))


def layer_of(node):
    l = kid(node, "layer")
    return l[1] if l else None


# ---------------------------------------------------------------- geometry helpers
def rot_pt(p, deg):
    # KiCad: (at x y a) rotates the footprint by a degrees CCW as seen on screen (y down)
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return (p[0] * c + p[1] * s, -p[0] * s + p[1] * c)


def circle3(a, b, c):
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return None
    ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
    return (ux, uy)


def bbox(pts):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))


def graphic_pts(g):
    """Sample points of a graphic item (local coords) for bounding boxes."""
    t = g[0]
    if t in ("fp_line", "gr_line"):
        return [xy(kid(g, "start")), xy(kid(g, "end"))]
    if t in ("fp_rect", "gr_rect"):
        (x0, y0), (x1, y1) = xy(kid(g, "start")), xy(kid(g, "end"))
        return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    if t in ("fp_circle", "gr_circle"):
        c, e = xy(kid(g, "center")), xy(kid(g, "end"))
        r = math.dist(c, e)
        return [(c[0] - r, c[1] - r), (c[0] + r, c[1] + r)]
    if t in ("fp_arc", "gr_arc"):
        s, m, e = xy(kid(g, "start")), xy(kid(g, "mid")), xy(kid(g, "end"))
        return [s, m, e]
    if t in ("fp_poly", "gr_poly"):
        return [xy(p) for p in kids(kid(g, "pts"), "xy")]
    return []


# ---------------------------------------------------------------- ck-part builders
class Doc:
    def __init__(self, name, thickness):
        self.features = []
        self.n = 0
        self.t = thickness
        self.name = name
        self.top_plane = None

    def fid(self, p):
        self.n += 1
        return f"{p}{self.n}"

    def plane_top(self):
        if not self.top_plane:
            self.top_plane = self.fid("PL")
            self.features.append({"type": "plane", "id": self.top_plane, "name": "board top face", "mode": "offset",
                                  "base": {"kind": "std", "id": "front"}, "offset": self.t, "visible": False})
        return self.top_plane

    @staticmethod
    def L(p):
        # KiCad (x, y down) -> sketch on the std front plane (XY, normal +Z): world y = -KiCad y
        return (round(p[0], 6), round(-p[1], 6))

    def sketch(self, name, plane, loops, circles=(), arcs_ok=True):
        """loops: list of closed loops, each a list of segments ('line', a, b) or ('arc', a, b, centre, mid)."""
        pts, ents = [], []

        def P(p):
            q = self.L(p)
            for d in pts:
                if abs(d["x"] - q[0]) < 1e-6 and abs(d["y"] - q[1]) < 1e-6:
                    return d["id"]
            pid = f"p{len(pts) + 1}"
            pts.append({"id": pid, "x": q[0], "y": q[1]})
            return pid

        for loop in loops:
            for seg in loop:
                eid = f"e{len(ents) + 1}"
                if seg[0] == "line":
                    ents.append({"id": eid, "type": "line", "p1": P(seg[1]), "p2": P(seg[2])})
                else:
                    _, a, b, c, m = seg
                    la, lb, lc, lm = self.L(a), self.L(b), self.L(c), self.L(m)
                    # sketch arcs run CCW from p1 to p2 (local frame); pick the order that passes the mid point
                    ang = lambda q: math.atan2(q[1] - lc[1], q[0] - lc[0])
                    sweep = (ang(lb) - ang(la)) % (2 * math.pi)
                    mid_in = (ang(lm) - ang(la)) % (2 * math.pi) < sweep
                    p1, p2 = (a, b) if mid_in else (b, a)
                    ents.append({"id": eid, "type": "arc", "c": P(c), "p1": P(p1), "p2": P(p2)})
        for c, r in circles:
            ents.append({"id": f"e{len(ents) + 1}", "type": "circle", "c": P(c), "r": round(r, 6)})
        sid = self.fid("S")
        self.features.append({"type": "sketch", "id": sid, "name": name, "plane": plane,
                              "points": pts, "entities": ents, "constraints": []})
        return sid

    def body(self, name, sketch, depth, down=False):
        self.features.append({"type": "extrude", "id": self.fid("E"), "name": name, "op": "newBody", "body": name,
                              "sketch": sketch, "end": "blind", "depth": round(depth, 4), "flip": bool(down)})

    def placed(self, name, side, z0, height, outline=None, circle=None):
        """A box (outline = 4 board-frame corners) or cylinder (circle = (centre, r)) on a board face.
        z0 = gap between the board face and the body's base."""
        base = {"kind": "std", "id": "front"}
        off = self.t + z0 if side == "top" else -z0
        if abs(off - self.t) < 1e-9:
            plane = {"kind": "feature", "id": self.plane_top()}
        elif abs(off) < 1e-9:
            plane = base
        else:
            pid = self.fid("PL")
            self.features.append({"type": "plane", "id": pid, "name": f"{name} base", "mode": "offset",
                                  "base": base, "offset": round(off, 4), "visible": False})
            plane = {"kind": "feature", "id": pid}
        if circle:
            sid = self.sketch(name, plane, [], [circle])
        else:
            loop = [("line", outline[i], outline[(i + 1) % len(outline)]) for i in range(len(outline))]
            sid = self.sketch(name, plane, [loop])
        self.body(name, sid, height, down=(side == "bottom"))

    def to_json(self, expect=None):
        d = {"format": "ck-part", "version": 1, "name": self.name, "features": self.features, "seq": self.n + 1}
        if expect:
            d["expect"] = expect
        return d


# ---------------------------------------------------------------- board outline
def chain_loops(segs, tol=1e-3):
    """segs: ('line', a, b) / ('arc', a, b, c, m); returns closed loops."""
    segs = list(segs)
    loops = []
    while segs:
        loop = [segs.pop(0)]
        start = loop[0][1]
        end = loop[0][2]
        while math.dist(start, end) > tol:
            for i, s in enumerate(segs):
                if math.dist(s[1], end) <= tol:
                    loop.append(s)
                    end = s[2]
                    segs.pop(i)
                    break
                if math.dist(s[2], end) <= tol:
                    r = (s[0], s[2], s[1]) + tuple(s[3:])
                    loop.append(r)
                    end = r[2]
                    segs.pop(i)
                    break
            else:
                raise SystemExit(f"Edge.Cuts outline is not closed near {end}")
        loops.append(loop)
    return loops


def loop_area(loop):
    pts = [s[1] for s in loop]
    return abs(sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1]
                   for i in range(len(pts)))) / 2


def board_outline(pcb):
    segs, circles = [], []
    for g in pcb[1:]:
        if not isinstance(g, list) or layer_of(g) != "Edge.Cuts":
            continue
        t = g[0]
        if t == "gr_line":
            segs.append(("line", xy(kid(g, "start")), xy(kid(g, "end"))))
        elif t == "gr_rect":
            p = graphic_pts(g)
            segs += [("line", p[i], p[(i + 1) % 4]) for i in range(4)]
        elif t == "gr_poly":
            p = graphic_pts(g)
            segs += [("line", p[i], p[(i + 1) % len(p)]) for i in range(len(p))]
        elif t == "gr_arc":
            s, m, e = xy(kid(g, "start")), xy(kid(g, "mid")), xy(kid(g, "end"))
            segs.append(("arc", s, e, circle3(s, m, e), m))
        elif t == "gr_circle":
            c, e = xy(kid(g, "center")), xy(kid(g, "end"))
            circles.append((c, math.dist(c, e)))
    loops = chain_loops(segs)
    loops.sort(key=loop_area, reverse=True)
    return loops, circles


# ---------------------------------------------------------------- footprints
def fp_info(fp):
    name = fp[1].split(":")[-1]
    at = kid(fp, "at")
    pos = (num(at[1]), num(at[2]))
    rot = num(at[3]) if len(at) > 3 else 0.0
    side = "bottom" if layer_of(fp) == "B.Cu" else "top"
    ref = None
    for p in kids(fp, "property"):
        if p[1] == "Reference":
            ref = p[2]
    fr = kid(fp, "fp_text")
    if ref is None and fr and fr[1] == "reference":
        ref = fr[2]
    descr = (kid(fp, "descr") or [None, ""])[1]
    return name, pos, rot, side, ref, descr


def fp_local_extent(fp):
    """(local bbox, source, fab circle or None) in footprint coordinates."""
    cy, fab, circ = [], [], []
    for g in fp[1:]:
        if not isinstance(g, list) or not g[0].startswith("fp_"):
            continue
        lay = layer_of(g) or ""
        if lay.endswith("CrtYd"):
            cy += graphic_pts(g)
        elif lay.endswith("Fab"):
            fab += graphic_pts(g)
            if g[0] == "fp_circle":
                c, e = xy(kid(g, "center")), xy(kid(g, "end"))
                circ.append((c, math.dist(c, e)))
    pads = []
    for p in kids(fp, "pad"):
        at, size = kid(p, "at"), kid(p, "size")
        if not at or not size:
            continue
        c = (num(at[1]), num(at[2]))
        prot = (num(at[3]) if len(at) > 3 else 0.0)
        w, h = num(size[1]) / 2, num(size[2]) / 2
        for q in [(-w, -h), (w, -h), (w, h), (-w, h)]:
            # pad angle in the file includes the footprint rotation; remove it to stay in local coords
            r = rot_pt(q, prot - fp_info(fp)[2])
            pads.append((c[0] + r[0], c[1] + r[1]))
    big = max(circ, key=lambda c: c[1]) if circ else None
    if cy:
        return bbox(cy), "courtyard", big
    if pads:
        return bbox(pads), "pads", big
    if fab:
        return bbox(fab), "fab", big
    return None, None, big


def fp_holes(fp, pth):
    name, pos, rot, side, ref, _ = fp_info(fp)
    out = []
    for p in kids(fp, "pad"):
        kind = p[2]
        if kind != "np_thru_hole" and not (pth and kind == "thru_hole"):
            continue
        dr = kid(p, "drill")
        if not dr or len(dr) < 2 or dr[1] == "oval":
            continue  # slots are not drilled in this simple model
        at = kid(p, "at")
        c = rot_pt((num(at[1]), num(at[2])), rot)
        out.append(((pos[0] + c[0], pos[1] + c[1]), num(dr[1]) / 2))
    return out


def lookup(name, descr):
    shape = None
    h = None
    src = "default"
    for rx, hh, sh in HEIGHTS:
        if re.search(rx, name):
            if sh == "skip":
                return None, "skip", "table"
            h, shape, src = hh, sh, "table"
            break
    m = re.search(r"height[ =:]*([\d.]+)\s*mm", descr or "", re.I)
    if m:
        h, src = float(m.group(1)), "descr"
    return (h if h is not None else DEFAULT_HEIGHT), shape, src


def world_corners(pos, rot, bb):
    x0, y0, x1, y1 = bb
    out = []
    for q in [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]:
        r = rot_pt(q, rot)
        out.append((pos[0] + r[0], pos[1] + r[1]))
    return out


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("pcb")
    ap.add_argument("out")
    ap.add_argument("--thickness", type=float, default=None, help="board thickness (default: the file's, else 1.6)")
    ap.add_argument("--extras")
    ap.add_argument("--report")
    ap.add_argument("--name")
    ap.add_argument("--pth-holes", action="store_true", help="also drill plated through holes (slow on big boards)")
    ap.add_argument("--skip", help="regex of references to leave out")
    a = ap.parse_args()

    pcb = parse_sexpr(open(a.pcb, encoding="utf-8").read())
    gen = kid(pcb, "general")
    th = a.thickness or (num(kid(gen, "thickness")[1]) if gen is not None and kid(gen, "thickness") else 1.6)
    name = a.name or re.sub(r"\.kicad_pcb$", "", a.pcb.split("/")[-1])
    doc = Doc(name, th)
    report = {"source": a.pcb.split("/")[-1], "thickness": th, "bodies": [], "skipped": [], "holes": []}

    # board
    loops, edge_circles = board_outline(pcb)
    holes = [(c, r) for c, r in edge_circles]
    fps = kids(pcb, "footprint")
    for fp in fps:
        holes += fp_holes(fp, a.pth_holes)
    bb = bbox([s[1] for s in loops[0]])
    report["outline"] = {"min": bb[:2], "max": bb[2:], "size": [bb[2] - bb[0], bb[3] - bb[1]],
                         "loops": len(loops), "segments": len(loops[0])}
    report["holes"] = [{"at": [round(c[0], 4), round(c[1], 4)], "d": round(2 * r, 4)} for c, r in holes]
    sid = doc.sketch("BOARD outline", {"kind": "std", "id": "front"}, loops, holes)
    doc.body("BOARD", sid, th)
    area = loop_area(loops[0]) - sum(loop_area(l) for l in loops[1:]) - sum(math.pi * r * r for _, r in holes)
    report["board_volume"] = area * th

    # keep-outs
    seen = {}
    for z in kids(pcb, "zone"):
        if not kid(z, "keepout"):
            continue
        zn = (kid(z, "name") or [None, "zone"])[1]
        lays = [layer_of(z)] if layer_of(z) else list((kid(z, "layers") or [None])[1:])
        pts = [xy(p) for p in kids(kid(kid(z, "polygon"), "pts"), "xy")]
        key = (zn, tuple(pts))
        seen.setdefault(key, set()).update(lays)
    used = set()
    for (zn, pts), lays in seen.items():
        sides = ["top"] if any(l.startswith("F.") or l == "*.Cu" or "F&B" in l for l in lays) else []
        if any(l.startswith("B.") or l == "*.Cu" or "F&B" in l for l in lays):
            sides.append("bottom")
        for side in sides:
            nm = re.sub(r"[^\w\-]+", "_", f"KEEPOUT_{zn}") + ("_B" if side == "bottom" else "")
            while nm in used:
                nm += "_"
            used.add(nm)
            doc.placed(nm, side, 0, 0.1, outline=list(pts))
            report["bodies"].append({"name": nm, "kind": "keepout", "layers": sorted(lays), "side": side,
                                     "outline": [list(p) for p in pts], "height": 0.1})

    # footprints
    names = set(used) | {"BOARD"}
    skip = re.compile(a.skip) if a.skip else None
    extents = {}
    for fp in fps:
        fname, pos, rot, side, ref, descr = fp_info(fp)
        ref = ref or fname
        bb, src, circ = fp_local_extent(fp)
        if bb:
            extents[ref] = world_corners(pos, rot, bb)
        h, shape, hsrc = lookup(fname, descr)
        if shape == "skip" or (skip and skip.search(ref)) or not bb:
            report["skipped"].append({"ref": ref, "footprint": fname, "why": "skip" if bb else "no geometry"})
            continue
        nm = re.sub(r"[^\w\-.+]+", "_", ref)
        while nm in names:
            nm += "_dup"
        names.add(nm)
        entry = {"name": nm, "ref": ref, "footprint": fname, "side": side, "at": list(pos), "rot": rot,
                 "size_from": src, "height": h, "height_from": hsrc}
        cw, ch = bb[2] - bb[0], bb[3] - bb[1]
        if circ and (shape == "cyl" or (shape is None and 2 * circ[1] >= 0.8 * max(cw, ch))):
            c = rot_pt(circ[0], rot)
            centre = (pos[0] + c[0], pos[1] + c[1])
            doc.placed(nm, side, 0, h, circle=(centre, circ[1]))
            entry.update(shape="cyl", centre=list(centre), d=2 * circ[1], size_from="fab circle")
        else:
            corners = world_corners(pos, rot, bb)
            doc.placed(nm, side, 0, h, outline=corners)
            entry.update(shape="box", size=[round(cw, 3), round(ch, 3)])
        report["bodies"].append(entry)

    # extras (module envelopes and anything not in the footprint list)
    if a.extras:
        for e in json.load(open(a.extras)).get("bodies", []):
            side = e.get("side", "top")
            rot = e.get("rot", 0.0)
            if "over" in e:
                pts = [p for r in e["over"] for p in extents[r]]
                b = bbox(pts)
                m = e.get("margin", 0.0)
                centre = ((b[0] + b[2]) / 2, (b[1] + b[3]) / 2)
                size = e.get("size") or [b[2] - b[0] + 2 * m, b[3] - b[1] + 2 * m]
            else:
                centre = tuple(e["at"])
                size = e["size"]
            centre = (centre[0] + e.get("offset", [0, 0])[0], centre[1] + e.get("offset", [0, 0])[1])
            if e.get("shape") == "cyl":
                doc.placed(e["name"], side, e.get("z0", 0), e["height"], circle=(centre, size[0] / 2))
            else:
                w, l = size[0] / 2, size[1] / 2
                corners = [(centre[0] + q[0], centre[1] + q[1]) for q in
                           (rot_pt(p, rot) for p in [(-w, -l), (w, -l), (w, l), (-w, l)])]
                doc.placed(e["name"], side, e.get("z0", 0), e["height"], outline=corners)
            report["bodies"].append({"name": e["name"], "kind": "extra", "centre": list(centre), "size": list(size),
                                     "rot": rot, "z0": e.get("z0", 0), "height": e["height"], "side": side,
                                     "note": e.get("note", "")})

    expect = [{"kind": "valid"}]
    json.dump(doc.to_json(expect), open(a.out, "w"), indent=1)
    if a.report:
        json.dump(report, open(a.report, "w"), indent=1)
    print(f"{a.out}: board {report['outline']['size'][0]:.3f} x {report['outline']['size'][1]:.3f} x {th} mm, "
          f"{len(holes)} holes, {len(report['bodies'])} bodies, {len(report['skipped'])} skipped", file=sys.stderr)


if __name__ == "__main__":
    main()
