//! Minimal TrueType (glyf) reader: head, maxp, hhea, hmtx, loca, glyf (simple and composite glyphs) and
//! cmap (formats 4 and 12). Enough for the two baked-in subset fonts; written from the OpenType
//! specification, no third-party code.



/// One outline point of a glyph, in font units.
#[derive(Clone, Copy, Debug)]
pub struct Pt {
    pub x: f64,
    pub y: f64,
    pub on: bool,
}

/// The contours of a glyph (each a closed list of on/off-curve points).
pub type Contours = Vec<Vec<Pt>>;

pub struct Font {
    data: &'static [u8],
    pub units_per_em: f64,
    num_glyphs: usize,
    loca_long: bool,
    num_hmetrics: usize,
    loca: usize,
    glyf: usize,
    hmtx: usize,
    cmap: usize,
    cap_height: f64,
}

fn u16_at(d: &[u8], o: usize) -> Option<u16> {
    Some(u16::from_be_bytes([*d.get(o)?, *d.get(o + 1)?]))
}
fn i16_at(d: &[u8], o: usize) -> Option<i16> {
    u16_at(d, o).map(|v| v as i16)
}
fn u32_at(d: &[u8], o: usize) -> Option<u32> {
    Some(u32::from_be_bytes([*d.get(o)?, *d.get(o + 1)?, *d.get(o + 2)?, *d.get(o + 3)?]))
}

impl Font {
    pub fn parse(data: &'static [u8]) -> Option<Font> {
        let n = u16_at(data, 4)? as usize;
        let table = |tag: &[u8; 4]| -> Option<usize> {
            for i in 0..n {
                let r = 12 + i * 16;
                if data.get(r..r + 4)? == tag {
                    return Some(u32_at(data, r + 8)? as usize);
                }
            }
            None
        };
        let head = table(b"head")?;
        let maxp = table(b"maxp")?;
        let hhea = table(b"hhea")?;
        let mut f = Font {
            data,
            units_per_em: u16_at(data, head + 18)? as f64,
            num_glyphs: u16_at(data, maxp + 4)? as usize,
            loca_long: i16_at(data, head + 50)? != 0,
            num_hmetrics: u16_at(data, hhea + 34)? as usize,
            loca: table(b"loca")?,
            glyf: table(b"glyf")?,
            hmtx: table(b"hmtx")?,
            cmap: table(b"cmap")?,
            cap_height: 0.0,
        };
        let h = f.glyph_index('H')?;
        let c = f.contours(h)?;
        f.cap_height = c.iter().flatten().map(|p| p.y).fold(0.0, f64::max);
        if f.cap_height <= 0.0 || f.units_per_em <= 0.0 {
            return None;
        }
        Some(f)
    }

    /// Height of the capital letters (the 'H' glyph), in font units.
    pub fn cap_height(&self) -> f64 {
        self.cap_height
    }

    /// Glyph id of a character (`None`: the font has no glyph for it).
    pub fn glyph_index(&self, ch: char) -> Option<u16> {
        let d = self.data;
        let cp = ch as u32;
        let nt = u16_at(d, self.cmap + 2)? as usize;
        let mut best: Option<(u32, usize)> = None;
        for i in 0..nt {
            let r = self.cmap + 4 + i * 8;
            let (plat, enc, off) = (u16_at(d, r)?, u16_at(d, r + 2)?, u32_at(d, r + 4)? as usize);
            let rank = match (plat, enc) {
                (3, 10) => 3,
                (0, 4) | (0, 6) => 3,
                (3, 1) => 2,
                (0, _) => 1,
                _ => 0,
            };
            if rank > 0 && best.map_or(true, |b| rank > b.0) {
                best = Some((rank, self.cmap + off));
            }
        }
        let st = best?.1;
        let g = match u16_at(d, st)? {
            4 => {
                if cp > 0xFFFF {
                    return None;
                }
                let sc = u16_at(d, st + 6)? as usize / 2;
                let ends = st + 14;
                let starts = ends + sc * 2 + 2;
                let deltas = starts + sc * 2;
                let ranges = deltas + sc * 2;
                let mut found = None;
                for s in 0..sc {
                    if cp <= u16_at(d, ends + s * 2)? as u32 {
                        let start = u16_at(d, starts + s * 2)? as u32;
                        if cp < start {
                            break;
                        }
                        let delta = u16_at(d, deltas + s * 2)?;
                        let ro = u16_at(d, ranges + s * 2)? as usize;
                        let g = if ro == 0 {
                            (cp as u16).wrapping_add(delta)
                        } else {
                            let a = ranges + s * 2 + ro + (cp - start) as usize * 2;
                            let g = u16_at(d, a)?;
                            if g == 0 {
                                0
                            } else {
                                g.wrapping_add(delta)
                            }
                        };
                        found = Some(g);
                        break;
                    }
                }
                found?
            }
            12 => {
                let ng = u32_at(d, st + 12)? as usize;
                let mut found = None;
                for gi in 0..ng {
                    let r = st + 16 + gi * 12;
                    let (a, b, g0) = (u32_at(d, r)?, u32_at(d, r + 4)?, u32_at(d, r + 8)?);
                    if cp >= a && cp <= b {
                        found = Some((g0 + cp - a) as u16);
                        break;
                    }
                }
                found?
            }
            _ => return None,
        };
        (g != 0 && (g as usize) < self.num_glyphs).then_some(g)
    }

    /// Horizontal advance of a glyph, font units.
    pub fn advance(&self, g: u16) -> f64 {
        let i = (g as usize).min(self.num_hmetrics.saturating_sub(1));
        u16_at(self.data, self.hmtx + i * 4).unwrap_or(0) as f64
    }

    fn glyph_range(&self, g: u16) -> Option<(usize, usize)> {
        let g = g as usize;
        if g >= self.num_glyphs {
            return None;
        }
        let (a, b) = if self.loca_long {
            (u32_at(self.data, self.loca + g * 4)? as usize, u32_at(self.data, self.loca + g * 4 + 4)? as usize)
        } else {
            (u16_at(self.data, self.loca + g * 2)? as usize * 2, u16_at(self.data, self.loca + g * 2 + 2)? as usize * 2)
        };
        Some((self.glyf + a, self.glyf + b))
    }

    /// Outline of a glyph (composites resolved), font units, y up. Empty for blank glyphs (space).
    pub fn contours(&self, g: u16) -> Option<Contours> {
        self.contours_depth(g, 0)
    }

    fn contours_depth(&self, g: u16, depth: u32) -> Option<Contours> {
        let d = self.data;
        let (a, b) = self.glyph_range(g)?;
        if b <= a {
            return Some(vec![]);
        }
        let nc = i16_at(d, a)?;
        if nc >= 0 {
            let nc = nc as usize;
            let mut end_pts = Vec::with_capacity(nc);
            for i in 0..nc {
                end_pts.push(u16_at(d, a + 10 + i * 2)? as usize);
            }
            let np = end_pts.last().map_or(0, |e| e + 1);
            let il = u16_at(d, a + 10 + nc * 2)? as usize;
            let mut p = a + 12 + nc * 2 + il;
            let mut flags = Vec::with_capacity(np);
            while flags.len() < np {
                let f = *d.get(p)?;
                p += 1;
                flags.push(f);
                if f & 8 != 0 {
                    let r = *d.get(p)?;
                    p += 1;
                    for _ in 0..r {
                        flags.push(f);
                    }
                }
            }
            flags.truncate(np);
            let mut xs = Vec::with_capacity(np);
            let mut v = 0i32;
            for &f in &flags {
                if f & 2 != 0 {
                    let dx = *d.get(p)? as i32;
                    p += 1;
                    v += if f & 16 != 0 { dx } else { -dx };
                } else if f & 16 == 0 {
                    v += i16_at(d, p)? as i32;
                    p += 2;
                }
                xs.push(v);
            }
            let mut ys = Vec::with_capacity(np);
            v = 0;
            for &f in &flags {
                if f & 4 != 0 {
                    let dy = *d.get(p)? as i32;
                    p += 1;
                    v += if f & 32 != 0 { dy } else { -dy };
                } else if f & 32 == 0 {
                    v += i16_at(d, p)? as i32;
                    p += 2;
                }
                ys.push(v);
            }
            let mut out = Vec::with_capacity(nc);
            let mut s = 0;
            for &e in &end_pts {
                if e < s || e >= np {
                    return None;
                }
                out.push((s..=e).map(|i| Pt { x: xs[i] as f64, y: ys[i] as f64, on: flags[i] & 1 != 0 }).collect());
                s = e + 1;
            }
            Some(out)
        } else {
            if depth > 4 {
                return None;
            }
            let mut out: Contours = vec![];
            let mut p = a + 10;
            loop {
                let fl = u16_at(d, p)?;
                let gi = u16_at(d, p + 2)?;
                p += 4;
                let (mut dx, mut dy) = (0.0, 0.0);
                if fl & 1 != 0 {
                    if fl & 2 != 0 {
                        dx = i16_at(d, p)? as f64;
                        dy = i16_at(d, p + 2)? as f64;
                    }
                    p += 4;
                } else {
                    if fl & 2 != 0 {
                        dx = *d.get(p)? as i8 as f64;
                        dy = *d.get(p + 1)? as i8 as f64;
                    }
                    p += 2;
                }
                let f2 = |o: usize| i16_at(d, o).map(|v| v as f64 / 16384.0);
                let (mut m00, mut m01, mut m10, mut m11) = (1.0, 0.0, 0.0, 1.0);
                if fl & 0x8 != 0 {
                    m00 = f2(p)?;
                    m11 = m00;
                    p += 2;
                } else if fl & 0x40 != 0 {
                    m00 = f2(p)?;
                    m11 = f2(p + 2)?;
                    p += 4;
                } else if fl & 0x80 != 0 {
                    m00 = f2(p)?;
                    m01 = f2(p + 2)?;
                    m10 = f2(p + 4)?;
                    m11 = f2(p + 6)?;
                    p += 8;
                }
                for c in self.contours_depth(gi, depth + 1)? {
                    out.push(
                        c.into_iter()
                            .map(|q| Pt { x: m00 * q.x + m10 * q.y + dx, y: m01 * q.x + m11 * q.y + dy, on: q.on })
                            .collect(),
                    );
                }
                if fl & 0x20 == 0 {
                    break;
                }
            }
            Some(out)
        }
    }
}
