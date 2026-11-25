// SPDX-License-Identifier: MIT
//! Box, cylinder, sphere, torus and coil primitives (mitcad#25) *(read from
//! the controlled variant models: one option of the placement, the sizes,
//! the operation, the torus position and the coil choices each)*.
//!
//! The record of a box, cylinder, sphere or torus, after its root part:
//!
//! ```text
//! ... | u32 operation | u8 1 | ref .. ref | u8 flag | [f64[16]] | ...
//! ```
//!
//! The operation is 1 join, 2 cut, 3 intersect, 4 new body, 5 new
//! component. The references (a run of them, none gap between): the
//! primitive's parameter holders, the frame input ([`FRAME_INPUT`]: the
//! frame of the plane or face the primitive stands on, a row-major 4x4
//! matrix with the plane's origin; no matrix stored: the identity, the
//! origin XY plane) and a definition object; after the run `flag` is 1 for
//! the identity or 0 with the primitive's own placement as a row-major 4x4
//! matrix (cm) in that frame. A torus has `u32 position` after that (0 on
//! the centre line, 1 outside, 2 inside).
//!
//! A point `p` of the primitive's own frame is at `frame * placement * p`
//! in the component's coordinates. The own frame: a box's width
//! (`DistanceX`) along x, its length (`DistanceY`) along y and its height
//! (`DistanceZ`) along z; a cylinder's base centre, a sphere's centre and a
//! torus' centre are the origin, the axes the z axis. The plane and the
//! bodies a cut or join works on are among the item's inputs.
//!
//! A coil: `.. | u8 flip | u32 operation | u32 type | holders ..| frame
//! input | u8 flag | [f64[16]] | [ref] | u32 position | ref | u32 section |
//! ..`. `flip` is 1 when the rotation is flipped (a left-handed helix),
//! `type` 1 revolutions and height, 2 revolutions and pitch, 3 height and
//! pitch, 4 a spiral; `position` 1 inside, 2 on centre, 3 outside; `section`
//! 1 circular, 2 square, 3 triangular outside, 4 triangular inside.

use super::super::classes::*;
use super::super::decode::TimelineEntry;
use super::super::ir::*;
use super::super::stream::{Segment, f64s_at, u32_at};
use super::{Builder, face_input, paramref, rigid_matrix};

/// `u8 flag | [f64[16]]` at `p`: `Some((None, end))` the identity,
/// `Some((Some(matrix), end))` a matrix; `None` where it is neither.
fn placement_at(d: &[u8], p: usize) -> Option<(Option<Mat4>, usize)> {
    match *d.get(p)? {
        1 => Some((None, p + 1)),
        0 => {
            let m = f64s_at(d, p + 1, 16)?;
            let last = m[12] == 0.0 && m[13] == 0.0 && m[14] == 0.0 && m[15] == 1.0;
            if !last || !m.iter().all(|x| x.is_finite()) {
                return None;
            }
            let mut out = [[0.0; 4]; 4];
            for (r, row) in out.iter_mut().enumerate() {
                row.copy_from_slice(&m[4 * r..4 * r + 4]);
            }
            Some((Some(out), p + 129))
        }
        _ => None,
    }
}

/// The operation of a primitive's or coil's code.
fn operation_name(code: u32) -> Option<&'static str> {
    Some(match code {
        1 => "JoinFeatureOperation",
        2 => "CutFeatureOperation",
        3 => "IntersectFeatureOperation",
        4 => "NewBodyFeatureOperation",
        5 => "NewComponentFeatureOperation",
        _ => return None,
    })
}

/// The fields of a primitive's record (see the module documentation).
struct Fields {
    operation: Option<&'static str>,
    /// The frame input among the references of the run.
    frame_input: Option<u64>,
    /// The primitive's own placement; `None` the identity.
    placement: Option<Mat4>,
    /// Where the fields after the placement start.
    after: usize,
}

/// Reads a box's, cylinder's, sphere's or torus' fields: the first
/// reference preceded by `u32 operation | u8 1` starts the run.
fn fields(seg: &Segment, d: &[u8], from: usize, to: usize) -> Option<Fields> {
    let refs = seg.refs_in(d, from, to);
    let (k, (first, _)) = refs.iter().enumerate().find(|(_, (p, _))| {
        // `u32 operation | u8 1` in front of the reference.
        *p >= 5 && d[*p - 1] == 1 && u32_at(d, p - 5).is_some_and(|c| operation_name(c).is_some())
    })?;
    let operation = operation_name(u32_at(d, first - 5)?);
    // The references that follow each other without a gap.
    let mut run_end = refs[k].1.end;
    let mut frame_input = (seg.guid_of(refs[k].1.id) == Some(FRAME_INPUT)).then_some(refs[k].1.id);
    for (p, r) in &refs[k + 1..] {
        if *p != run_end {
            break;
        }
        run_end = r.end;
        if seg.guid_of(r.id) == Some(FRAME_INPUT) {
            frame_input = Some(r.id);
        }
    }
    let (placement, after) = placement_at(d, run_end)?;
    Some(Fields {
        operation,
        frame_input,
        placement,
        after,
    })
}

impl Builder<'_> {
    /// The plane or face an item stands on: its first plane or face input.
    fn standing_plane(&self, it: &TimelineEntry) -> Option<Reference> {
        self.item_inputs(it)
            .into_iter()
            .find_map(|i| match self.seg.guid_of(i) {
                Some(ENTITY_REF) => self.plane_ref(i),
                Some(FACE_REF) => face_input(self.seg, i),
                _ => None,
            })
    }

    /// The frame input's matrix (the rotation of the plane's frame): `None`
    /// when it stores none, the identity.
    fn frame_matrix(&self, frame_input: Option<u64>) -> Option<Mat4> {
        rigid_matrix(self.seg.data_of(frame_input?))
    }

    /// A box, cylinder, sphere or torus (see the module documentation).
    pub(super) fn primitive(&self, it: &TimelineEntry) -> Detail {
        let seg = self.seg;
        let rl = self.roles(it.id);
        let pr = |role: &str| rl.get(role).and_then(|v| v.first()).map(|p| paramref(p));
        let mut x = PrimitiveDetail::default();
        let d = seg.data_of(it.id);
        let Some(root) = seg.root_part(d) else {
            return Detail::Primitive(Box::new(x));
        };
        let end = it.tail.as_ref().map_or(d.len(), |t| t.start.min(d.len()));
        if let Some(f) = fields(seg, d, root.end, end) {
            x.operation = f.operation.map(str::to_owned);
            x.placement = f.placement;
            x.frame = self.frame_matrix(f.frame_input);
            if it.class == TORUS {
                x.torus_position = match u32_at(d, f.after) {
                    Some(0) => Some("CenterTorusPosition".to_owned()),
                    Some(1) => Some("OutsideTorusPosition".to_owned()),
                    Some(2) => Some("InsideTorusPosition".to_owned()),
                    _ => None,
                };
            }
        }
        x.plane = self.standing_plane(it);
        x.participant_bodies = self.body_inputs(it);
        x.distance_x = pr("DistanceX");
        x.distance_y = pr("DistanceY");
        x.distance_z = pr("DistanceZ");
        x.offset_distance_x = pr("OffsetDistanceX");
        x.offset_distance_y = pr("OffsetDistanceY");
        x.height = pr("Height");
        x.diameter = pr("Diameter");
        x.major_diameter = pr("MajorDiameter");
        x.minor_diameter = pr("MinorDiameter");
        Detail::Primitive(Box::new(x))
    }

    /// A coil (see the module documentation).
    pub(super) fn coil(&self, it: &TimelineEntry) -> Detail {
        let seg = self.seg;
        let rl = self.roles(it.id);
        let pr = |role: &str| rl.get(role).and_then(|v| v.first()).map(|p| paramref(p));
        let mut x = CoilDetail {
            diameter: pr("Diameter"),
            pitch: pr("Pitch"),
            revolutions: pr("Revolutions"),
            height: pr("Height"),
            section_size: pr("SectionSize"),
            angle: pr("TaperAngle"),
            ..CoilDetail::default()
        };
        let d = seg.data_of(it.id);
        if let Some(root) = seg.root_part(d) {
            self.coil_fields(it, d, root.end, &mut x);
        }
        x.plane = self.standing_plane(it);
        x.participant_bodies = self.body_inputs(it);
        Detail::Coil(Box::new(x))
    }

    /// The choices of a coil's record: `u8 flip | u32 operation | u32 type`
    /// before the first reference, the frame input, its placement and
    /// `[ref] | u32 position | ref | u32 section` after it.
    fn coil_fields(&self, it: &TimelineEntry, d: &[u8], from: usize, x: &mut CoilDetail) {
        let seg = self.seg;
        let end = it.tail.as_ref().map_or(d.len(), |t| t.start.min(d.len()));
        // The operation and the type, followed by the first holder.
        let Some(q) = (from.max(1)..end.saturating_sub(12)).find(|&q| {
            u32_at(d, q).is_some_and(|c| operation_name(c).is_some())
                && u32_at(d, q + 4).is_some_and(|t| (1..=4).contains(&t))
                && seg.ref_at(d, q + 8).is_some()
        }) else {
            return;
        };
        x.operation = u32_at(d, q).and_then(operation_name).map(str::to_owned);
        x.coil_type = u32_at(d, q + 4)
            .and_then(|t| match t {
                1 => Some("RevolutionAndHeightCoilType"),
                2 => Some("PitchAndRevolutionCoilType"),
                3 => Some("PitchAndHeightCoilType"),
                4 => Some("SpiralCoilType"),
                _ => None,
            })
            .map(str::to_owned);
        // `isClockwise`: the default rotation is right-handed.
        x.is_clockwise = match d[q - 1] {
            0 => Some(true),
            1 => Some(false),
            _ => None,
        };
        let Some((_, frame)) = seg
            .refs_in(d, q + 8, end)
            .into_iter()
            .find(|(_, r)| seg.guid_of(r.id) == Some(FRAME_INPUT))
        else {
            return;
        };
        x.frame = self.frame_matrix(Some(frame.id));
        let Some((placement, p)) = placement_at(d, frame.end) else {
            return;
        };
        x.placement = placement;
        // `[ref | 00] u32 position | ref | u32 section`.
        let q = match seg.ref_at(d, p) {
            Some(r) => r.end,
            None if d.get(p) == Some(&0) => p + 1,
            None => return,
        };
        x.section_position = u32_at(d, q)
            .and_then(|c| match c {
                1 => Some("InsideCoilSectionPosition"),
                2 => Some("CenterCoilSectionPosition"),
                3 => Some("OutsideCoilSectionPosition"),
                _ => None,
            })
            .map(str::to_owned);
        if let Some(b) = seg.ref_at(d, q + 4) {
            x.section_type = u32_at(d, b.end)
                .and_then(|c| match c {
                    1 => Some("CircularCoilSectionType"),
                    2 => Some("SquareCoilSectionType"),
                    3 => Some("TriangleExternalCoilSectionType"),
                    4 => Some("TriangleInternalCoilSectionType"),
                    _ => None,
                })
                .map(str::to_owned);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::Doc;

    const ROOT: &str = "98542EB9-A4F2-4137-A808-DBB5B3CD6159";

    fn r(id: u64) -> Vec<u8> {
        let mut v = vec![1];
        v.extend(id.to_le_bytes());
        v.extend([0, 0]);
        v
    }

    /// A box record: root part, `u32 operation | u8 1`, three holders and a
    /// frame input in a run, then the placement.
    fn record(operation: u32, placement: &[u8]) -> Vec<u8> {
        let mut d = vec![0, 0];
        d.extend([0, 0, 0, 0, 0]);
        d.extend(operation.to_le_bytes());
        d.push(1);
        for id in [41, 41, 41, 42] {
            d.extend(r(id));
        }
        d.extend(placement);
        d.extend([0, 0, 0, 0]);
        d
    }

    fn matrix(rows: [[f64; 4]; 4]) -> Vec<u8> {
        let mut v = vec![0];
        v.extend(rows.iter().flatten().flat_map(|x| x.to_le_bytes()));
        v
    }

    fn doc(body: Vec<u8>) -> Doc {
        let mut d = Doc::default();
        d.class(ROOT, "", 0);
        d.class(PARAMETER_HOLDER, ROOT, 0);
        d.class(FRAME_INPUT, ROOT, 15);
        d.class(BOX, ROOT, 0);
        d.obj(41, PARAMETER_HOLDER, vec![0, 0]);
        d.obj(42, FRAME_INPUT, vec![0, 0]);
        d.obj(50, BOX, body);
        d
    }

    #[test]
    fn a_primitive_has_its_operation_frame_input_and_placement() {
        let m = [
            [0.0, -1.0, 0.0, 2.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let seg = doc(record(2, &matrix(m))).segment();
        let d = seg.data_of(50);
        let root = seg.root_part(d).unwrap();
        let f = fields(&seg, d, root.end, d.len()).unwrap();
        assert_eq!(f.operation, Some("CutFeatureOperation"));
        assert_eq!(f.frame_input, Some(42));
        assert_eq!(f.placement, Some(m));
        // After the 16 doubles.
        assert_eq!(f.after, d.len() - 4);
    }

    #[test]
    fn a_placement_flag_of_one_is_the_identity() {
        let seg = doc(record(4, &[1])).segment();
        let d = seg.data_of(50);
        let root = seg.root_part(d).unwrap();
        let f = fields(&seg, d, root.end, d.len()).unwrap();
        assert_eq!(f.operation, Some("NewBodyFeatureOperation"));
        assert_eq!(f.placement, None);
        assert_eq!(f.after, d.len() - 4);
    }

    #[test]
    fn a_matrix_that_is_no_placement_is_refused() {
        // The last row is not 0 0 0 1.
        let mut m = matrix([[1.0; 4]; 4]);
        m[0] = 0;
        assert_eq!(placement_at(&m, 0), None);
        assert_eq!(placement_at(&[2], 0), None);
        assert_eq!(operation_name(6), None);
        assert_eq!(operation_name(5), Some("NewComponentFeatureOperation"));
    }
}
