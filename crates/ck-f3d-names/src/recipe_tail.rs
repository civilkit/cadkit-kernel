// SPDX-License-Identifier: MIT
//! The tail of an edge recipe (mitcad#106): the faces round the edge's
//! ends. The design streams' recipes (`mitcad_f3d::design::recipe`) carry
//! it; [`crate::NamedState`] finds edges by it.

/// What an edge recipe's tail says about the edge's neighbourhood
/// (mitcad#106): which of the entities are the two faces
/// along the edge, and for each end vertex the faces around it in cyclic
/// order. All indices are into the recipe's entity list.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EdgeTail {
    /// The entities of the two faces along the edge.
    pub faces: [usize; 2],
    /// Per end vertex (the edge's start first), the entities of the faces
    /// around it, in cyclic order starting where the file starts them.
    pub fans: Vec<Vec<usize>>,
}

/// Reads an edge recipe's tail (little-endian `i32` words):
///
/// ```text
/// o f | 2, a f, b f | v f |
/// v × ( k, k × (i f) | c, c × (i f 0 0 f) | d, d × (8 words) | f )
/// ```
///
/// where each flag `f` is -1 or 0. `o` is -1, or the entity of the edge
/// itself where the recipe's first entity names the edge (a negative tag),
/// `a` and `b` are the entities of the two faces along the edge. Per end
/// vertex (the edge's start first), `k` entity indices give the faces
/// around it in cyclic order. The rest of a vertex's record (named edges,
/// and per pair of faces a code and two triples) is skipped by its framing.
/// `None` when the words do not follow the layout *(all 1,559 distinct
/// tails of the corpora do)*.
pub fn edge_tail(tail: &[u8]) -> Option<EdgeTail> {
    let (words, rest) = tail.as_chunks::<4>();
    if !rest.is_empty() {
        return None;
    }
    let w: Vec<i64> = words
        .iter()
        .map(|c| i64::from(i32::from_le_bytes(*c)))
        .collect();
    let at = |p: usize| w.get(p).copied();
    let flag = |p: usize| at(p).filter(|f| matches!(f, -1 | 0));
    let index = |v: i64| usize::try_from(v).ok();
    if at(0)? < -1 || flag(1).is_none() || at(2)? != 2 || flag(4).is_none() || flag(6).is_none() {
        return None;
    }
    let faces = [index(at(3)?)?, index(at(5)?)?];
    let vertices = index(at(7)?)?;
    if flag(8).is_none() || vertices > 2 {
        return None;
    }
    let mut p = 9;
    let mut fans = Vec::new();
    for _ in 0..vertices {
        let k = index(at(p)?)?;
        p += 1;
        let mut fan = Vec::new();
        for _ in 0..k {
            fan.push(index(at(p)?)?);
            flag(p + 1)?;
            p += 2;
        }
        let named = index(at(p)?)?;
        p += 1;
        for _ in 0..named {
            if at(p)? < 0 || flag(p + 1).is_none() || at(p + 2)? != 0 || at(p + 3)? != 0 {
                return None;
            }
            flag(p + 4)?;
            p += 5;
        }
        let pairs = index(at(p)?)?;
        p += 1 + 8 * pairs;
        flag(p)?;
        p += 1;
        fans.push(fan);
    }
    (p == w.len()).then_some(EdgeTail { faces, fans })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn words(w: &[i32]) -> Vec<u8> {
        w.iter().flat_map(|x| x.to_le_bytes()).collect()
    }

    #[test]
    fn reads_the_faces_round_the_ends_of_an_edge() {
        // Two ends of three faces each, no named edges, no pairs.
        let tail = words(&[
            -1, -1, 2, 0, -1, 1, -1, 2, -1, // the faces along the edge, two ends
            3, 2, -1, 0, -1, 1, -1, 0, 0, -1, // the start: faces 2, 0, 1
            3, 0, -1, 3, -1, 1, -1, 0, 0, -1, // the end: faces 0, 3, 1
        ]);
        let t = edge_tail(&tail).unwrap();
        assert_eq!(t.faces, [0, 1]);
        assert_eq!(t.fans, vec![vec![2, 0, 1], vec![0, 3, 1]]);
    }

    #[test]
    fn skips_named_edges_and_pairs_by_their_framing() {
        // Flags 0 where -1 is usual, a named edge (entity 4, then 0 0) and
        // two pairs of eight words at the first end; one end only.
        let tail = words(&[
            -1, -1, 2, 0, 0, 1, -1, 1, -1, // one end
            4, 0, -1, 1, 0, 2, -1, 3, -1, // four faces
            1, 4, -1, 0, 0, -1, // one named edge
            2, 0, 4, 1, 0, 1, 2, 1, 2, 1, 4, 3, 2, 3, 3, 2, 3, // two pairs
            -1,
        ]);
        let t = edge_tail(&tail).unwrap();
        assert_eq!(t.fans, vec![vec![0, 1, 2, 3]]);
        // Where the recipe's first entity is the edge itself the header
        // says which entity it is, and the faces are numbered after it.
        let tail = words(&[0, -1, 2, 1, -1, 2, -1, 1, -1, 2, 1, -1, 2, -1, 0, 0, -1]);
        let t = edge_tail(&tail).unwrap();
        assert_eq!(t.faces, [1, 2]);
        assert_eq!(t.fans, vec![vec![1, 2]]);
    }

    #[test]
    fn refuses_tails_that_do_not_follow_the_layout() {
        let ok = [-1, -1, 2, 0, -1, 1, -1, 1, -1, 2, 0, -1, 1, -1, 0, 0, -1];
        assert!(edge_tail(&words(&ok)).is_some());
        // Cut short, one word too many, bytes that are not whole words,
        // a count that runs past the end, a flag that is not 0 or -1.
        assert!(edge_tail(&words(&ok[..ok.len() - 1])).is_none());
        let mut long = ok.to_vec();
        long.push(-1);
        assert!(edge_tail(&words(&long)).is_none());
        let mut odd = words(&ok);
        odd.push(0);
        assert!(edge_tail(&odd).is_none());
        let mut huge = ok.to_vec();
        huge[9] = 400;
        assert!(edge_tail(&words(&huge)).is_none());
        let mut flag = ok.to_vec();
        flag[4] = 7;
        assert!(edge_tail(&words(&flag)).is_none());
        let mut negative = ok.to_vec();
        negative[10] = -2;
        assert!(edge_tail(&words(&negative)).is_none());
        assert!(edge_tail(&[]).is_none());
    }
}
