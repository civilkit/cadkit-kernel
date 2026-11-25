// SPDX-License-Identifier: MIT
//! The names by which feature inputs refer to B-rep entities.
//!
//! A face, edge or body input of a feature (`5662F619`, `9716F783`) refers
//! to a recipe object (`7ACC2A03`) that names the entity the way the bodies
//! in the file name theirs: each face and body of the ASM blobs carries
//! `generic_tag_attrib_def` names (see [`ck_f3d_names`]), a tag chosen by
//! the operation that made it and the operations that made and changed it.
//! The recipe lists entities by such names. Layout after the root part
//! (both flags 0) *(verified on the corpus and the reference models)*:
//!
//! ```text
//! u32 1 | u32 3 | u32 n
//! n × entity: u32 m, m × name
//!   name: str8 tag | u32 kind | u32 c | c × i32 op | u32 word
//!   (word: 0, or flags in some sheet-metal flanges' edges; not read)
//! u32 n2 | n2 × entity (mostly none)
//! str8 "<type>_recipe_data" | ... (more data, not read)
//! ```
//!
//! The second entity list (mitcad#96, in some edge recipes: one entity of
//! one name with an empty tag and one operation, e.g. `[340]` beside faces
//! of `[301]`) is read over and left out in edge recipes, where the first
//! list names the faces as usual, and in face and bounded-face recipes
//! (the first list names the face and the faces round it); other recipes
//! with one are not decoded.
//!
//! Types and entities *(meaning inferred from the reference models, where
//! the add-in dumps name the same entities)*:
//! - `edge_recipe_data`: the two faces along the edge, then the faces at its
//!   ends; where the first entity's names have a negative tag (`"-1029"`),
//!   that entity is the edge itself and the faces follow it (mitcad#96,
//!   [`ck_f3d_names::NamedState::find_edge`]);
//! - `face_recipe_data`: the face;
//! - `bounded_face_recipe_data`: the face, then the faces around it;
//! - `body_recipe_data`: the body (tags of type 3 in the blobs);
//! - `vertex_recipe_data`: the faces at the vertex.

use super::ir::EntityName;
use super::stream::{Segment, i32_at, str16_at, u32_at};

// The edge recipe's tail is read by the names' crate (mitcad#776).
pub use ck_f3d_names::{EdgeTail, edge_tail};

/// Recipe objects: they name entities.
pub const RECIPE: &str = "7ACC2A03-0261-4879-A14A-A93D661A5BDC";

/// A decoded recipe.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Recipe {
    /// `edge`, `face`, `bounded_face`, `body`, `vertex` (the type without
    /// `_recipe_data`).
    pub kind: String,
    /// The named entities, each by its names.
    pub entities: Vec<Vec<EntityName>>,
}

/// Recipe framing: the bytes after the type string are read by
/// [`edge_tail`] (the faces round an edge's ends); the secondary list has no
/// verified interpretation and must not decide a match.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecipeDetails {
    pub recipe: Recipe,
    pub secondary_entities: Vec<Vec<EntityName>>,
    /// Byte offset in the object's data, immediately after the type string.
    pub tail_offset: usize,
    pub tail: Vec<u8>,
}

/// At most this many entities, names per entity and operations per name.
const MAX_ITEMS: u32 = 100_000;

fn str8_at(d: &[u8], p: usize) -> Option<(String, usize)> {
    let n = u32_at(d, p)? as usize;
    let b = d.get(p + 4..p.checked_add(4 + n)?)?;
    Some((b.iter().map(|&c| c as char).collect(), p + 4 + n))
}

fn name_at(d: &[u8], p: usize) -> Option<(EntityName, usize)> {
    let (tag, p) = str8_at(d, p)?;
    let kind = u32_at(d, p)?;
    let c = u32_at(d, p + 4)?;
    if c > MAX_ITEMS {
        return None;
    }
    let mut q = p + 8;
    let mut ops = Vec::with_capacity(c as usize);
    for _ in 0..c {
        ops.push(i64::from(i32_at(d, q)?));
        q += 4;
    }
    // A word after the operations: 0 in most names; flags such as
    // `0x04000102` in the edge recipes of some sheet-metal flanges
    // (mitcad#211), as the blobs' own names have a word there that is read
    // over (`ck_f3d_names::attribute_names`).
    u32_at(d, q)?;
    let name = EntityName {
        tag,
        kind: i64::from(kind),
        ops,
    };
    Some((name, q + 4))
}

/// Decodes a recipe from an object's data (header and root part first).
pub fn parse(seg: &Segment, d: &[u8]) -> Option<Recipe> {
    parse_after(d, seg.root_part(d)?.end)
}

/// Decodes the same supported recipes as [`parse`], retaining opaque data.
pub fn parse_details(seg: &Segment, d: &[u8]) -> Option<RecipeDetails> {
    details_after(d, seg.root_part(d)?.end)
}

/// Decodes a recipe from object data whose root part ends at `p`.
fn parse_after(d: &[u8], p: usize) -> Option<Recipe> {
    framing_after(d, p).map(|(recipe, _, _)| recipe)
}

fn details_after(d: &[u8], p: usize) -> Option<RecipeDetails> {
    let (recipe, secondary_entities, tail_offset) = framing_after(d, p)?;
    Some(RecipeDetails {
        recipe,
        secondary_entities,
        tail_offset,
        tail: d[tail_offset..].to_vec(),
    })
}

fn framing_after(d: &[u8], mut p: usize) -> Option<(Recipe, Vec<Vec<EntityName>>, usize)> {
    if u32_at(d, p)? != 1 || u32_at(d, p + 4)? != 3 {
        return None;
    }
    let n = u32_at(d, p + 8)?;
    if n > MAX_ITEMS {
        return None;
    }
    p += 12;
    let (entities, p) = entities_at(d, p, n)?;
    // The second list (none in most recipes), not used.
    let n2 = u32_at(d, p)?;
    if n2 > MAX_ITEMS {
        return None;
    }
    let (secondary_entities, p) = entities_at(d, p + 4, n2)?;
    let (kind, tail_offset) = str8_at(d, p)?;
    let kind = kind.strip_suffix("_recipe_data")?.to_owned();
    // Only edge and face recipes are read with one (what it means
    // elsewhere is not known: a combine's body recipe with one, read the
    // same way, named its tool as the target in one design). A face's
    // names are in the first list, as an edge's faces are.
    if n2 > 0 && !matches!(kind.as_str(), "edge" | "face" | "bounded_face") {
        return None;
    }
    Some((Recipe { kind, entities }, secondary_entities, tail_offset))
}

/// `n` entities from `p` (each `u32 m | m names`), and where they end.
fn entities_at(d: &[u8], mut p: usize, n: u32) -> Option<(Vec<Vec<EntityName>>, usize)> {
    let mut entities = Vec::new();
    for _ in 0..n {
        let m = u32_at(d, p)?;
        if m > MAX_ITEMS {
            return None;
        }
        p += 4;
        let mut names = Vec::new();
        for _ in 0..m {
            let (name, q) = name_at(d, p)?;
            names.push(name);
            p = q;
        }
        entities.push(names);
    }
    Some((entities, p))
}

/// [`of_input`] with the recipe's framing (the bytes after the type string).
pub fn details_of_input(seg: &Segment, input: u64) -> Option<RecipeDetails> {
    let r = seg
        .ref_ids(seg.data_of(input))
        .into_iter()
        .find(|&r| seg.guid_of(r) == Some(RECIPE))?;
    parse_details(seg, seg.data_of(r))
}

/// The recipe of a feature input (a reference object that refers to a
/// recipe object), if it has one that decodes.
pub fn of_input(seg: &Segment, input: u64) -> Option<Recipe> {
    let r = seg
        .ref_ids(seg.data_of(input))
        .into_iter()
        .find(|&r| seg.guid_of(r) == Some(RECIPE))?;
    parse(seg, seg.data_of(r))
}

/// Whether `s` is a GUID as the streams write them (`8-4-4-4-12` hex
/// digits).
fn is_guid(s: &str) -> bool {
    s.len() == 36
        && s.char_indices().all(|(i, c)| match i {
            8 | 13 | 18 | 23 => c == '-',
            _ => c.is_ascii_hexdigit(),
        })
}

/// The GUID of the document that owns the entity a feature input refers
/// to (lower case): the first of the two GUIDs of the reference object
/// *(read from the proxies of faces, edges and bodies: in a design that
/// refers to other documents it is the GUID of the referenced file's
/// `Manifest.dat` where the entity is in a component of that document
/// (mitcad#196), the same for all its entities; for the design's own
/// entities another one)*. None when the object has no such text.
pub fn owner_guid(seg: &Segment, input: u64) -> Option<String> {
    let d = seg.data_of(input);
    let start = seg.root_part(d)?.end;
    (start..d.len().saturating_sub(4)).find_map(|p| {
        if u32_at(d, p)? != 36 {
            return None;
        }
        let (s, _) = str16_at(d, p)?;
        is_guid(&s).then(|| s.to_lowercase())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn name(tag: &str, ops: &[i32]) -> Vec<u8> {
        let mut b = Vec::new();
        b.extend((tag.len() as u32).to_le_bytes());
        b.extend(tag.as_bytes());
        b.extend(0u32.to_le_bytes());
        b.extend((ops.len() as u32).to_le_bytes());
        for o in ops {
            b.extend(o.to_le_bytes());
        }
        b.extend(0u32.to_le_bytes());
        b
    }

    /// A recipe object's data: header, empty root part, entities, type.
    pub(crate) fn recipe_data(id: u64, kind: &str, entities: &[&[(&str, &[i32])]]) -> Vec<u8> {
        let mut d = Vec::new();
        d.extend(3u32.to_le_bytes());
        d.extend(b"300");
        d.extend(id.to_le_bytes());
        d.extend(0u32.to_le_bytes());
        d.extend([0, 0]);
        d.extend(1u32.to_le_bytes());
        d.extend(3u32.to_le_bytes());
        d.extend((entities.len() as u32).to_le_bytes());
        for e in entities {
            d.extend((e.len() as u32).to_le_bytes());
            for (tag, ops) in e.iter() {
                d.extend(name(tag, ops));
            }
        }
        d.extend(0u32.to_le_bytes());
        let t = format!("{kind}_recipe_data");
        d.extend((t.len() as u32).to_le_bytes());
        d.extend(t.as_bytes());
        d.extend([0xff; 8]);
        d
    }

    #[test]
    fn parses_an_edge_recipe() {
        let d = recipe_data(
            7,
            "edge",
            &[
                &[("3", &[301])],
                &[("4", &[301]), ("2", &[303, -308])],
                &[("1", &[301])],
                &[("2", &[301])],
            ],
        );
        let r = parse_after(&d, 21).unwrap();
        assert_eq!(r.kind, "edge");
        assert_eq!(r.entities.len(), 4);
        assert_eq!(r.entities[1].len(), 2);
        assert_eq!(r.entities[1][1].tag, "2");
        assert_eq!(r.entities[1][1].ops, vec![303, -308]);
        assert_eq!(r.entities[3][0].kind, 0);
        let details = details_after(&d, 21).unwrap();
        assert_eq!(details.recipe, r);
        assert!(details.secondary_entities.is_empty());
        assert_eq!(details.tail, vec![0xff; 8]);
        assert_eq!(&d[details.tail_offset..], details.tail);
    }

    #[test]
    fn reads_over_a_second_entity_list_of_a_face() {
        // A face's names are in the first list, as an edge's faces are.
        for kind in ["face", "bounded_face"] {
            let d = recipe_data(7, kind, &[&[("3", &[301])], &[("4", &[301])]]);
            let at = d.len() - 8 - format!("{kind}_recipe_data").len() - 4 - 4;
            let mut e = d[..at].to_vec();
            e.extend(1u32.to_le_bytes());
            e.extend(1u32.to_le_bytes());
            e.extend(name("1", &[340]));
            e.extend(&d[at + 4..]);
            let r = parse_after(&e, 21).unwrap();
            assert_eq!(r.kind, kind);
            assert_eq!(r.entities.len(), 2);
        }
    }
    #[test]
    fn reads_over_a_second_entity_list() {
        // `u32 1 | u32 1 | name` in place of `u32 0` after the entities.
        let d = recipe_data(7, "edge", &[&[("3", &[301])], &[("4", &[301])]]);
        let at = d.len() - 8 - "edge_recipe_data".len() - 4 - 4;
        assert_eq!(&d[at..at + 4], &0u32.to_le_bytes());
        let mut e = d[..at].to_vec();
        e.extend(1u32.to_le_bytes());
        e.extend(1u32.to_le_bytes());
        e.extend(name("", &[340]));
        e.extend(&d[at + 4..]);
        let r = parse_after(&e, 21).unwrap();
        assert_eq!(r.kind, "edge");
        assert_eq!(r.entities.len(), 2);
        assert_eq!(r.entities[1][0].tag, "4");
        let details = details_after(&e, 21).unwrap();
        assert_eq!(details.recipe, r);
        assert_eq!(
            details.secondary_entities,
            vec![vec![EntityName {
                tag: String::new(),
                kind: 0,
                ops: vec![340],
            }]]
        );
        assert_eq!(details.tail, vec![0xff; 8]);
        // Other recipes with a second list are not read.
        let f = recipe_data(7, "body", &[&[("3", &[301])]]);
        let bt = f.len() - 8 - "body_recipe_data".len() - 4 - 4;
        let mut g = f[..bt].to_vec();
        g.extend(1u32.to_le_bytes());
        g.extend(1u32.to_le_bytes());
        g.extend(name("", &[340]));
        g.extend(&f[bt + 4..]);
        assert_eq!(parse_after(&g, 21), None);
        // A second list that does not end before the type is refused.
        let mut e = d[..at].to_vec();
        e.extend(1u32.to_le_bytes());
        e.extend(&d[at + 4..]);
        assert_eq!(parse_after(&e, 21), None);
    }

    #[test]
    fn refuses_other_layouts() {
        let mut d = recipe_data(7, "face", &[&[("1", &[301])]]);
        // A name whose closing word is not 0 (flags, as in some
        // sheet-metal flanges' edges, mitcad#211) reads as the same name.
        let at = d.len() - 8 - 16 - 4 - 4 - 4;
        d[at] = 5;
        let r = parse_after(&d, 21).unwrap();
        assert_eq!(r.entities[0][0].ops, vec![301]);
        let d = recipe_data(7, "face", &[]);
        assert_eq!(parse_after(&d, 21).unwrap().entities.len(), 0);
        let mut d = recipe_data(7, "face", &[]);
        let n = d.len();
        d[n - 8 - 1] = b'X'; // "..._recipe_datX"
        assert_eq!(parse_after(&d, 21), None);
    }
}
