// SPDX-License-Identifier: MIT
//! An item's state in the file, from the result number and the three state
//! bytes of its tail (mitcad#180). The stream decoder reads them
//! (`mitcad_f3d::design::decode`), the IR's [`crate::TimelineItem`] from
//! its `_f3d` keys.

/// An item's state in the file, from its result number and the three state
/// bytes of its tail (mitcad#180).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ItemState {
    /// The file keeps the item's result, or the item has none to keep
    /// (sketches, construction geometry, occurrences; modelling items
    /// after the timeline marker, never computed, also have result -1 and
    /// no state byte set).
    Normal,
    /// Suppressed in the file: result -1 and the first or the second byte
    /// set (`010000`, `000100`, on sketches, extrusions, fillets and others
    /// alike, alone and in runs). The two bytes tell how it came to be
    /// suppressed ([`Suppression`]).
    Suppressed,
    /// Result -1 and the third byte set (`000001`): an item whose change
    /// is all in other components, in its parts there (mitcad#169); every
    /// such item of the corpora has parts with results.
    InParts,
}

/// An item's state from its result number and the three state bytes of
/// its tail. In the corpora the bytes are set only on items with result
/// -1; such items also have `01 01 00` where the others have `01 00 00`
/// after the health reference, and -1 where the others have their start
/// state's number (the result number before theirs). An item that failed
/// in the file (one checked in the original application) has the bytes
/// and numbers of a healthy item: the history tells it (mitcad#156).
pub fn item_state(result_no: i32, flags: [u8; 3]) -> ItemState {
    if result_no != -1 || flags == [0, 0, 0] {
        ItemState::Normal
    } else if flags[0] != 0 || flags[1] != 0 {
        ItemState::Suppressed
    } else {
        ItemState::InParts
    }
}

/// How an item came to be suppressed in the file. The state
/// bytes of the controlled variants: an item the user suppressed (by the
/// design API, the timeline menu, a configuration's row or as a member of a
/// suppressed group, all the same bytes) has `000100`; an item suppressed
/// only because an item it depends on is (the fillet of a suppressed hole's
/// edge, the extrusion of a suppressed sketch) has `010000`. In the
/// variants the two differ in these bytes alone. In the corpora 24 of the
/// 27 designs with suppressed items begin with a `000100` item (an item
/// suppressed through another cannot come first); the three that do not
/// are one design's versions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Suppression {
    /// Suppressed itself (`000100`).
    Direct,
    /// Suppressed through an item it depends on (`010000`).
    Indirect,
}

/// How an item is suppressed, from its result number and state bytes
/// (`None` for an item that is not suppressed, [`item_state`]).
pub fn item_suppression(result_no: i32, flags: [u8; 3]) -> Option<Suppression> {
    if item_state(result_no, flags) != ItemState::Suppressed {
        None
    } else if flags[1] != 0 {
        Some(Suppression::Direct)
    } else {
        Some(Suppression::Indirect)
    }
}

#[cfg(test)]
mod tests {
    use serde_json::{Value, json};

    use super::*;
    use crate::TimelineItem;

    #[test]
    fn item_states_from_the_tail() {
        // mitcad#180: the state bytes count only with result -1.
        assert_eq!(item_state(-1, [1, 0, 0]), ItemState::Suppressed);
        assert_eq!(item_state(-1, [0, 1, 0]), ItemState::Suppressed);
        assert_eq!(item_state(-1, [0, 0, 1]), ItemState::InParts);
        assert_eq!(item_state(-1, [0, 0, 0]), ItemState::Normal);
        assert_eq!(item_state(7, [0, 1, 0]), ItemState::Normal);
        // The IR: isSuppressed, else the bytes in `_f3d`.
        let item = |v: Value| -> TimelineItem { serde_json::from_value(v).unwrap() };
        let state = |v: Value| item(v).file_state();
        assert_eq!(
            state(json!({"index": 0, "isSuppressed": true})),
            ItemState::Suppressed
        );
        assert_eq!(
            state(json!({"index": 0, "_f3d": {"result_no": -1, "flags": "010000"}})),
            ItemState::Suppressed
        );
        assert_eq!(
            state(json!({"index": 0, "_f3d": {"result_no": -1, "flags": "000001"}})),
            ItemState::InParts
        );
        assert_eq!(
            state(json!({"index": 0, "_f3d": {"result_no": 4, "flags": "000100"}})),
            ItemState::Normal
        );
        assert_eq!(state(json!({"index": 0})), ItemState::Normal);
    }

    #[test]
    fn suppression_tells_direct_from_indirect() {
        // `000100` suppressed by the user, `010000` through an item
        // it depends on; only with result -1.
        assert_eq!(item_suppression(-1, [0, 1, 0]), Some(Suppression::Direct));
        assert_eq!(item_suppression(-1, [1, 0, 0]), Some(Suppression::Indirect));
        assert_eq!(item_suppression(-1, [0, 0, 1]), None);
        assert_eq!(item_suppression(-1, [0, 0, 0]), None);
        assert_eq!(item_suppression(5, [0, 1, 0]), None);
        let item = |v: Value| -> TimelineItem { serde_json::from_value(v).unwrap() };
        let how = |v: Value| item(v).suppression();
        assert_eq!(
            how(json!({"index": 0, "isSuppressed": true,
                       "_f3d": {"result_no": -1, "flags": "010000"}})),
            Some(Suppression::Indirect)
        );
        assert_eq!(
            how(json!({"index": 0, "_f3d": {"result_no": -1, "flags": "000100"}})),
            Some(Suppression::Direct)
        );
        // An external dump's `isSuppressed` has no bytes: direct.
        assert_eq!(
            how(json!({"index": 0, "isSuppressed": true})),
            Some(Suppression::Direct)
        );
        assert_eq!(how(json!({"index": 0})), None);
    }
}
