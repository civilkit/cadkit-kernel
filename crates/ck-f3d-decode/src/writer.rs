// SPDX-License-Identifier: MIT
//! A writer of synthetic design streams (`MetaStream.dat` and
//! `BulkStream.dat`) built object by object from the layouts of the
//! timeline format study: the decoder's tests and the test files of other
//! crates (a design that refers to another document, mitcad#196) are made
//! with it. Not a writer of real designs.

use super::stream::Segment;

fn s8(s: &str) -> Vec<u8> {
    let mut v = (s.len() as u32).to_le_bytes().to_vec();
    v.extend(s.as_bytes());
    v
}

fn s16(s: &str) -> Vec<u8> {
    let units: Vec<u16> = s.encode_utf16().collect();
    let mut v = (units.len() as u32).to_le_bytes().to_vec();
    v.extend(units.iter().flat_map(|u| u.to_le_bytes()));
    v
}

/// Object id, class index, body, sub-chunk body.
pub type WriterObject = (u64, usize, Vec<u8>, Option<Vec<u8>>);

/// An external document of the MetaStream tail: its key, the two GUIDs
/// before the URN, the URN and the GUID after it.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct WriterExternal {
    pub key: u64,
    pub guids: [String; 2],
    pub urn: String,
    pub document_guid: String,
}

/// A synthetic design: class table, objects and external documents.
#[derive(Default)]
pub struct DesignWriter {
    /// GUID, parent GUID, class version.
    pub classes: Vec<(String, String, u32)>,
    pub objects: Vec<WriterObject>,
    /// External documents as (key, URN), their GUIDs `a`, `b` and `c`.
    pub external: Vec<(u64, String)>,
    /// External documents with GUIDs of their own (after `external`).
    pub externals: Vec<WriterExternal>,
    /// The MetaStream's header form B (`1234`, a later writer).
    pub form_b: bool,
}

impl DesignWriter {
    /// Adds a class; returns its index.
    pub fn class(&mut self, guid: &str, parent: &str, version: u32) -> usize {
        self.classes.push((guid.into(), parent.into(), version));
        self.classes.len() - 1
    }

    /// The index of a class added before.
    pub fn class_index(&self, guid: &str) -> usize {
        self.classes
            .iter()
            .position(|c| c.0 == guid)
            .unwrap_or_else(|| panic!("class {guid} not added"))
    }

    /// Adds an object of a class (added before) with its body after the
    /// header.
    pub fn obj(&mut self, id: u64, guid: &str, body: Vec<u8>) {
        let c = self.class_index(guid);
        self.objects.push((id, c, body, None));
    }

    /// Adds an object with a sub-chunk.
    pub fn obj_sub(&mut self, id: u64, guid: &str, body: Vec<u8>, sub: Vec<u8>) {
        let c = self.class_index(guid);
        self.objects.push((id, c, body, Some(sub)));
    }

    /// The MetaStream and the BulkStream.
    pub fn streams(&self) -> (Vec<u8>, Vec<u8>) {
        let u32b = |v: u32| v.to_le_bytes().to_vec();
        let mut bulk = Vec::new();
        let mut index = Vec::new();
        let mut subs = Vec::new();
        for (id, class, body, sub) in &self.objects {
            index.push((*id, bulk.len() as u64));
            bulk.extend(s8(&(256 + class).to_string()));
            bulk.extend(id.to_le_bytes());
            bulk.extend(s8(""));
            bulk.extend(body);
            if let Some(sub) = sub {
                subs.push((*id, bulk.len() as u64));
                bulk.extend(s8(&(256 + class).to_string()));
                bulk.extend(id.to_le_bytes());
                bulk.extend(sub);
            }
        }
        let mut m = s8("Design");
        m.extend(u32b(0));
        m.extend(s16("00000000-0000-0000-0000-000000000000"));
        if self.form_b {
            m.extend(u32b(1234));
            m.extend(u32b(8));
            m.extend(u32b(36));
            m.extend(u32b((2704 << 18) | (1 << 6)));
        } else {
            m.extend(u32b(0x0800_0000 | 20981));
            m.extend(u32b(5));
        }
        m.extend(s8("Design"));
        m.extend(s8(if self.form_b { "Design" } else { "" }));
        m.extend(u32b(1));
        m.extend(u32b(0));
        m.extend(u32b(self.classes.len() as u32));
        for (i, (guid, parent, version)) in self.classes.iter().enumerate() {
            m.extend(s8(guid));
            m.extend(s8(parent));
            m.extend(u32b(*version));
            m.extend(s8("Design"));
            let ids: Vec<u64> = self
                .objects
                .iter()
                .filter(|o| o.1 == i)
                .map(|o| o.0)
                .collect();
            m.extend(u32b(ids.len() as u32));
            for id in ids {
                m.extend(id.to_le_bytes());
            }
        }
        m.extend(u32b(0)); // named roots
        for list in [&index, &subs] {
            m.extend(u32b(list.len() as u32));
            for (id, off) in list.iter() {
                m.extend(id.to_le_bytes());
                m.extend(off.to_le_bytes());
            }
        }
        let next = self.objects.iter().map(|o| o.0).max().unwrap_or(0) + 1;
        m.extend(next.to_le_bytes());
        let externals: Vec<WriterExternal> = self
            .external
            .iter()
            .map(|(key, urn)| WriterExternal {
                key: *key,
                guids: ["a".into(), "b".into()],
                urn: urn.clone(),
                document_guid: "c".into(),
            })
            .chain(self.externals.iter().cloned())
            .collect();
        m.extend(u32b(externals.len() as u32));
        for x in &externals {
            m.extend(x.key.to_le_bytes());
            for s in [&x.guids[0], &x.guids[1], &x.urn, &x.document_guid] {
                m.extend(s16(s));
            }
            m.extend(u32b(1));
        }
        if self.form_b {
            m.extend(u32b(2));
            m.extend(s8("Application"));
            m.extend(u32b(2));
            m.extend(s8("Server"));
            m.extend(u32b(6));
        }
        (m, bulk)
    }

    /// The streams parsed as a segment (the writer writes what it reads).
    pub fn segment(&self) -> Segment {
        let (m, b) = self.streams();
        Segment::parse(&m, b).expect("the written streams parse")
    }
}
