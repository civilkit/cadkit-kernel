// SPDX-License-Identifier: MIT
//! What an `.f3z` package says about its designs (mitcad#229):
//! `Manifest.json` names the root design's file (`{"root": "<file>.f3d"}`)
//! and `DesignDescription.json` lists every design of the package with the
//! URN the designs that refer to it name it by (`about`,
//! `urn:…:fs.file:vf.<id>?version=N`), its lineage, its `.f3d` inside the
//! package (`relativePath`) and its document's name (`friendlyName`,
//! `displayName`). Both are optional: a package without them is read by
//! its designs alone.

use serde_json::Value;

use ck_f3d_file::container::F3dFile;

/// A design of the package as `DesignDescription.json` lists it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DescribedDesign {
    /// The URN designs refer to it by (with its version).
    pub urn: String,
    pub lineage: Option<String>,
    /// Its `.f3d` entry in the package.
    pub path: String,
    /// Its document's name.
    pub name: Option<String>,
}

/// What `Manifest.json` and `DesignDescription.json` say.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PackageDescription {
    /// The root design's `.f3d` entry.
    pub root: Option<String>,
    pub designs: Vec<DescribedDesign>,
}

impl PackageDescription {
    /// Reads the package's two files (missing or unreadable ones give
    /// nothing).
    pub fn read(package: &F3dFile) -> Self {
        let text = |name: &str| {
            package
                .read(name)
                .ok()
                .and_then(|d| String::from_utf8(d).ok())
        };
        Self::parse(
            text("Manifest.json").as_deref(),
            text("DesignDescription.json").as_deref(),
        )
    }

    /// Parses the texts of `Manifest.json` and `DesignDescription.json`.
    pub fn parse(manifest: Option<&str>, description: Option<&str>) -> Self {
        let json = |t: Option<&str>| t.and_then(|t| serde_json::from_str::<Value>(t).ok());
        let root = json(manifest)
            .and_then(|m| m.get("root").and_then(Value::as_str).map(str::to_owned))
            .filter(|r| !r.is_empty());
        let mut designs = Vec::new();
        if let Some(d) = json(description) {
            let graphs = d
                .pointer("/designDescription/designGraphs")
                .and_then(Value::as_array);
            for object in graphs
                .into_iter()
                .flatten()
                .filter_map(|g| g.get("designObjects").and_then(Value::as_array))
                .flatten()
            {
                let text = |key: &str| {
                    object
                        .get(key)
                        .and_then(Value::as_str)
                        .filter(|s| !s.is_empty())
                        .map(str::to_owned)
                };
                let (Some(urn), Some(path)) = (text("about"), text("relativePath")) else {
                    continue;
                };
                designs.push(DescribedDesign {
                    urn,
                    lineage: text("lineage"),
                    path,
                    name: text("friendlyName").or_else(|| text("displayName")),
                });
            }
        }
        PackageDescription { root, designs }
    }

    /// Whether the package said nothing.
    pub fn is_empty(&self) -> bool {
        self.root.is_none() && self.designs.is_empty()
    }
}

/// Whether the entry `entry` of a package is the file `path` names
/// (`relativePath`, `root`): the same name, or the name in a folder.
pub fn is_entry(entry: &str, path: &str) -> bool {
    let path = path.trim_start_matches("./").replace('\\', "/");
    entry == path || entry.ends_with(&format!("/{path}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_root_and_the_designs_are_read() {
        let manifest = r#"{"root":"aa.f3d"}"#;
        let description = r#"{"designDescription":{"designGraphs":[{"designObjects":[
            {"id":1,"about":"urn:x:fs.file:vf.a?version=3","lineage":"urn:x:dm.lineage:a",
             "relativePath":"aa.f3d","friendlyName":"Assembly","displayName":"Assembly"},
            {"id":2,"about":"urn:x:fs.file:vf.b?version=1","relativePath":"bb.f3d",
             "displayName":"Bolt"},
            {"id":3,"about":"urn:x:fs.file:vf.c?version=1"}]}]}}"#;
        let p = PackageDescription::parse(Some(manifest), Some(description));
        assert_eq!(p.root.as_deref(), Some("aa.f3d"));
        assert_eq!(p.designs.len(), 2);
        assert_eq!(p.designs[0].name.as_deref(), Some("Assembly"));
        assert_eq!(p.designs[0].lineage.as_deref(), Some("urn:x:dm.lineage:a"));
        assert_eq!(p.designs[1].urn, "urn:x:fs.file:vf.b?version=1");
        assert_eq!(p.designs[1].name.as_deref(), Some("Bolt"));
        assert!(PackageDescription::parse(None, Some("not json")).is_empty());
        assert!(is_entry("bb.f3d", "bb.f3d"));
        assert!(is_entry("x/bb.f3d", "./bb.f3d"));
        assert!(!is_entry("abb.f3d", "bb.f3d"));
    }
}
