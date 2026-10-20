//! Tiny XML attribute scanner for the `material.xml` / `style.xml` entries of a VFF file.
//!
//! The ZIP is attacker controlled; this is not an XML parser. It finds start tags by local name, decodes the five
//! predefined entities and numeric character references in attribute values, and never expands anything else (so
//! entity-expansion attacks do not exist) and never allocates more than the input size.
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.

#[derive(Debug, Clone, Default)]
pub struct Tag {
    /// Local name (namespace prefix removed).
    pub name: String,
    pub attrs: Vec<(String, String)>,
    /// Text content up to the next `<` (only meaningful for simple elements such as `<variant>123</variant>`).
    pub text: String,
}

impl Tag {
    pub fn attr(&self, name: &str) -> Option<&str> {
        self.attrs.iter().find(|(k, _)| k == name).map(|(_, v)| v.as_str())
    }
}

fn decode(s: &str) -> String {
    if !s.contains('&') {
        return s.to_string();
    }
    let mut out = String::with_capacity(s.len());
    let mut rest = s;
    while let Some(i) = rest.find('&') {
        out.push_str(&rest[..i]);
        rest = &rest[i..];
        let Some(j) = rest.find(';').filter(|&j| j <= 10) else {
            out.push('&');
            rest = &rest[1..];
            continue;
        };
        let ent = &rest[1..j];
        let ch = match ent {
            "amp" => Some('&'),
            "lt" => Some('<'),
            "gt" => Some('>'),
            "quot" => Some('"'),
            "apos" => Some('\''),
            _ if ent.starts_with("#x") || ent.starts_with("#X") => u32::from_str_radix(&ent[2..], 16).ok().and_then(char::from_u32),
            _ if ent.starts_with('#') => ent[1..].parse::<u32>().ok().and_then(char::from_u32),
            _ => None,
        };
        match ch {
            Some(c) => {
                out.push(c);
                rest = &rest[j + 1..];
            }
            None => {
                out.push('&');
                rest = &rest[1..];
            }
        }
    }
    out.push_str(rest);
    out
}

/// All start tags (and empty-element tags) in document order. Comments, processing instructions, DOCTYPE and CDATA are
/// skipped. At most `max_tags` tags are returned.
pub fn tags(xml: &str, max_tags: usize) -> Vec<Tag> {
    let b = xml.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() && out.len() < max_tags {
        let Some(p) = xml[i..].find('<') else { break };
        i += p;
        let rest = &xml[i..];
        if rest.starts_with("<!--") {
            i += rest.find("-->").map_or(rest.len(), |e| e + 3);
            continue;
        }
        if rest.starts_with("<![CDATA[") {
            i += rest.find("]]>").map_or(rest.len(), |e| e + 3);
            continue;
        }
        if rest.starts_with("<?") || rest.starts_with("<!") || rest.starts_with("</") {
            i += rest.find('>').map_or(rest.len(), |e| e + 1);
            continue;
        }
        // name
        let mut j = 1;
        while j < rest.len() && !rest.as_bytes()[j].is_ascii_whitespace() && rest.as_bytes()[j] != b'>' && rest.as_bytes()[j] != b'/' {
            j += 1;
        }
        let full = &rest[1..j];
        let name = full.rsplit(':').next().unwrap_or(full).to_string();
        let mut attrs = Vec::new();
        // attributes
        loop {
            while j < rest.len() && rest.as_bytes()[j].is_ascii_whitespace() {
                j += 1;
            }
            if j >= rest.len() || rest.as_bytes()[j] == b'>' || rest.as_bytes()[j] == b'/' {
                break;
            }
            let ks = j;
            while j < rest.len() && !matches!(rest.as_bytes()[j], b'=' | b'>' | b'/') && !rest.as_bytes()[j].is_ascii_whitespace() {
                j += 1;
            }
            let key = &rest[ks..j];
            while j < rest.len() && rest.as_bytes()[j].is_ascii_whitespace() {
                j += 1;
            }
            if j < rest.len() && rest.as_bytes()[j] == b'=' {
                j += 1;
                while j < rest.len() && rest.as_bytes()[j].is_ascii_whitespace() {
                    j += 1;
                }
                if j < rest.len() && (rest.as_bytes()[j] == b'"' || rest.as_bytes()[j] == b'\'') {
                    let q = rest.as_bytes()[j];
                    j += 1;
                    let vs = j;
                    while j < rest.len() && rest.as_bytes()[j] != q {
                        j += 1;
                    }
                    let val = &rest[vs..j.min(rest.len())];
                    j = (j + 1).min(rest.len());
                    let k = key.rsplit(':').next().unwrap_or(key);
                    attrs.push((k.to_string(), decode(val)));
                } else {
                    // unquoted value: skip it
                    while j < rest.len() && !rest.as_bytes()[j].is_ascii_whitespace() && rest.as_bytes()[j] != b'>' {
                        j += 1;
                    }
                }
            } else if key.is_empty() {
                j += 1;
            }
        }
        let end = rest[j..].find('>').map_or(rest.len(), |e| j + e + 1);
        let self_closing = rest[..end].ends_with("/>");
        let mut text = String::new();
        if !self_closing {
            let after = &xml[i + end..];
            let t = after.find('<').unwrap_or(after.len());
            text = decode(after[..t].trim());
        }
        out.push(Tag { name, attrs, text });
        i += end.max(1);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn attrs_and_entities() {
        let t = tags(r#"<?xml version="1.0"?><mat:material xmlns:mat="u" name="A &amp; B" colorRed='12' useTrans="1"/><x:item id="2002"><t:variant>-5</t:variant></x:item>"#, 100);
        assert_eq!(t[0].name, "material");
        assert_eq!(t[0].attr("name"), Some("A & B"));
        assert_eq!(t[0].attr("colorRed"), Some("12"));
        assert_eq!(t[1].name, "item");
        assert_eq!(t[2].name, "variant");
        assert_eq!(t[2].text, "-5");
    }

    #[test]
    fn garbage_does_not_panic() {
        for s in ["<", "<a", "<a b", "<a b=", "<a b=\"", "<!--", "&&&;<a x=\"&#xZZ;\">", "<<<<>>>"] {
            let _ = tags(s, 10);
        }
    }
}
