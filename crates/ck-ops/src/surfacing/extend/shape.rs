//! Extension shapes: how the new boundary piece continues the surface.
//!
//! The UI offers `same` (natural continuation), `linear`, `soft`,
//! `reflective`. For the analytic surfaces this pass extends (planes,
//! cylinders, extrusions) the natural continuation IS the linear one, so
//! all four shapes produce the same geometry here — pinned by the oracle
//! probe (square sheet, edge extended 5: the result is the exact
//! rectangle). Splines would differ (reflective mirrors the second
//! derivative); that path is not reconstructed yet.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Shape {
    Same,
    Linear,
    Soft,
    Reflective,
}

impl Shape {
    pub fn parse(s: &str) -> Option<Shape> {
        match s {
            "same" => Some(Shape::Same),
            "linear" => Some(Shape::Linear),
            "soft" => Some(Shape::Soft),
            "reflective" => Some(Shape::Reflective),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_shapes() {
        assert_eq!(Shape::parse("same"), Some(Shape::Same));
        assert_eq!(Shape::parse("linear"), Some(Shape::Linear));
        assert_eq!(Shape::parse("soft"), Some(Shape::Soft));
        assert_eq!(Shape::parse("reflective"), Some(Shape::Reflective));
        assert_eq!(Shape::parse("bogus"), None);
    }
}
