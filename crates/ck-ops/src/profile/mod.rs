//! Design-rule engine: named profiles of rules (parameters + severity) run
//! over a regenerated body; every issue names the failing faces / edges by
//! persistent name. Wire shapes: docs/spec-agent-interface.md ("Profiles").
//!
//! * `{type:"profiles"}` lists the built-in profiles and the rule catalogue
//!   (parameter names, types, defaults, units) so a GUI can build a form.
//! * `{type:"checkProfile", profile: "<id>" | {inline}, opts?}` runs one.
//!
//! Rules use the exact kernel geometry where it exists (cylinder radii for
//! hole / pin diameters, plane normals for draft and overhang, edge and
//! face adjacency for sharp corners) and otherwise the display
//! triangulation: area-proportional surface samples cast inward / outward
//! rays at a BVH of the mesh. Sampled answers are estimates (see the doc).

mod ctx;
mod rules;

pub use ctx::Ctx;
use ck_core::linalg::Vec3;
use ck_json::{Json, Obj};
use ck_topo::body::Body;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Severity {
    Error,
    Warn,
    Info,
}

impl Severity {
    pub fn as_str(self) -> &'static str {
        match self {
            Severity::Error => "error",
            Severity::Warn => "warn",
            Severity::Info => "info",
        }
    }
    fn parse(s: &str) -> Option<Severity> {
        match s {
            "error" => Some(Severity::Error),
            "warn" | "warning" => Some(Severity::Warn),
            "info" => Some(Severity::Info),
            _ => None,
        }
    }
}

/// One finding. `faces` / `edges` are persistent names (`F2.C1`, `F2.C1|F2.S.e3#4`).
#[derive(Clone, Debug)]
pub struct Issue {
    pub rule: &'static str,
    pub severity: Severity,
    pub message: String,
    pub faces: Vec<String>,
    pub edges: Vec<String>,
    pub value: Option<f64>,
    pub limit: Option<f64>,
    pub unit: &'static str,
    /// A representative point on the part (viewport focus), when known.
    pub at: Option<Vec3>,
}

impl Issue {
    pub(crate) fn new(rule: &'static str, severity: Severity, message: String) -> Issue {
        Issue { rule, severity, message, faces: vec![], edges: vec![], value: None, limit: None, unit: "", at: None }
    }
    fn to_json(&self) -> Json {
        let names = |v: &[String]| Json::Arr(v.iter().map(|s| Json::from(s.as_str())).collect());
        let mut o = Obj::new()
            .put("rule", self.rule)
            .put("severity", self.severity.as_str())
            .put("message", self.message.as_str())
            .put("faces", names(&self.faces))
            .put("edges", names(&self.edges))
            .put("value", self.value.map_or(Json::Null, |v| Json::Num(round(v))))
            .put("limit", self.limit.map_or(Json::Null, |v| Json::Num(round(v))))
            .put("unit", self.unit);
        if let Some(p) = self.at {
            o = o.put("at", Json::Arr(vec![Json::Num(round(p.x)), Json::Num(round(p.y)), Json::Num(round(p.z))]));
        }
        o.build()
    }
}

fn round(x: f64) -> f64 {
    if x.is_finite() {
        (x * 1e6).round() / 1e6
    } else {
        0.0
    }
}

// ---------------------------------------------------------------- params

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PVal {
    Num(f64),
    Vec([f64; 3]),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PKind {
    Num,
    Vec3,
}

pub struct ParamDef {
    pub name: &'static str,
    pub kind: PKind,
    pub default: PVal,
    pub unit: &'static str,
    pub min: Option<f64>,
    pub max: Option<f64>,
    pub doc: &'static str,
}

/// Resolved parameter values of one rule instance.
#[derive(Clone, Debug)]
pub struct Params(Vec<(&'static str, PVal)>);

impl Params {
    pub fn num(&self, k: &str) -> f64 {
        match self.0.iter().find(|(n, _)| *n == k) {
            Some((_, PVal::Num(x))) => *x,
            _ => f64::NAN,
        }
    }
    /// A unit direction vector (parameters are validated non-zero).
    pub fn dir(&self, k: &str) -> Vec3 {
        match self.0.iter().find(|(n, _)| *n == k) {
            Some((_, PVal::Vec(v))) => Vec3::from_array(*v).normalize(),
            _ => Vec3::new(0.0, 0.0, 1.0),
        }
    }
}

fn pval_json(v: &PVal) -> Json {
    match v {
        PVal::Num(x) => Json::Num(*x),
        PVal::Vec(a) => Json::Arr(a.iter().map(|&x| Json::Num(x)).collect()),
    }
}

type RunFn = fn(&Ctx, &Params, Severity) -> Vec<Issue>;

pub struct RuleDef {
    pub id: &'static str,
    pub title: &'static str,
    pub doc: &'static str,
    pub severity: Severity,
    pub params: &'static [ParamDef],
    pub run: RunFn,
}

// ---------------------------------------------------------------- profiles

/// A rule instance inside a profile: overrides of the rule's defaults.
#[derive(Clone, Debug)]
pub struct RuleSpec {
    pub id: String,
    pub params: Vec<(String, PVal)>,
    pub severity: Option<Severity>,
}

#[derive(Clone, Debug)]
pub struct Profile {
    pub id: String,
    pub name: String,
    pub description: String,
    pub rules: Vec<RuleSpec>,
}

fn rs(id: &str, params: &[(&str, PVal)], severity: Option<Severity>) -> RuleSpec {
    RuleSpec { id: id.into(), params: params.iter().map(|(k, v)| (k.to_string(), *v)).collect(), severity }
}

pub fn builtin_profiles() -> Vec<Profile> {
    use PVal::{Num, Vec as V};
    use Severity::{Error, Info, Warn};
    let p = |id: &str, name: &str, description: &str, rules: Vec<RuleSpec>| Profile { id: id.into(), name: name.into(), description: description.into(), rules };
    vec![
        p(
            "fdm-print",
            "FDM 3D printing",
            "Filament printing: watertight single solid, wall thickness, overhangs against the build direction, bridges, small features, bed contact.",
            vec![
                rs("openShell", &[], Some(Error)),
                rs("bodyCount", &[], Some(Warn)),
                rs("minWall", &[("min", Num(1.2))], Some(Error)),
                rs("overhang", &[("angle", Num(45.0)), ("direction", V([0.0, 1.0, 0.0]))], Some(Warn)),
                rs("bridge", &[("maxSpan", Num(10.0)), ("direction", V([0.0, 1.0, 0.0]))], Some(Warn)),
                rs("minFeature", &[("minDiameter", Num(0.8)), ("minGap", Num(0.8))], Some(Warn)),
                rs("plateContact", &[("minArea", Num(20.0)), ("direction", V([0.0, 1.0, 0.0]))], Some(Warn)),
            ],
        ),
        p(
            "cnc-3axis",
            "CNC milling, 3 axis",
            "Single setup from +Z: internal corner radius against the cutter, wall thickness, faces the tool cannot reach.",
            vec![
                rs("openShell", &[], Some(Error)),
                rs("minInternalRadius", &[("minRadius", Num(1.0)), ("direction", V([0.0, 0.0, 1.0]))], Some(Warn)),
                rs("minWall", &[("min", Num(1.0))], Some(Warn)),
                rs("undercut", &[("direction", V([0.0, 0.0, 1.0]))], Some(Error)),
            ],
        ),
        p(
            "injection",
            "Injection moulding",
            "Two-part mould pulled along +Z: draft on every wall, uniform wall thickness.",
            vec![
                rs("openShell", &[], Some(Error)),
                rs("draft", &[("minAngle", Num(1.0)), ("direction", V([0.0, 0.0, 1.0]))], Some(Warn)),
                rs("uniformWall", &[("tolerance", Num(0.25))], Some(Warn)),
                rs("minWall", &[("min", Num(0.8))], Some(Info)),
            ],
        ),
    ]
}

pub(crate) fn rule_defs() -> &'static [RuleDef] {
    rules::RULES
}

fn find_rule(id: &str) -> Result<&'static RuleDef, String> {
    rule_defs().iter().find(|r| r.id == id).ok_or_else(|| {
        format!("unknown rule \"{id}\"; rules: {}", rule_defs().iter().map(|r| r.id).collect::<Vec<_>>().join(", "))
    })
}

/// Rule instance with defaults merged and values validated.
struct Resolved {
    def: &'static RuleDef,
    params: Params,
    severity: Severity,
}

fn resolve_rule(spec: &RuleSpec) -> Result<Resolved, String> {
    let def = find_rule(&spec.id)?;
    let mut vals: Vec<(&'static str, PVal)> = def.params.iter().map(|d| (d.name, d.default)).collect();
    for (k, v) in &spec.params {
        let Some(d) = def.params.iter().find(|d| d.name == k) else {
            return Err(format!("rule {}: unknown parameter \"{k}\"; parameters: {}", def.id, def.params.iter().map(|d| d.name).collect::<Vec<_>>().join(", ")));
        };
        match (d.kind, v) {
            (PKind::Num, PVal::Num(x)) => {
                if !x.is_finite() || d.min.is_some_and(|m| *x < m) || d.max.is_some_and(|m| *x > m) {
                    return Err(format!("rule {}: parameter {k} = {x} is out of range [{}, {}]", def.id, d.min.map_or("-inf".into(), |m| m.to_string()), d.max.map_or("inf".into(), |m| m.to_string())));
                }
            }
            (PKind::Vec3, PVal::Vec(a)) => {
                if a.iter().any(|x| !x.is_finite()) || Vec3::from_array(*a).len() < 1e-9 {
                    return Err(format!("rule {}: parameter {k} must be a non-zero [x,y,z] vector", def.id));
                }
            }
            (PKind::Num, _) => return Err(format!("rule {}: parameter {k} must be a number", def.id)),
            (PKind::Vec3, _) => return Err(format!("rule {}: parameter {k} must be an [x,y,z] array", def.id)),
        }
        if let Some(slot) = vals.iter_mut().find(|(n, _)| *n == d.name) {
            slot.1 = *v;
        }
    }
    Ok(Resolved { def, params: Params(vals), severity: spec.severity.unwrap_or(def.severity) })
}

fn parse_profile(j: &Json) -> Result<Profile, String> {
    let Some(rules) = j.get_arr("rules") else { return Err("profile needs a \"rules\" array".into()) };
    let mut out = vec![];
    for r in rules {
        let id = r.get_str("id").ok_or("every rule needs an \"id\"")?;
        let mut params = vec![];
        match r.get("params") {
            None | Some(Json::Null) => {}
            Some(Json::Obj(m)) => {
                for (k, v) in m.iter() {
                    let pv = match v {
                        Json::Num(x) => PVal::Num(*x),
                        Json::Arr(a) if a.len() == 3 && a.iter().all(|x| x.as_f64().is_some()) => {
                            PVal::Vec([a[0].as_f64().unwrap(), a[1].as_f64().unwrap(), a[2].as_f64().unwrap()])
                        }
                        _ => return Err(format!("rule {id}: parameter {k} must be a number or an [x,y,z] array")),
                    };
                    params.push((k.clone(), pv));
                }
            }
            Some(_) => return Err(format!("rule {id}: \"params\" must be an object")),
        }
        let severity = match r.get_str("severity") {
            None => None,
            Some(s) => Some(Severity::parse(s).ok_or_else(|| format!("rule {id}: severity must be error, warn or info (got \"{s}\")"))?),
        };
        out.push(RuleSpec { id: id.to_string(), params, severity });
    }
    Ok(Profile {
        id: j.get_str("id").unwrap_or("inline").to_string(),
        name: j.get_str("name").unwrap_or("Inline profile").to_string(),
        description: j.get_str("description").unwrap_or("").to_string(),
        rules: out,
    })
}

/// A `profile` field: a built-in id or an inline definition.
pub fn select_profile(j: Option<&Json>) -> Result<Profile, String> {
    match j {
        Some(Json::Str(id)) => {
            let all = builtin_profiles();
            all.iter().find(|p| p.id == *id).cloned().ok_or_else(|| format!("unknown profile \"{id}\"; profiles: {}", all.iter().map(|p| p.id.as_str()).collect::<Vec<_>>().join(", ")))
        }
        Some(o @ Json::Obj(_)) => parse_profile(o),
        _ => Err(format!("checkProfile needs \"profile\": a profile id ({}) or an inline {{rules:[...]}} profile", builtin_profiles().iter().map(|p| p.id.as_str()).collect::<Vec<_>>().join(", "))),
    }
}

// ---------------------------------------------------------------- JSON

fn param_defs_json(defs: &[ParamDef], current: Option<&Params>) -> Json {
    Json::Arr(
        defs.iter()
            .map(|d| {
                let value = current.and_then(|c| c.0.iter().find(|(n, _)| *n == d.name)).map(|(_, v)| *v).unwrap_or(d.default);
                Obj::new()
                    .put("name", d.name)
                    .put("type", if d.kind == PKind::Num { "number" } else { "vec3" })
                    .put("default", pval_json(&d.default))
                    .put("value", pval_json(&value))
                    .put("unit", d.unit)
                    .put_opt("min", d.min)
                    .put_opt("max", d.max)
                    .put("description", d.doc)
                    .build()
            })
            .collect(),
    )
}

fn rule_json(r: &Resolved) -> Json {
    Obj::new()
        .put("id", r.def.id)
        .put("title", r.def.title)
        .put("description", r.def.doc)
        .put("severity", r.severity.as_str())
        .put("params", param_defs_json(r.def.params, Some(&r.params)))
        .build()
}

/// `{type:"profiles"}`: built-in profiles (effective parameters) and the rule catalogue.
pub fn profiles_reply() -> Json {
    let profiles: Vec<Json> = builtin_profiles()
        .iter()
        .map(|p| {
            let rules: Vec<Json> = p.rules.iter().filter_map(|s| resolve_rule(s).ok()).map(|r| rule_json(&r)).collect();
            Obj::new().put("id", p.id.as_str()).put("name", p.name.as_str()).put("description", p.description.as_str()).put("rules", Json::Arr(rules)).build()
        })
        .collect();
    let catalogue: Vec<Json> = rule_defs()
        .iter()
        .map(|d| {
            Obj::new()
                .put("id", d.id)
                .put("title", d.title)
                .put("description", d.doc)
                .put("severity", d.severity.as_str())
                .put("params", param_defs_json(d.params, None))
                .build()
        })
        .collect();
    Obj::new().put("profiles", Json::Arr(profiles)).put("rules", Json::Arr(catalogue)).build()
}

/// Run options from `opts`.
pub struct RunOpts {
    /// Surface samples for the sampled rules (default 3000).
    pub samples: usize,
    /// Cap per rule (default 100); the summary reports what was cut.
    pub max_issues: usize,
}

impl RunOpts {
    pub fn from_json(o: Option<&Json>) -> RunOpts {
        let g = |k: &str, d: f64| o.and_then(|o| o.get_f64(k)).filter(|x| x.is_finite() && *x >= 1.0).unwrap_or(d);
        RunOpts { samples: g("samples", 3000.0).min(100000.0) as usize, max_issues: g("maxIssues", 100.0) as usize }
    }
}

/// Run `profile` on `body`: `{profile, ok, issues, summary}`.
pub fn run(body: &Body, profile: &Profile, opts: &RunOpts) -> Result<Json, String> {
    let resolved: Vec<Resolved> = profile.rules.iter().map(resolve_rule).collect::<Result<_, _>>()?;
    let cx = Ctx::new(body, opts.samples);
    let mut issues: Vec<Issue> = vec![];
    let mut per_rule = vec![];
    let mut truncated = vec![];
    for r in &resolved {
        let mut found = (r.def.run)(&cx, &r.params, r.severity);
        // errors first inside a rule is not needed: rules emit worst-first
        let total = found.len();
        if total > opts.max_issues {
            found.truncate(opts.max_issues);
            truncated.push(Obj::new().put("rule", r.def.id).put("total", total as f64).put("shown", opts.max_issues as f64).build());
        }
        per_rule.push(
            Obj::new().put("rule", r.def.id).put("severity", r.severity.as_str()).put("issues", total as f64).put("passed", total == 0).build(),
        );
        issues.extend(found);
    }
    let count = |s: Severity| issues.iter().filter(|i| i.severity == s).count() as f64;
    let skipped: Vec<Json> = cx.skipped_if_built().iter().filter_map(|&i| cx.names.get(i)).map(|n| Json::from(n.as_str())).collect();
    let (ne, nw, ni) = (count(Severity::Error), count(Severity::Warn), count(Severity::Info));
    let summary = Obj::new()
        .put("errors", ne)
        .put("warnings", nw)
        .put("infos", ni)
        .put("total", issues.len() as f64)
        .put("rules", Json::Arr(per_rule))
        .put("truncated", Json::Arr(truncated))
        .put("skippedFaces", Json::Arr(skipped))
        .put("samples", opts.samples as f64)
        .build();
    Ok(Obj::new()
        .put("profile", Obj::new().put("id", profile.id.as_str()).put("name", profile.name.as_str()).put("rules", describe_profile(profile)?).build())
        .put("ok", ne == 0.0)
        .put("issues", Json::Arr(issues.iter().map(Issue::to_json).collect()))
        .put("summary", summary)
        .build())
}

/// The effective rule list of a profile (defaults merged), for the reply and docs.
pub fn describe_profile(p: &Profile) -> Result<Json, String> {
    let rules: Vec<Json> = p.rules.iter().map(|s| resolve_rule(s).map(|r| rule_json(&r))).collect::<Result<_, _>>()?;
    Ok(Json::Arr(rules))
}
