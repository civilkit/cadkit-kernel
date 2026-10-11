//! Golden tests: every request in tests-golden/wire.json (captured from the
//! ORIGINAL kernel by tools/golden.mjs) must produce the identical reply
//! JSON and total frame length.

use super::*;

const GOLDEN: &str = include_str!("../../../tests-golden/wire.json");

fn run(server: &mut Server, body: &str) -> (String, usize) {
    let r = server.handle(body.as_bytes());
    let (j, _) = reply::decode(&r).expect("frame");
    (j.to_string(), r.len())
}

#[test]
fn wire_golden() {
    let g = ck_json::parse(GOLDEN).unwrap();
    let mut fails = vec![];
    let mut n = 0;
    for case in g.as_arr().unwrap() {
        let name = case.get_str("name").unwrap();
        let mut s = Server::new();
        for step in case.get_arr("steps").unwrap() {
            let body = step.get_str("body").unwrap();
            let Some(want) = step.get_str("json") else { continue };
            n += 1;
            let (got, total) = run(&mut s, body);
            let want_total = step.get_f64("total").unwrap() as usize;
            if got != want || total != want_total {
                fails.push(format!("{name}: {}\n   want {want} ({want_total})\n   got  {got} ({total})", &body[..body.len().min(120)]));
            }
        }
    }
    assert!(fails.is_empty(), "{}/{n} golden mismatches:\n{}", fails.len(), fails.join("\n"));
}

#[test]
fn all_types_dispatch() {
    assert_eq!(REQUEST_TYPES.len(), 19); // 17 original + validate + importSat extensions
    for (t, _) in REQUEST_TYPES {
        let (j, _) = run(&mut Server::new(), &format!(r#"{{"msg":{{"type":"{t}"}}}}"#));
        assert!(!j.contains("unknown request"), "{t}");
    }
}

#[test]
fn debug_clock_protocol() {
    // oracle with ck_now = 1,4,9,16,25,... :
    // {"ok":true,"result":null,"ms":24,"buffers":{"count":0,"bytes":0},
    //  "trace":[{"at":5,"ms":7,"depth":0,"label":"interference between the bodies","level":"span"}]}
    thread_local!(static T: std::cell::Cell<f64> = const { std::cell::Cell::new(0.0) });
    fn sq() -> f64 {
        T.with(|t| {
            t.set(t.get() + 1.0);
            t.get() * t.get()
        })
    }
    clock::set_now(sq);
    let mut s = Server::new();
    T.with(|t| t.set(0.0));
    let (j, _) = run(&mut s, r#"{"msg":{"type":"regen","part":{"format":"ck-part","features":[]},"debug":true}}"#);
    assert_eq!(j, r#"{"ok":true,"result":{"errors":[],"components":[],"planes":[],"markers":[],"sketches":[],"sketches3":[],"curves":[],"body":null,"surfaces":[]},"ms":48,"buffers":{"count":0,"bytes":0},"trace":[{"at":5,"ms":7,"depth":0,"label":"regenerate 0 features","level":"span"},{"at":21,"ms":11,"depth":0,"label":"snapshot for the view (display meshes, edges)","level":"span"}]}"#);
    T.with(|t| t.set(0.0));
    let (j, _) = run(&mut s, r#"{"msg":{"type":"interference","debug":true}}"#);
    assert_eq!(j, r#"{"ok":true,"result":null,"ms":24,"buffers":{"count":0,"bytes":0},"trace":[{"at":5,"ms":7,"depth":0,"label":"interference between the bodies","level":"span"}]}"#);
    T.with(|t| t.set(0.0));
    let (j, _) = run(&mut s, r#"{"msg":{"type":"interference","debug":true,"tag":"zz"}}"#);
    assert_eq!(j, r#"{"ok":false,"error":"noState: nothing regenerated yet","ms":24,"trace":[{"at":5,"ms":7,"depth":0,"label":"interference between the bodies","level":"span"}]}"#);
    T.with(|t| t.set(0.0));
    let (j, _) = run(&mut s, r#"{"msg":{"type":"ping","debug":true}}"#);
    assert_eq!(j, r#"{"ok":true,"result":{"pong":true},"ms":8,"buffers":{"count":0,"bytes":0},"trace":[]}"#);
    T.with(|t| t.set(0.0));
    // non-debug: 2 samples (1 and 4) -> ms 3; bad body: ms 0
    let (j, _) = run(&mut s, r#"{"msg":{"type":"interference"}}"#);
    assert_eq!(j, r#"{"ok":true,"result":null,"ms":3}"#);
    let (j, _) = run(&mut s, "nope");
    assert_eq!(j, r#"{"ok":false,"error":"bad request: the body must be JSON {id, msg}","ms":0}"#);
    clock::set_now(|| 0.0);
}
