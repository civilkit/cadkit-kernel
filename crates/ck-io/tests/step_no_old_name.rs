//! Leak test: a STEP file the kernel writes must not carry the old product
//! name. (The name is built from pieces so this source does not contain it.)
mod bodies;
use ck_io::step::{write_step, StepOpts};

#[test]
fn written_step_carries_no_old_product_name() {
    let old = ["purra", "metric"].concat();
    for name in bodies::NAMES {
        let text = write_step(&bodies::build(name), name, &StepOpts { unix_ms: 0.0 });
        assert!(!text.to_lowercase().contains(&old), "{name}: STEP output names the old product");
        assert!(text.contains("FILE_DESCRIPTION(('CadKit model'),'2;1');"), "{name}: header description");
    }
}

#[test]
fn golden_files_carry_no_old_product_name() {
    let old = ["purra", "metric"].concat();
    let dir = format!("{}/tests/golden", env!("CARGO_MANIFEST_DIR"));
    for e in std::fs::read_dir(dir).unwrap() {
        let p = e.unwrap().path();
        let t = std::fs::read_to_string(&p).unwrap_or_default();
        assert!(!t.to_lowercase().contains(&old), "{}: names the old product", p.display());
    }
}
