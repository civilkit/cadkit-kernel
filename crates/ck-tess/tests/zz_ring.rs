#[path = "oracle_mesh.rs"]
#[allow(dead_code)]
mod om;
use om::build;
use ck_json::Json;
use ck_tess::mesh::tessellate;
fn gold() -> Json { ck_json::parse(include_str!("oracle_mesh.json")).unwrap() }
fn nums(j: &Json) -> Vec<f64> { j.as_arr().unwrap().iter().map(|x| x.as_f64().unwrap()).collect() }
#[test]
fn ring_probe() {
    let name = std::env::var("RP_CASE").unwrap_or("cyl".into());
    let gdir = "/private/tmp/claude-501/-Volumes-second-disk-tmp-purr-rebuild/296fdc97-740a-4f73-841f-21dbda16b0ed/scratchpad/gold/";
    let gtxt = std::fs::read_to_string(format!("{gdir}{name}.json")).unwrap();
    let gc = ck_json::parse(&gtxt).unwrap();
    let face: i32 = std::env::var("RP_FACE").unwrap_or("2".into()).parse().unwrap();
    let gpos: Vec<f32> = nums(gc.get("positions").unwrap()).into_iter().map(|x| x as f32).collect();
    let gtf: Vec<i32> = nums(gc.get("triFace").unwrap()).into_iter().map(|x| x as i32).collect();
    let b = build(&name);
    let want: Vec<[u32; 9]> = (0..gtf.len()).filter(|&t| gtf[t] == face).map(|t| { let mut a = [0u32; 9]; for k in 0..9 { a[k] = gpos[t * 9 + k].to_bits(); } a }).collect();
    for dir in 0..2 {
        for st in 0..400 {
            std::env::set_var("CK_RING", format!("{st},{dir}"));
            let t = tessellate(&b);
            let got: Vec<[u32; 9]> = (0..t.mesh.tri_face.len()).filter(|&i| t.mesh.tri_face[i] == face).map(|i| { let mut a = [0u32; 9]; for k in 0..9 { a[k] = t.mesh.positions[i * 9 + k].to_bits(); } a }).collect();
            let eq = got.iter().zip(&want).take_while(|(a, b)| a == b).count();
            if eq >= 8 { println!("RING st={st} dir={dir} prefix={eq}/{}", want.len()); }
        }
    }
}

#[test]
fn dump_tris() {
    let name = std::env::var("RP_CASE").unwrap_or("dext".into());
    let face: i32 = std::env::var("RP_FACE").unwrap_or("0".into()).parse().unwrap();
    let gdir = "/private/tmp/claude-501/-Volumes-second-disk-tmp-purr-rebuild/296fdc97-740a-4f73-841f-21dbda16b0ed/scratchpad/gold/";
    let gtxt = std::fs::read_to_string(format!("{gdir}{name}.json")).unwrap();
    let gc = ck_json::parse(&gtxt).unwrap();
    let gpos: Vec<f32> = nums(gc.get("positions").unwrap()).into_iter().map(|x| x as f32).collect();
    let gtf: Vec<i32> = nums(gc.get("triFace").unwrap()).into_iter().map(|x| x as i32).collect();
    let b = build(&name);
    let t = tessellate(&b);
    // vertex ids by first appearance in OUR list
    let mut ids: Vec<[u32; 3]> = vec![];
    let mut id = |p: [u32; 3], ids: &mut Vec<[u32; 3]>| -> usize { if let Some(i) = ids.iter().position(|q| *q == p) { i } else { ids.push(p); ids.len() - 1 } };
    let mut a = vec![];
    for i in 0..t.mesh.tri_face.len() { if t.mesh.tri_face[i] == face { let v: Vec<usize> = (0..3).map(|k| id([0,1,2].map(|c| t.mesh.positions[i*9+k*3+c].to_bits()), &mut ids)).collect(); a.push(v); } }
    let mut o = vec![];
    for i in 0..gtf.len() { if gtf[i] == face { let v: Vec<usize> = (0..3).map(|k| id([0,1,2].map(|c| gpos[i*9+k*3+c].to_bits()), &mut ids)).collect(); o.push(v); } }
    for (i, q) in ids.iter().enumerate().take(14) { println!("ID {} {} {} {}", i, f32::from_bits(q[0]), f32::from_bits(q[1]), f32::from_bits(q[2])); }
    println!("OURS {:?}", a);
    println!("ORIG {:?}", o);
}
