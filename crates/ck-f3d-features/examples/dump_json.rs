fn main() {
    let a: Vec<String> = std::env::args().collect();
    let bytes = std::fs::read(&a[1]).unwrap();
    for d in ck_f3d_design::decode_bytes(bytes, &a[1]).unwrap() {
        if let Some(dump) = d.dump {
            println!("{}", serde_json::to_string_pretty(&dump).unwrap());
        }
    }
}
