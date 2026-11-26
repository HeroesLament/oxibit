#![cfg(feature = "builtin-data")]
//! The built-in chip data must equal the Trellis text it was generated from.
//! Skipped when no Trellis checkout is found ($OXIBIT_TRELLIS_DB or the
//! oss-cad-suite copy).

use std::path::PathBuf;

use oxibit_ecp5::{DeviceDb, Source};

fn trellis() -> Option<PathBuf> {
    let p = match std::env::var_os("OXIBIT_TRELLIS_DB") {
        Some(p) => PathBuf::from(p),
        None => PathBuf::from(std::env::var_os("HOME")?).join("oss-cad-suite/share/trellis/database"),
    };
    p.join("devices.json").exists().then_some(p)
}

#[test]
fn builtin_equals_text() {
    let Some(root) = trellis() else {
        eprintln!("no Trellis database; skipped");
        return;
    };
    let text = Source::text(&root);
    for dev in oxibit_arch_ecp5::devices::ALL.iter().map(|d| d.name) {
        let a = DeviceDb::load_from(&text, dev).unwrap();
        let b = DeviceDb::load_from(&Source::Builtin, dev).unwrap();
        assert_eq!(a.info, b.info, "{dev}: info");
        assert_eq!(a.tiles, b.tiles, "{dev}: tiles");
        assert_eq!(a.bits.keys().collect::<Vec<_>>(), b.bits.keys().collect::<Vec<_>>(), "{dev}: types");
        for (ty, tb) in &a.bits {
            assert_eq!(tb, &b.bits[ty], "{dev}: tile type {ty}");
        }
        assert_eq!(text.iodb(dev).unwrap(), Source::Builtin.iodb(dev).unwrap(), "{dev}: iodb.json");
        assert_eq!(text.globals(dev).unwrap(), Source::Builtin.globals(dev).unwrap(), "{dev}: globals.json");
    }
}

#[test]
fn builtin_idcode_lookup() {
    let id = oxibit_ecp5::db::device_for_idcode_in(&Source::Builtin, 0x4111_3043).unwrap();
    assert_eq!(id, "LFE5U-85F");
}
