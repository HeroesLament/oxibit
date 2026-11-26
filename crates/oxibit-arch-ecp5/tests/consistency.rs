use oxibit_arch_ecp5::{device, devices, tile_type, tiles};

#[test]
fn tile_types_are_sorted_and_found() {
    assert!(tiles::ALL.windows(2).all(|w| w[0].name < w[1].name));
    for t in tiles::ALL {
        assert!(std::ptr::eq(tile_type(t.name).expect("listed tile type"), *t));
    }
    assert!(tile_type("NO_SUCH_TILE").is_none());
}

#[test]
fn device_tiles_are_sorted_and_typed() {
    for d in devices::ALL {
        assert!(d.tiles.windows(2).all(|w| w[0].name < w[1].name), "{} tiles", d.name);
        for t in d.tiles {
            let known = tile_type(t.ty.name).expect("tile type of a tile");
            assert!(std::ptr::eq(known, t.ty), "{} {}", d.name, t.name);
        }
        assert!(std::ptr::eq(device(d.name).expect("listed device"), *d));
    }
}

#[test]
fn spot_checks() {
    let plc2 = tile_type("PLC2").expect("PLC2");
    let a1 = plc2.muxes.iter().find(|m| m.sink == "A1").expect("mux A1");
    assert_eq!(a1.arcs.len(), 24);
    assert_eq!(a1.arcs[0].source, "E1_H01E0001");
    let d = device("LFE5U-85F").expect("85F");
    assert_eq!((d.idcode, d.tiles.len()), (0x4111_3043, 14231));
}
