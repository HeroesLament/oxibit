//! Device identity: IDCODE to die.
//!
//! Table extracted from prjxray-db revision 517d66a (the openXC7 fork),
//! `idcode:` in `<family>/<part>/part.yaml`; see `data/prjxray-idcodes.txt`.
//! IDCODE bits [31:28] are the silicon revision and are ignored for matching.

use serde::Serialize;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct Die {
    pub family: &'static str,
    pub name: &'static str,
    pub idcode: u32,
}

const fn die(family: &'static str, name: &'static str, idcode: u32) -> Die {
    Die { family, name, idcode }
}

pub const DIES: &[Die] = &[
    die("artix7", "xc7a35t", 0x0362_D093),
    die("artix7", "xc7a50t", 0x0362_C093),
    die("artix7", "xc7a100t", 0x0363_1093),
    die("artix7", "xc7a200t", 0x0363_6093),
    die("kintex7", "xc7k70t", 0x0364_7093),
    die("kintex7", "xc7k160t", 0x0364_C093),
    die("kintex7", "xc7k325t", 0x0365_1093),
    die("kintex7", "xc7k420t", 0x0375_2093),
    die("kintex7", "xc7k480t", 0x0375_1093),
    die("spartan7", "xc7s6", 0x0362_2093),
    die("spartan7", "xc7s15", 0x0362_0093),
    die("spartan7", "xc7s25", 0x037C_4093),
    die("spartan7", "xc7s50", 0x0362_F093),
    die("spartan7", "xc7s75", 0x037C_8093),
    die("spartan7", "xc7s100", 0x037C_7093),
    die("virtex7", "xc7vx485t", 0x0368_7093),
    die("zynq7", "xc7z007", 0x0372_3093),
    die("zynq7", "xc7z010", 0x0372_2093),
    die("zynq7", "xc7z020", 0x0372_7093),
    die("zynq7", "xc7z030", 0x0372_C093),
    die("zynq7", "xc7z035", 0x0373_2093),
    die("zynq7", "xc7z045", 0x0373_1093),
    die("zynq7", "xc7z100", 0x0373_6093),
];

const REVISION_MASK: u32 = 0x0FFF_FFFF;

pub fn by_idcode(idcode: u32) -> Option<&'static Die> {
    DIES.iter().find(|d| d.idcode & REVISION_MASK == idcode & REVISION_MASK)
}

/// Die for a full part name (`xc7z020clg400-1`) or a bare die (`xc7z020`).
/// Longest prefix wins, so `xc7s100...` never matches `xc7s15`.
///
/// Vivado writes record `b` without the `xc` prefix (`7z020clg400`); that is
/// accepted too.
pub fn by_part(part: &str) -> Option<&'static Die> {
    let mut part = part.to_ascii_lowercase();
    if !part.starts_with("xc") {
        part.insert_str(0, "xc");
    }
    DIES.iter().filter(|d| part.starts_with(d.name)).max_by_key(|d| d.name.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lookups() {
        assert_eq!(by_idcode(0x1372_7093).unwrap().name, "xc7z020");
        assert_eq!(by_part("xc7z020clg400-1").unwrap().name, "xc7z020");
        assert_eq!(by_part("XC7S100FGGA484-1").unwrap().name, "xc7s100");
        assert_eq!(by_part("7z020clg400").unwrap().name, "xc7z020");
        assert!(by_part("xc7z999").is_none());
    }
}
