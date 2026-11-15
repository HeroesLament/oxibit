//! Bels per tile type, their pins, and the tiles each bel's configuration
//! is spread over. This mirrors prjoxide's `get_tile_bels`,
//! `get_bel_tiles` and the `Bel::make_*` pin lists case for case: a
//! tilegroup name or membership that differs from prjoxide's changes the
//! bitstream, and a pin wire that differs connects a cell to the wrong
//! routing node.

use std::collections::BTreeSet;

use crate::chip::Chip;
use crate::db::TileBits;
use crate::{Result, err};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PinDir {
    Input,
    Output,
    Inout,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BelPin {
    pub name: String,
    pub dir: PinDir,
    /// Wire name in the namespace of the tile the bel was found in,
    /// relative prefix included (`S1:JHFCLKOUT_OSC_CORE`).
    pub wire: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Bel {
    pub name: String,
    pub beltype: String,
    /// Offset of the bel's nominal location from its tile; tilegroups are
    /// named `R<tile.y + rel_y>C<tile.x + rel_x>_<name>`.
    pub rel_x: i32,
    pub rel_y: i32,
    pub z: u32,
    pub pins: Vec<BelPin>,
}

const Z_TO_CHAR: [char; 4] = ['A', 'B', 'C', 'D'];

/// prjoxide's relative-location prefix: `N<dy>`/`S<dy>` then `E<dx>`/`W<dx>`, then `:`.
pub fn rel_prefix(dx: i32, dy: i32) -> String {
    let mut p = String::new();
    if dy < 0 {
        p += &format!("N{}", -dy);
    }
    if dy > 0 {
        p += &format!("S{dy}");
    }
    if dx < 0 {
        p += &format!("W{}", -dx);
    }
    if dx > 0 {
        p += &format!("E{dx}");
    }
    if !p.is_empty() {
        p.push(':');
    }
    p
}

/// A pin before the bel's offset is applied: wire relative to the bel.
struct RawPin {
    name: String,
    dir: PinDir,
    wire: String,
    dx: i32,
    dy: i32,
}

/// `J<phys>_<postfix>`, the standard bel pin wire.
fn p(dir: PinDir, postfix: &str, name: &str, phys: &str) -> RawPin {
    RawPin { name: name.into(), dir, wire: format!("J{phys}_{postfix}"), dx: 0, dy: 0 }
}

fn i(postfix: &str, name: &str) -> RawPin {
    p(PinDir::Input, postfix, name, name)
}

fn o(postfix: &str, name: &str) -> RawPin {
    p(PinDir::Output, postfix, name, name)
}

fn mk(name: impl Into<String>, beltype: impl Into<String>, rel_x: i32, rel_y: i32, z: u32, raw: Vec<RawPin>) -> Bel {
    let pins = raw
        .into_iter()
        .map(|r| BelPin { name: r.name, dir: r.dir, wire: format!("{}{}", rel_prefix(rel_x + r.dx, rel_y + r.dy), r.wire) })
        .collect();
    Bel { name: name.into(), beltype: beltype.into(), rel_x, rel_y, z, pins }
}

/// Pins discovered from routing: wires ending in `postfix` that pips or
/// fixed connections enter (inputs) or leave (outputs), excluding
/// route-throughs. Pin name is the wire minus postfix and leading `J`.
fn get_io(tb: &TileBits, postfix: &str, rel_x: i32, rel_y: i32) -> Vec<RawPin> {
    let prefix = rel_prefix(rel_x, rel_y);
    let mut sinks = BTreeSet::new();
    let mut srcs = BTreeSet::new();
    let arcs = tb
        .pips
        .iter()
        .flat_map(|(to, ps)| ps.iter().map(move |p| (to, &p.from_wire)))
        .chain(tb.conns.iter().flat_map(|(to, cs)| cs.iter().map(move |c| (to, &c.from_wire))));
    for (to, from) in arcs {
        if to.ends_with(postfix) && !from.ends_with(postfix) {
            sinks.insert(to.clone());
        }
        if !to.ends_with(postfix) && from.ends_with(postfix) {
            srcs.insert(from.clone());
        }
    }
    let mut pins = Vec::new();
    let mut add = |w: &str, dir| {
        if let Some(local) = w.strip_prefix(prefix.as_str()) {
            if let Some(stem) = local.strip_suffix(postfix) {
                let name = stem.strip_prefix('J').unwrap_or(stem);
                pins.push(RawPin { name: name.into(), dir, wire: local.into(), dx: 0, dy: 0 });
            }
        }
    };
    for w in &srcs {
        add(w, PinDir::Output);
    }
    for w in &sinks {
        add(w, PinDir::Input);
    }
    pins
}

fn comb(slice: usize, lut: usize) -> Bel {
    let pf = format!("SLICE{}", Z_TO_CHAR[slice]);
    let pf = pf.as_str();
    let mut v = vec![
        p(PinDir::Input, pf, "A", &format!("A{lut}")),
        p(PinDir::Input, pf, "B", &format!("B{lut}")),
        p(PinDir::Input, pf, "C", &format!("C{lut}")),
        p(PinDir::Input, pf, "D", &format!("D{lut}")),
        p(PinDir::Input, pf, "FCI", if lut == 0 { "FCI" } else { "INT_CARRY" }),
        p(PinDir::Output, pf, "F", &format!("F{lut}")),
        p(PinDir::Output, pf, "FCO", if lut == 1 { "FCO" } else { "INT_CARRY" }),
    ];
    if lut == 0 {
        v.extend([i(pf, "SEL"), i(pf, "F1"), p(PinDir::Output, pf, "OFX", "OFX0")]);
    }
    if slice < 2 {
        v.extend([i(pf, "WAD0"), i(pf, "WAD1"), i(pf, "WAD2"), i(pf, "WAD3")]);
        v.push(p(PinDir::Input, pf, "WDI", &format!("WDI{lut}")));
        v.extend([i(pf, "WCK"), i(pf, "WRE")]);
    }
    mk(format!("{pf}_LUT{lut}"), "OXIDE_COMB", 0, 0, (slice << 3 | lut) as u32, v)
}

fn ff(slice: usize, n: usize) -> Bel {
    let pf = format!("SLICE{}", Z_TO_CHAR[slice]);
    let pf = pf.as_str();
    let v = vec![
        i(pf, "CLK"),
        i(pf, "CE"),
        i(pf, "LSR"),
        p(PinDir::Input, pf, "DI", &format!("DI{n}")),
        p(PinDir::Input, pf, "M", &format!("M{n}")),
        p(PinDir::Output, pf, "Q", &format!("Q{n}")),
    ];
    mk(format!("{pf}_FF{n}"), "OXIDE_FF", 0, 0, (slice << 3 | (n + 2)) as u32, v)
}

fn ramw() -> Bel {
    let pf = "SLICEC";
    let mut v: Vec<RawPin> =
        ["A0", "A1", "B0", "B1", "C0", "C1", "D0", "D1", "CLK", "LSR"].iter().map(|n| i(pf, n)).collect();
    v.extend(
        ["WADO0", "WADO1", "WADO2", "WADO3", "WCKO", "WREO", "WDO0", "WDO1", "WDO2", "WDO3"].iter().map(|n| o(pf, n)),
    );
    mk("SLICEC_RAMW", "RAMW", 0, 0, (2 << 3 | 4) as u32, v)
}

fn plc() -> Vec<Bel> {
    let mut v = Vec::new();
    for slice in 0..4 {
        v.extend([comb(slice, 0), comb(slice, 1), ff(slice, 0), ff(slice, 1)]);
        if slice == 2 {
            v.push(ramw());
        }
    }
    v
}

fn seio33(z: usize) -> Bel {
    let ch = Z_TO_CHAR[z];
    let pf = format!("SEIO33_CORE_IO{ch}");
    let pf = pf.as_str();
    let v = vec![
        p(PinDir::Inout, pf, "B", "PAD"),
        p(PinDir::Input, pf, "I", "PADDO"),
        p(PinDir::Input, pf, "T", "PADDT"),
        p(PinDir::Output, pf, "O", "PADDI"),
        i(pf, "I3CRESEN"),
        i(pf, "I3CWKPU"),
    ];
    mk(format!("PIO{ch}"), "SEIO33_CORE", 0, 0, z as u32, v)
}

fn seio18(z: usize) -> Bel {
    let ch = Z_TO_CHAR[z];
    let pf = if z == 1 { format!("SEIO18_CORE_IO{ch}") } else { format!("DIFFIO18_CORE_IO{ch}") };
    let pf = pf.as_str();
    let v = vec![
        p(PinDir::Inout, pf, "B", "IOPAD"),
        p(PinDir::Input, pf, "I", "PADDO"),
        p(PinDir::Input, pf, "T", "PADDT"),
        p(PinDir::Output, pf, "O", "PADDI"),
        i(pf, "DOLP"),
        o(pf, "INLP"),
        o(pf, "INADC"),
    ];
    mk(format!("PIO{ch}"), "SEIO18_CORE", 0, 0, z as u32, v)
}

fn diffio18() -> Bel {
    let pf = "DIFFIO18_CORE_IOA";
    let v = vec![
        p(PinDir::Inout, pf, "B", "IOPAD"),
        p(PinDir::Input, pf, "I", "PADDO"),
        p(PinDir::Input, pf, "T", "PADDT"),
        p(PinDir::Output, pf, "O", "PADDI"),
        i(pf, "DOLP"),
        i(pf, "HSRXEN"),
        i(pf, "HSTXEN"),
        o(pf, "INLP"),
        o(pf, "INADC"),
    ];
    mk("DIFFIO18", "DIFFIO18_CORE", 0, 0, 2, v)
}

fn osc() -> Bel {
    let pf = "OSC_CORE";
    let mut v = vec![o(pf, "HFCLKOUT"), o(pf, "LFCLKOUT"), i(pf, "HFOUTEN")];
    for k in (0..9).rev() {
        v.push(i(pf, &format!("HFTRMFAB{k}")));
    }
    for k in (0..9).rev() {
        v.push(i(pf, &format!("LFTRMFAB{k}")));
    }
    v.extend([i(pf, "HFSDSCEN"), o(pf, "HFCLKCFG"), o(pf, "HFSDCOUT")]);
    mk("OSC_CORE", "OSC_CORE", 0, 1, 0, v)
}

fn iol(tb: &TileBits, s: bool, z: usize) -> Bel {
    let ch = Z_TO_CHAR[z];
    let pf = if s { format!("_SIOLOGIC_CORE_IBASE_PIC_{ch}") } else { format!("_IOLOGIC_CORE_I_GEARING_PIC_TOP_{ch}") };
    let s = if s { "S" } else { "" };
    mk(format!("{s}IOLOGIC{ch}"), format!("{s}IOLOGIC"), 0, 0, (z + 3) as u32, get_io(tb, &pf, 0, 0))
}

fn config(tb: &TileBits, rel_x: i32, hse_postfix: &str) -> Vec<Bel> {
    [
        ("CONFIG_MULTIBOOT_CORE", "_CONFIG_MULTIBOOT_CORE_CONFIG_MULTIBOOT"),
        ("CONFIG_HSE_CORE", hse_postfix),
        ("CONFIG_LMMI_CORE", "_CONFIG_LMMI_CORE_LMMI_MODEL"),
        ("CONFIG_CLKRST_CORE", "_CONFIG_CLKRST_CORE_CONFIG_CLKRST"),
    ]
    .iter()
    .enumerate()
    .map(|(z, (n, pf))| mk(*n, *n, rel_x, 0, z as u32, get_io(tb, pf, rel_x, 0)))
    .collect()
}

fn ebr(tb: &TileBits, z: u32) -> Bel {
    mk(format!("EBR{z}"), "OXIDE_EBR", -1, -1, z, get_io(tb, "_EBR_CORE", -1, -1))
}

fn dsp(tb: &TileBits, names: &[(&str, &str)]) -> Vec<Bel> {
    names
        .iter()
        .enumerate()
        .map(|(z, (n, t))| mk(*n, *t, 0, -1, z as u32, get_io(tb, &format!("_{t}_{n}"), 0, -1)))
        .collect()
}

fn dcc(side: &str, z: u32) -> Bel {
    let rel_x = if side == "R" { -1 } else { 0 };
    let rel_y = if side == "C" { -1 } else { 0 };
    let pf = format!("DCC_DCC{z}");
    let v = vec![i(&pf, "CLKI"), i(&pf, "CE"), o(&pf, "CLKO")];
    mk(format!("DCC_{side}{z}"), "DCC", rel_x, rel_y, z, v)
}

fn dcs() -> Bel {
    let pf = "DCS_DCSIP";
    let v = vec![i(pf, "CLK0"), i(pf, "CLK1"), i(pf, "SEL"), i(pf, "SELFORCE"), o(pf, "DCSOUT")];
    mk("DCS0", "DCS", 0, -1, 4, v)
}

fn vcc() -> Bel {
    let v = vec![RawPin { name: "Z".into(), dir: PinDir::Output, wire: "G:VCC".into(), dx: 0, dy: 0 }];
    mk("VCC_DRV", "VCC_DRV", 0, 0, 16, v)
}

fn get_io_bel(tb: &TileBits, name: &str, beltype: &str, postfix: &str, rel_x: i32, rel_y: i32, z: u32) -> Bel {
    mk(name, beltype, rel_x, rel_y, z, get_io(tb, postfix, rel_x, rel_y))
}

fn eclk(tb: &TileBits, bank: u32) -> Vec<Bel> {
    let mut v: Vec<Bel> = (0..4)
        .map(|z| {
            let pf = format!("_ECLKSYNC_CORE_ECLKSYNC{bank}_{z}");
            get_io_bel(tb, &format!("ECLKSYNC{bank}{}", Z_TO_CHAR[z as usize]), "ECLKSYNC_CORE", &pf, 0, -1, 24 + z)
        })
        .collect();
    v.extend((0..4).map(|z| {
        let pf = format!("_ECLKDIV_CORE_ECLKDIVB{bank}_{z}");
        get_io_bel(tb, &format!("ECLKDIV{bank}{}", Z_TO_CHAR[z as usize]), "ECLKDIV_CORE", &pf, 0, -1, 28 + z)
    }));
    v
}

fn dqsbuf(tb: &TileBits) -> Bel {
    let mut v = get_io(tb, "_DQSBUF_CORE_I_DQS_TOP", -5, -1);
    let extra = |n: &str| RawPin {
        name: n.into(),
        dir: PinDir::Output,
        wire: format!("J{n}_DQSBUF_CORE_I_DQS_TOP"),
        dx: 6,
        dy: 1,
    };
    v.extend([extra("DQSR90"), extra("DQSW270"), extra("DQSW")]);
    for k in 0..3 {
        v.push(extra(&format!("RDPNTR{k}")));
        v.push(extra(&format!("WRPNTR{k}")));
    }
    // Extra pins sit at (+6, +1) from the bel, i.e. E1: of the tile.
    mk("DQSBUF", "DQSBUF_CORE", -5, -1, 5, v)
}

const DSP1: [(&str, &str); 5] = [
    ("PREADD9_L0", "PREADD9_CORE"),
    ("PREADD9_H0", "PREADD9_CORE"),
    ("MULT9_L0", "MULT9_CORE"),
    ("MULT9_H0", "MULT9_CORE"),
    ("MULT18_0", "MULT18_CORE"),
];
const DSP2: [(&str, &str); 5] = [
    ("PREADD9_L1", "PREADD9_CORE"),
    ("PREADD9_H1", "PREADD9_CORE"),
    ("MULT9_L1", "MULT9_CORE"),
    ("MULT9_H1", "MULT9_CORE"),
    ("MULT18_1", "MULT18_CORE"),
];
const DSP3: [(&str, &str); 6] = [
    ("REG18_L0_0", "REG18_CORE"),
    ("REG18_L0_1", "REG18_CORE"),
    ("REG18_L1_0", "REG18_CORE"),
    ("REG18_L1_1", "REG18_CORE"),
    ("MULT18X36_0", "MULT18X36_CORE"),
    ("ACC54_0", "ACC54_CORE"),
];
const DSP5: [(&str, &str); 5] = [
    ("PREADD9_L2", "PREADD9_CORE"),
    ("PREADD9_H2", "PREADD9_CORE"),
    ("MULT9_L2", "MULT9_CORE"),
    ("MULT9_H2", "MULT9_CORE"),
    ("MULT18_2", "MULT18_CORE"),
];
const DSP6: [(&str, &str); 5] = [
    ("PREADD9_L3", "PREADD9_CORE"),
    ("PREADD9_H3", "PREADD9_CORE"),
    ("MULT9_L3", "MULT9_CORE"),
    ("MULT9_H3", "MULT9_CORE"),
    ("MULT18_3", "MULT18_CORE"),
];
const DSP7: [(&str, &str); 7] = [
    ("REG18_H0_0", "REG18_CORE"),
    ("REG18_H0_1", "REG18_CORE"),
    ("REG18_H1_0", "REG18_CORE"),
    ("REG18_H1_1", "REG18_CORE"),
    ("MULT18X36_1", "MULT18X36_CORE"),
    ("ACC54_1", "ACC54_CORE"),
    ("MULT36", "MULT36_CORE"),
];

/// Bels of a tile type. `tb` is that tile type's bit database, which pins
/// of the IP-like bels are discovered from.
pub fn tile_bels(tiletype: &str, tb: &TileBits) -> Vec<Bel> {
    let mut stt = tiletype;
    if tiletype.ends_with("_EVEN") || tiletype.ends_with("_ODD") {
        stt = &tiletype[..tiletype.rfind('_').unwrap_or(tiletype.len())];
    }
    match stt {
        "PLC" => plc(),
        "SYSIO_B0_0" | "SYSIO_B1_0" | "SYSIO_B1_0_C" | "SYSIO_B2_0" | "SYSIO_B2_0_C" | "SYSIO_B6_0"
        | "SYSIO_B6_0_C" | "SYSIO_B7_0" | "SYSIO_B7_0_C" | "SYSIO_B0_0_15K" | "SYSIO_B1_0_15K" => {
            vec![seio33(0), seio33(1), iol(tb, true, 0), iol(tb, true, 1)]
        }
        "SYSIO_B1_DED" | "SYSIO_B1_DED_15K" => vec![seio33(1)],
        "SYSIO_B3_0" | "SYSIO_B3_0_DLY30_V18" | "SYSIO_B3_0_DQS1" | "SYSIO_B3_0_DQS3" | "SYSIO_B4_0"
        | "SYSIO_B4_0_DQS1" | "SYSIO_B4_0_DQS3" | "SYSIO_B4_0_DLY50" | "SYSIO_B4_0_DLY42" | "SYSIO_B5_0"
        | "SYSIO_B5_0_15K_DQS52" | "SYSIO_B4_0_15K_DQS42" | "SYSIO_B4_0_15K_BK4_V42"
        | "SYSIO_B4_0_15K_V31" | "SYSIO_B3_0_15K_DQS32" => {
            vec![seio18(0), seio18(1), diffio18(), iol(tb, false, 0), iol(tb, false, 1)]
        }
        "EFB_0" => config(tb, -2, "_CONFIG_HSE_CORE_CONFIG_HSE"),
        "EFB_15K" => config(tb, -14, "_CONFIG_CRE_CORE_CONFIG_HSE"),
        "EFB_1_OSC" | "OSC_15K" => vec![osc()],
        "EBR_1" => vec![ebr(tb, 0)],
        "EBR_4" => vec![ebr(tb, 1)],
        "EBR_7" => vec![ebr(tb, 2)],
        "EBR_9" => vec![ebr(tb, 3)],
        "DSP_R_1" | "DSP_L_1" => dsp(tb, &DSP1),
        "DSP_R_2" | "DSP_L_2" => dsp(tb, &DSP2),
        "DSP_R_3" | "DSP_L_3" => dsp(tb, &DSP3),
        "DSP_R_5" | "DSP_L_5" => dsp(tb, &DSP5),
        "DSP_R_6" | "DSP_L_6" => dsp(tb, &DSP6),
        "DSP_R_7" | "DSP_L_7" => dsp(tb, &DSP7),
        "CIB_T" => vec![vcc()],
        "LMID" | "LMID_RBB_5_15K" => (0..12).map(|z| dcc("L", z)).collect(),
        "RMID_DLY20" | "RMID_PICB_DLY10" => (0..12).map(|z| dcc("R", z)).collect(),
        "TMID_0" => (0..16).map(|z| dcc("T", z)).collect(),
        "CMUX_0" => {
            let mut v: Vec<Bel> = (0..4).map(|z| dcc("C", z)).collect();
            v.push(dcs());
            v
        }
        "GPLL_LLC" => vec![get_io_bel(tb, "PLL_LLC", "PLL_CORE", "_PLL_CORE_PLL_LMMI", 1, 0, 0)],
        "GPLL_ULC" => vec![get_io_bel(tb, "PLL_ULC", "PLL_CORE", "_PLL_CORE_PLL_LMMI", 0, 1, 0)],
        "GPLL_LRC" => vec![get_io_bel(tb, "PLL_LRC", "PLL_CORE", "_PLL_CORE_PLL_LMMI", -1, 0, 0)],
        "GPLL_LLC_15K" => vec![get_io_bel(tb, "PLL_LLC", "PLL_CORE", "_PLL_CORE_PLL_LMMI", 0, -1, 0)],
        "GPLL_LRC_15K" => vec![get_io_bel(tb, "PLL_LRC", "PLL_CORE", "_PLL_CORE_PLL_LMMI", 0, -1, 0)],
        "LRAM_0" => vec![get_io_bel(tb, "LRAM0", "LRAM_CORE", "_LRAM_CORE", -1, -5, 0)],
        "LRAM_1" => vec![get_io_bel(tb, "LRAM1", "LRAM_CORE", "_LRAM_CORE", -1, -1, 0)],
        "ECLK_0" => eclk(tb, 5),
        "BMID_0_ECLK_1" => {
            let mut v: Vec<Bel> = (0..18).map(|z| dcc("B", z)).collect();
            v.extend(eclk(tb, 4));
            v
        }
        "BMID_1_ECLK_2" => eclk(tb, 3),
        "SYSIO_B4_1_DQS0" | "SYSIO_B3_1_DQS0" => vec![dqsbuf(tb)],
        "DOSCL_P18_V18" => vec![get_io_bel(tb, "DDRDLL", "DDRDLL_CORE", "_DDRDLL_CORE_I1", 0, -1, 0)],
        "DDR_OSC_R" => vec![get_io_bel(tb, "DDRDLL", "DDRDLL_CORE", "_DDRDLL_CORE_I1", -1, 0, 0)],
        "LRAM_0_15K" => vec![get_io_bel(tb, "LRAM0", "LRAM_CORE", "_LRAM_CORE", -1, 0, 0)],
        "LRAM_1_15K" => vec![get_io_bel(tb, "LRAM1", "LRAM_CORE", "_LRAM_CORE", -1, 0, 0)],
        "LRAM_2_15K" => vec![get_io_bel(tb, "LRAM2", "LRAM_CORE", "_LRAM_CORE", 0, -1, 0)],
        "LRAM_3_15K" => vec![get_io_bel(tb, "LRAM3", "LRAM_CORE", "_LRAM_CORE", 0, -1, 0)],
        "LRAM_4_15K" => vec![get_io_bel(tb, "LRAM4", "LRAM_CORE", "_LRAM_CORE", 0, -1, 0)],
        "MIPI_DPHY_0" => vec![get_io_bel(tb, "TDPHY_CORE2", "DPHY_CORE", "_DPHY_CORE_DPHY0", -2, 0, 0)],
        "MIPI_DPHY_1" => vec![get_io_bel(tb, "TDPHY_CORE26", "DPHY_CORE", "_DPHY_CORE_DPHY0", -2, 0, 0)],
        _ => vec![],
    }
}

/// Indices (into `chip.tiles`) of the tiles a bel's configuration may be
/// split across. `t` is the index of the tile the bel was found in.
pub fn bel_tiles(chip: &Chip, t: usize, bel: &Bel) -> Result<Vec<usize>> {
    let tile = &chip.tiles[t];
    let at = |dx: i32, dy: i32| -> Result<(u32, u32)> {
        let x = tile.x as i64 + dx as i64;
        let y = tile.y as i64 + dy as i64;
        if x < 0 || y < 0 {
            return Err(err!("{}: offset ({dx},{dy}) leaves the grid", tile.name));
        }
        Ok((x as u32, y as u32))
    };
    let find = |dx: i32, dy: i32, ok: &dyn Fn(&str) -> bool, what: &str| -> Result<usize> {
        let (x, y) = at(dx, dy)?;
        chip.tiles_at(x, y)
            .iter()
            .copied()
            .find(|&i| ok(&chip.tiles[i].tiletype))
            .ok_or_else(|| err!("{}: no tile {what} at ({x},{y}) for bel {}", tile.name, bel.name))
    };
    let rel = |dx: i32, dy: i32, tt: &str| find(dx, dy, &|s| s == tt, tt);
    let rel_prefix = |dx: i32, dy: i32, p: &str| find(dx, dy, &|s| s.starts_with(p), p);
    let rel_suffix = |dx: i32, dy: i32, p: &str| find(dx, dy, &|s| s.ends_with(p), p);

    let tt = tile.tiletype.as_str();
    Ok(match bel.beltype.as_str() {
        "SEIO33_CORE" | "SIOLOGIC" => match tt {
            "SYSIO_B1_0_C" => vec![t, rel(0, 1, "SYSIO_B1_0_REM")?],
            "SYSIO_B2_0_C" => vec![t, rel(0, 1, "SYSIO_B2_0_REM")?],
            "SYSIO_B6_0_C" => vec![t, rel(0, 1, "SYSIO_B6_0_REM")?],
            "SYSIO_B7_0_C" => vec![t, rel(0, 1, "SYSIO_B7_0_REM")?],
            "SYSIO_B1_0_15K" => match tile.name.as_str() {
                "CIB_R9C75:SYSIO_B1_0_15K" => vec![t, rel(0, 1, "RMID_PICB_DLY10")?],
                _ => vec![t, rel(0, 1, "SYSIO_B1_1_15K")?],
            },
            _ => vec![t],
        },
        "SEIO18_CORE" | "DIFF18_CORE" | "IOLOGIC" => vec![t, rel_prefix(1, 0, "SYSIO")?],
        "OXIDE_EBR" => match bel.z {
            0 => vec![rel(0, 0, "EBR_1")?, rel(1, 0, "EBR_2")?],
            1 => vec![rel(0, 0, "EBR_4")?, rel(1, 0, "EBR_5")?],
            2 => vec![rel(0, 0, "EBR_7")?, rel(1, 0, "EBR_8")?],
            3 => vec![rel(0, 0, "EBR_9")?, rel_suffix(1, 0, "EBR_10")?],
            z => return Err(err!("{}: EBR z {z}", tile.name)),
        },
        "PREADD9_CORE" | "MULT9_CORE" | "MULT18_CORE" | "REG18_CORE" | "MULT18X36_CORE" | "MULT36_CORE"
        | "ACC54_CORE" => {
            let parts: Vec<&str> = tt.split('_').collect();
            let offset: i32 = parts
                .get(2)
                .and_then(|s| s.parse().ok())
                .ok_or_else(|| err!("bad DSP tile {tt}"))?;
            let range = match parts.get(1) {
                Some(&"L") => (0..11, "L"),
                Some(&"R") => (1..12, "R"),
                _ => return Err(err!("bad DSP tile {tt}")),
            };
            range.0.map(|x| rel(x - offset, 0, &format!("DSP_{}_{x}", range.1))).collect::<Result<_>>()?
        }
        "DCC" => match bel.name.get(..5) {
            Some("DCC_T") => match bel.z {
                0..=7 => vec![t],
                8..=15 => vec![t, rel_prefix(1, 0, "TMID_1")?],
                _ => return Err(err!("bad DCC_T tile {tt}")),
            },
            Some("DCC_B") => match bel.z {
                0..=8 => vec![t],
                9..=17 => vec![t, rel(1, 0, "BMID_1_ECLK_2")?],
                _ => return Err(err!("bad DCC_B tile {tt}")),
            },
            _ => vec![t],
        },
        "CONFIG_MULTIBOOT_CORE" | "CONFIG_LMMI_CORE" | "CONFIG_CLKRST_CORE" => match tt {
            "EFB_0" => vec![
                t,
                rel_prefix(2, 0, "EFB_1_OSC")?,
                rel_prefix(4, 0, "EFB_2")?,
                rel_prefix(6, 0, "I2C_EFB_3")?,
            ],
            _ => vec![t],
        },
        "ECLKSYNC_CORE" | "ECLKDIV_CORE" => match tt {
            "ECLK_0" => vec![rel(1, 0, "BMID_0_ECLK_1")?, rel(2, 0, "BMID_1_ECLK_2")?],
            "BMID_0_ECLK_1" => vec![rel(0, 0, "BMID_0_ECLK_1")?, rel(1, 0, "BMID_1_ECLK_2")?],
            "BMID_1_ECLK_2" => vec![rel(-1, 0, "BMID_0_ECLK_1")?, rel(0, 0, "BMID_1_ECLK_2")?],
            _ => return Err(err!("bad ECLK tile {tt}")),
        },
        "DQSBUF_CORE" => match tt {
            "SYSIO_B3_1_DQS0" => vec![
                rel(0, 0, "SYSIO_B3_1_DQS0")?,
                rel(1, 0, "SYSIO_B3_0_DQS1")?,
                rel(2, 0, "SYSIO_B3_1_DQS2")?,
                rel(3, 0, "SYSIO_B3_0_DQS3")?,
                rel(4, 0, "SYSIO_B3_1_DQS4")?,
            ],
            "SYSIO_B4_1_DQS0" => vec![
                rel(0, 0, "SYSIO_B4_1_DQS0")?,
                rel(1, 0, "SYSIO_B4_0_DQS1")?,
                rel(2, 0, "SYSIO_B4_1_DQS2")?,
                rel(3, 0, "SYSIO_B4_0_DQS3")?,
                rel(4, 0, "SYSIO_B4_1_DQS4")?,
            ],
            _ => return Err(err!("bad DQSBUF tile {tt}")),
        },
        _ => vec![t],
    })
}
