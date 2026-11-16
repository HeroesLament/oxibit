//! I/O pads, buffers, IOLOGIC and IDELAYCTRL (nextpnr-xilinx `pack_io`,
//! `decompose_iob`, `pack_iologic`, `pack_idelayctrl`), high-range (IOB33)
//! banks only: the xc7z020 has no others.
//!
//! Each top-level port bit becomes a `PAD` cell on the package pin the XDC
//! names, carrying the port's XDC properties. The netlist's I/O buffer on
//! that pad is split into the site's bel-sized pieces, all fixed at packing:
//! an input buffer on `INBUF_EN`, an output buffer on `OUTBUF`, and for a
//! differential output the P buffer on the master site, an inverter and the
//! N buffer on the slave. An ODDR goes on the OLOGIC beside the pad it
//! drives, an IDELAYE2 on the IDELAY beside the pad that feeds it, and the
//! IDELAYCTRL is duplicated into each clock region holding a delay.

use std::collections::{BTreeMap, BTreeSet};

use serde_json::{Value, json};

use super::{Packer, Rule};
use crate::device::Device;
use crate::netlist::{CellId, Dir, NetId};
use crate::xdc::PortProps;

/// Netlist buffer pins that sit on a package pad.
fn pad_pins(ty: &str) -> &'static [&'static str] {
    match ty {
        "IBUF" | "IBUF_IBUFDISABLE" | "IBUF_INTERMDISABLE" => &["I"],
        "IBUFDS" | "IBUFGDS" | "IBUFDS_INTERMDISABLE" => &["I", "IB"],
        "IOBUF" | "IOBUF_DCIEN" | "IOBUF_INTERMDISABLE" => &["IO"],
        "IOBUFDS" | "IOBUFDS_DCIEN" => &["IO", "IOB"],
        "OBUF" | "OBUFT" => &["O"],
        "OBUFDS" | "OBUFTDS" => &["O", "OB"],
        _ => &[],
    }
}

fn add_cell(p: &mut Packer, name: &str, ty: &str, ports: &[(&str, Dir, Option<NetId>)]) -> CellId {
    let c = p.nl.add_cell(name, ty);
    for (port, d, n) in ports {
        p.nl.add_port(c, port, *d);
        if let Some(n) = n {
            p.nl.connect(c, port, *n);
        }
    }
    c
}

pub fn pack_io(p: &mut Packer, dev: &Device, props: &PortProps) -> Result<(), String> {
    // PAD cells, and which buffer each pad belongs to.
    let mut pad_site: BTreeMap<NetId, String> = BTreeMap::new();
    let mut bufs: Vec<CellId> = Vec::new();
    let ports = p.nl.ports.clone();
    for tp in &ports {
        let Some(net) = tp.net else { continue };
        let on_pad = |p: &Packer, c: CellId, port: &str| pad_pins(&p.nl.cell(c).ty).contains(&port);
        let buf = match tp.dir {
            Dir::Input => p.nl.net(net).users.iter().find(|(c, port)| on_pad(p, *c, port)).map(|(c, _)| *c),
            Dir::Output => p.nl.driver(net).filter(|(c, port)| on_pad(p, *c, port)).map(|(c, _)| c),
            Dir::Inout => None,
        };
        let Some(buf) = buf else {
            return Err(format!("port {} has no I/O buffer on its pad", tp.name));
        };
        let pr = props.get(&tp.name).cloned().unwrap_or_default();
        let pin = pr
            .get("PACKAGE_PIN")
            .or_else(|| pr.get("LOC"))
            .ok_or_else(|| format!("port {} has no PACKAGE_PIN", tp.name))?;
        let site = dev
            .pins
            .get(pin)
            .ok_or_else(|| format!("port {}: package pin {pin} not in this part", tp.name))?
            .clone();
        let site_ty = dev
            .site_type(&site)
            .ok_or_else(|| format!("site {site} not in the tilegrid"))?
            .to_string();
        let pad = add_cell(p, &tp.name, "PAD", &[("PAD", Dir::Inout, Some(net))]);
        let cell = p.nl.cell_mut(pad);
        for (k, v) in &pr {
            cell.attrs.insert(k.clone(), json!(v));
        }
        cell.attrs.insert("LOC".into(), json!(pin));
        cell.attrs
            .insert("X_IO_DIR".into(), json!(if tp.dir == Dir::Input { "IN" } else { "OUT" }));
        cell.bel = dev.grid_loc(&site, &format!("{site_ty}.PAD"));
        pad_site.insert(net, site);
        if !bufs.contains(&buf) {
            bufs.push(buf);
        }
    }

    // Split each buffer into its site pieces.
    let site_of = |n: Option<NetId>| -> Result<String, String> {
        n.and_then(|n| pad_site.get(&n).cloned())
            .ok_or_else(|| "buffer pin not on a pad".to_string())
    };
    for b in bufs {
        let ty = p.nl.cell(b).ty.clone();
        let name = p.nl.cell(b).name.clone();
        let get = |p: &Packer, port: &str| p.nl.cell(b).net(port);
        match ty.as_str() {
            "IBUF" => {
                let (i, o) = (get(p, "I"), get(p, "O"));
                let site = site_of(i)?;
                p.nl.remove_cell(b);
                let c = add_cell(p, &format!("{name}$intcell$IBUF"), "IBUF", &[("I", Dir::Input, i), ("O", Dir::Output, o)]);
                p.nl.cell_mut(c).bel = dev.grid_loc(&site, "IOB33.INBUF_EN");
                p.io_site.insert(c, site);
            }
            "OBUF" => {
                let (i, o) = (get(p, "I"), get(p, "O"));
                let site = site_of(o)?;
                p.nl.remove_cell(b);
                let c = add_cell(p, &format!("{name}$intcell$OBUF"), "OBUF", &[("I", Dir::Input, i), ("O", Dir::Output, o)]);
                p.nl.cell_mut(c).bel = dev.grid_loc(&site, "IOB33.OUTBUF");
                p.io_site.insert(c, site);
            }
            "IBUFDS" | "IBUFGDS" => {
                let (i, ib, o) = (get(p, "I"), get(p, "IB"), get(p, "O"));
                let site = site_of(i)?;
                p.nl.remove_cell(b);
                let c = add_cell(
                    p,
                    &format!("{name}$intcell$IBUF"),
                    "IBUFDS",
                    &[("I", Dir::Input, i), ("IB", Dir::Input, ib), ("O", Dir::Output, o)],
                );
                let cell = p.nl.cell_mut(c);
                cell.bel = dev.grid_loc(&site, "IOB33M.INBUF_EN");
                cell.attrs.insert("X_IOB_SITE_TYPE".into(), json!("IOB33M"));
                p.io_site.insert(c, site);
            }
            "OBUFDS" => {
                let (i, o, ob) = (get(p, "I"), get(p, "O"), get(p, "OB"));
                let (sp, sn) = (site_of(o)?, site_of(ob)?);
                p.nl.remove_cell(b);
                let inv_o = p.nl.add_net_unique(&format!("{name}$subnet$I_B"));
                let inv = add_cell(
                    p,
                    &format!("{name}$subcell$INV"),
                    "INV",
                    &[("I", Dir::Input, i), ("O", Dir::Output, Some(inv_o))],
                );
                let cell = p.nl.cell_mut(inv);
                cell.bel = dev.grid_loc(&sn, "IOB33S.O_ININV");
                cell.attrs.insert("X_IOB_SITE_TYPE".into(), json!("IOB33S"));
                let bp = add_cell(p, &format!("{name}$subcell$P"), "OBUF", &[("I", Dir::Input, i), ("O", Dir::Output, o)]);
                let cell = p.nl.cell_mut(bp);
                cell.bel = dev.grid_loc(&sp, "IOB33M.OUTBUF");
                cell.attrs.insert("X_IOB_SITE_TYPE".into(), json!("IOB33M"));
                p.io_site.insert(bp, sp.clone());
                let bn = add_cell(
                    p,
                    &format!("{name}$subcell$N"),
                    "OBUF",
                    &[("I", Dir::Input, Some(inv_o)), ("O", Dir::Output, ob)],
                );
                let cell = p.nl.cell_mut(bn);
                cell.bel = dev.grid_loc(&sn, "IOB33S.OUTBUF");
                cell.attrs.insert("X_IOB_SITE_TYPE".into(), json!("IOB33S"));
            }
            t => return Err(format!("I/O buffer {name}: {t} not packed yet")),
        }
    }

    // Buffers onto their bel types.
    let ob = Rule::to("IOB33_OUTBUF").port("I", "IN").port("O", "OUT").port("T", "TRI");
    let ib = Rule::to("IOB33_INBUF_EN").port("I", "PAD").port("O", "OUT");
    let rules: BTreeMap<String, Rule> = [
        ("OBUF".to_string(), ob.clone()),
        ("OBUFT".to_string(), ob),
        ("IBUF".to_string(), ib.clone()),
        ("IBUFDS".to_string(), ib.port("IB", "DIFFI_IN")),
    ]
    .into_iter()
    .collect();
    for c in p.nl.cell_ids() {
        let cell = p.nl.cell(c);
        if cell.bel.is_none() {
            continue;
        }
        if let Some(r) = rules.get(&cell.ty).cloned() {
            p.xform(c, &r);
        }
    }
    let rules: BTreeMap<String, Rule> = [
        ("PAD".to_string(), Rule::to("PAD")),
        ("INV".to_string(), Rule::to("INVERTER").port("I", "IN").port("O", "OUT")),
    ]
    .into_iter()
    .collect();
    p.generic_xform(&rules);
    for c in p.nl.cell_ids() {
        let cell = p.nl.cell_mut(c);
        if !cell.ty.starts_with("IOB33") {
            continue;
        }
        if let Some(Value::String(st)) = cell.attrs.get("X_IOB_SITE_TYPE").cloned() {
            cell.ty = format!("{st}{}", &cell.ty[5..]);
        }
    }
    p.count("io.pads", pad_site.len());
    Ok(())
}

/// ODDR onto the OLOGIC beside its pad; IDELAYE2 onto the IDELAY beside the
/// pad feeding it. Other IOLOGIC primitives are refused.
pub fn pack_iologic(p: &mut Packer, dev: &Device) -> Result<(), String> {
    for c in p.nl.cell_ids() {
        let ty = p.nl.cell(c).ty.clone();
        let name = p.nl.cell(c).name.clone();
        match ty.as_str() {
            "IDELAYE2" => {
                let d = p.nl.cell(c).net("IDATAIN").ok_or_else(|| format!("{name}: IDATAIN unconnected"))?;
                let (drv, _) = p.nl.driver(d).ok_or_else(|| format!("{name}: IDATAIN undriven"))?;
                if !p.nl.cell(drv).ty.contains("INBUF_EN") {
                    return Err(format!("{name}: IDATAIN from {}", p.nl.cell(drv).ty));
                }
                let io_loc = p.nl.cell(drv).bel.clone().ok_or("input buffer unplaced")?;
                let io_site = p.io_site.get(&drv).cloned().ok_or("input buffer site")?;
                let site = dev.io_logic_site(&io_site, "IDELAY").ok_or_else(|| format!("no IDELAY for {io_site}"))?;
                let cell = p.nl.cell_mut(c);
                cell.bel = dev.grid_loc(&site, "IDELAYE2");
                cell.attrs.insert("X_IO_BEL".into(), json!(io_loc.text()));
                p.io_site.insert(c, io_site);
            }
            "ODDR" => {
                let q = p.nl.cell(c).net("Q").ok_or_else(|| format!("{name}: Q unconnected"))?;
                let users = p.nl.net(q).users.clone();
                let outs: Vec<CellId> = users
                    .iter()
                    .filter(|(u, _)| matches!(p.nl.cell(*u).ty.as_str(), "IOB33_OUTBUF" | "IOB33M_OUTBUF"))
                    .map(|(u, _)| *u)
                    .collect();
                let [ob] = outs[..] else {
                    return Err(format!("{name}: Q must drive one output buffer"));
                };
                let io_site = p.io_site.get(&ob).cloned().ok_or("output buffer site")?;
                let site = dev.io_logic_site(&io_site, "OLOGIC").ok_or_else(|| format!("no OLOGIC for {io_site}"))?;
                let tri = users.first().is_some_and(|(_, port)| port == "TRI");
                let rule = Rule::to(if tri { "OLOGICE3_TFF" } else { "OLOGICE3_OUTFF" })
                    .port("C", "CK")
                    .port("S", "SR")
                    .port("R", "SR");
                p.xform(c, &rule);
                p.nl.cell_mut(c).bel = dev.grid_loc(&site, if tri { "TFF" } else { "OUTFF" });
            }
            "IDDR" | "ISERDESE2" | "OSERDESE2" | "ODELAYE2" => return Err(format!("{name}: {ty} not packed yet")),
            _ => {}
        }
    }
    let rules: BTreeMap<String, Rule> = [("IDELAYE2".to_string(), Rule::to("IDELAYE2_IDELAYE2"))].into_iter().collect();
    let n = p.generic_xform(&rules);
    p.count("io.idelays", n);
    Ok(())
}

/// One IDELAYCTRL per clock region holding a delay; their RDY outputs ANDed
/// when there are several.
pub fn pack_idelayctrl(p: &mut Packer, dev: &Device) -> Result<(), String> {
    let ctrls = p.nl.cells_of("IDELAYCTRL");
    let delays: Vec<CellId> = p.nl.cells_of("IDELAYE2_IDELAYE2");
    let Some(&ctrl) = ctrls.first() else {
        if !delays.is_empty() {
            return Err("IDELAYE2 without an IDELAYCTRL: its taps never calibrate".into());
        }
        return Ok(());
    };
    if ctrls.len() > 1 {
        return Err("more than one IDELAYCTRL".into());
    }
    let mut sites: BTreeSet<String> = BTreeSet::new();
    for d in &delays {
        let io = p.io_site.get(d).ok_or("delay without its IOB site")?;
        sites.insert(dev.ioctrl_site(io).ok_or_else(|| format!("no IDELAYCTRL for {io}"))?);
    }
    if sites.is_empty() {
        return Err("IDELAYCTRL with no delays".into());
    }
    let name = p.nl.cell(ctrl).name.clone();
    let (refclk, rst, rdy) = (p.nl.cell(ctrl).net("REFCLK"), p.nl.cell(ctrl).net("RST"), p.nl.cell(ctrl).net("RDY"));
    p.nl.remove_cell(ctrl);
    let mut rdys = Vec::new();
    let single = sites.len() == 1;
    for (i, site) in sites.iter().enumerate() {
        let r = match rdy {
            Some(out) if single => Some(out),
            Some(_) => Some(p.nl.add_net_unique(&format!("{name}$intnet$CTRL_DUP_{i}_RDY"))),
            None => None,
        };
        let c = add_cell(
            p,
            &format!("{name}$intcell$CTRL_DUP_{i}"),
            "IDELAYCTRL",
            &[("REFCLK", Dir::Input, refclk), ("RST", Dir::Input, rst), ("RDY", Dir::Output, r)],
        );
        p.nl.cell_mut(c).bel = dev.grid_loc(site, "IDELAYCTRL");
        if let Some(r) = r {
            rdys.push(r);
        }
    }
    if let Some(out) = rdy
        && rdys.len() > 1
    {
        let mut acc = rdys[0];
        for j in 1..rdys.len() {
            let o = if j == rdys.len() - 1 {
                out
            } else {
                p.nl.add_net_unique(&format!("{name}$intnet$ANDED_RDY_{j}"))
            };
            p.create_lut(&format!("{name}/RDY_AND_LUT_{j}"), &[acc, rdys[j]], Some(o), super::constants::prop32(8));
            acc = o;
        }
    }
    let rules: BTreeMap<String, Rule> = [("IDELAYCTRL".to_string(), Rule::to("IDELAYCTRL_IDELAYCTRL"))].into_iter().collect();
    p.generic_xform(&rules);
    p.count("io.idelayctrls", sites.len());
    Ok(())
}
