//! Port properties from an XDC file: `set_property KEY VALUE [get_ports
//! NAME]` and `set_property -dict {KEY VALUE ...} [get_ports NAME]`. Other
//! commands (clocks, timing) are not port properties and are skipped.

use std::collections::BTreeMap;

/// Port name -> property -> value.
pub type PortProps = BTreeMap<String, BTreeMap<String, String>>;

fn unbrace(s: &str) -> &str {
    s.trim().trim_start_matches('{').trim_end_matches('}').trim()
}

pub fn parse(text: &str) -> PortProps {
    let mut out = PortProps::new();
    for raw in text.lines() {
        let line = raw.split('#').next().unwrap_or("").trim();
        let Some(rest) = line.strip_prefix("set_property") else { continue };
        let Some(gp) = rest.find("[get_ports") else { continue };
        let target = rest[gp + "[get_ports".len()..].trim().trim_end_matches(']').trim();
        let ports: Vec<String> = unbrace(target).split_whitespace().map(|p| unbrace(p).to_string()).collect();
        let head = rest[..gp].trim();
        let mut props: Vec<(String, String)> = Vec::new();
        if let Some(d) = head.strip_prefix("-dict") {
            let toks: Vec<&str> = unbrace(d).split_whitespace().collect();
            for kv in toks.chunks(2) {
                if let [k, v] = kv {
                    props.push((k.to_string(), v.to_string()));
                }
            }
        } else {
            let mut it = head.split_whitespace();
            if let (Some(k), Some(v)) = (it.next(), it.next()) {
                props.push((k.to_string(), unbrace(v).to_string()));
            }
        }
        for p in ports {
            let e = out.entry(p).or_default();
            for (k, v) in &props {
                e.insert(k.clone(), v.clone());
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn both_forms() {
        let t = "set_property PACKAGE_PIN R14 [get_ports ad9363_spi_clk]\n\
                 set_property IOSTANDARD LVCMOS25 [get_ports {ad9363_spi_clk}]\n\
                 set_property -dict {PACKAGE_PIN N20 IOSTANDARD LVDS_25} [get_ports {clk_p}]\n\
                 create_clock -period 10.000 [get_nets axi_clk]\n";
        let p = parse(t);
        assert_eq!(p["ad9363_spi_clk"]["PACKAGE_PIN"], "R14");
        assert_eq!(p["ad9363_spi_clk"]["IOSTANDARD"], "LVCMOS25");
        assert_eq!(p["clk_p"]["IOSTANDARD"], "LVDS_25");
        assert_eq!(p.len(), 2);
    }
}
