#!/bin/bash
# Co-simulate one memory case from the memcases example: yosys's reading of
# the $mem_v2 (ref.json) against the RAM mapping (dut.json), block RAMs on
# our RAMB models (ramb.v).
#   cosim.sh CASEDIR [CYCLES]
# Needs yosys and iverilog. YOSYS / YOSYS_SHARE override the yosys binary and
# its share directory; LUTRAM_SIM names the cut-down distributed RAM models
# (made from cells_sim.v on first use).
# Distributed RAM cases use yosys's cells_sim.v models.
# Reads only for the first min(4 x depth, 4000) cycles (initial contents),
# then random writes and reads over the whole address space; a read is compared
# bit by bit wherever the reference is not x. Prints PASS/FAIL per case.
set -u
dir=$1
cycles=${2:-20000}
here=$(cd "$(dirname "$0")" && pwd)
Y=${YOSYS:-yosys}
cd "$dir" || exit 1
read -r width depth abits two sync < case.txt

$Y -q -p "read_json ref.json; rename top ref_top; write_verilog -noattr ref.v" || exit 1
$Y -q -p "read_json dut.json; rename top dut_top; write_verilog -noattr dut.v" || exit 1

# Distributed RAM models: yosys's, cut out of cells_sim.v (its RAMB
# modules are stubs and would clash with ramb.v).
lutsim=${LUTRAM_SIM:-$PWD/lutram_sim.v}
if [ ! -s "$lutsim" ]; then
  share=${YOSYS_SHARE:-$("$(dirname "$(command -v "$Y")")/yosys-config" --datdir)}
  awk '/^module (RAM32M|RAM64M|RAM128X1D)[ (]/,/^endmodule/' "$share/xilinx/cells_sim.v" > "$lutsim"
fi

# Random write data: one \$random per 32 bits.
words=$(( (width + 31) / 32 ))
rnd=$(for ((k = 0; k < words; k++)); do printf '$random(seed),'; done)
rnd="{${rnd%,}}"
wclk_port=""
wclk_decl=""
wclk_gen=""
if [ "$two" = 1 ]; then
  wclk_port=".wclk(wclk),"
  wclk_decl="reg wclk = 0;"
  wclk_gen="always #7 wclk = ~wclk;"
fi
cat > tb.v <<TB
\`timescale 1ns/1ps
module tb;
  reg rclk = 0;
  $wclk_decl
  reg [$((abits - 1)):0] raddr = 0, waddr = 0;
  reg [$((width - 1)):0] wdata = 0;
  reg we = 0;
  wire [$((width - 1)):0] r_ref, r_dut;
  ref_top R(.rclk(rclk), $wclk_port .raddr(raddr), .waddr(waddr), .wdata(wdata), .we(we), .rdata(r_ref));
  dut_top D(.rclk(rclk), $wclk_port .raddr(raddr), .waddr(waddr), .wdata(wdata), .we(we), .rdata(r_dut));
  always #5 rclk = ~rclk;
  $wclk_gen
  integer cyc, i, bad, checked, seed;
  initial begin
    bad = 0; checked = 0; seed = 1;
    for (cyc = 0; cyc < $cycles; cyc = cyc + 1) begin
      @(negedge rclk);
      if (cyc > 1) begin
        for (i = 0; i < $width; i = i + 1)
          if (r_ref[i] !== 1'bx) begin
            checked = checked + 1;
            if (r_ref[i] !== r_dut[i]) begin
              bad = bad + 1;
              if (bad <= 5) \$display("MISMATCH cyc=%0d bit=%0d ref=%b dut=%b raddr=%0d", cyc, i, r_ref[i], r_dut[i], raddr);
            end
          end
      end
      raddr = \$random(seed);
      waddr = \$random(seed);
      wdata = $rnd;
      // Reads only for the first cycles, so initial contents get checked.
      we = cyc < $(( depth * 4 < 4000 ? depth * 4 : 4000 )) ? 1'b0 : \$random(seed);
    end
    if (bad == 0) \$display("PASS checked=%0d", checked);
    else \$display("FAIL bad=%0d checked=%0d", bad, checked);
    \$finish;
  end
endmodule
TB
iverilog -g2005 -o sim tb.v ref.v dut.v "$here/ramb.v" "$lutsim" > iverilog.log 2>&1 || { echo "$(basename "$dir") COMPILE $(head -2 iverilog.log)"; exit 1; }
res=$(vvp -n sim | grep -E "PASS|FAIL|MISMATCH" | tail -6)
echo "$(basename "$dir") ${width}x${depth} sync=$sync two=$two $(echo "$res" | tail -1)"
echo "$res" | grep MISMATCH | head -3
