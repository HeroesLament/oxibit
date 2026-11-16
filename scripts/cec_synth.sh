#!/usr/bin/env bash
# Check an oxibit-synth checking view against its source, register by
# register: every flop and memory is cut by name, and oxibit-cec proves the
# combinational logic between the cuts equal.
#
#   scripts/cec_synth.sh SRC.v OUT.check.json TOP WORKDIR [JOBS]
#
# YOSYS (default yosys) builds the miter; CELLS (default: yosys's
# share/yosys/xilinx) gives cells_sim.v, whose models turn the gate
# netlist's primitives into logic, and cells_xtra.v, whose primitives stay
# black boxes on both sides (PS7, MMCME2_ADV) and are cut at their ports.
#
# Each side is flattened, its flops turned into a Q input and a D output
# named after the Q wire (expose -evert-dff), and each memory into a black
# box named after the memory, cut the same way (expose -evert). miter
# -equiv then pairs the two sides' ports by name. A port on one side only
# (a register or memory the other side lacks) is listed in
# WORKDIR/unmatched.txt: an input gets a stand-in that nothing reads on the
# other side, an output is not compared. The miter goes to oxibit-cec as
# AIGER, one cmp_<port> output per compared output; the verdict lines land
# in WORKDIR/result.txt and the FAIL / OPEN ones in WORKDIR/verdict.txt.
# Exits 0 when every compared output is proven.
set -euo pipefail

src=$1
gate=$2
top=$3
work=$4
jobs=${5:-$(sysctl -n hw.ncpu 2>/dev/null || nproc)}
yosys=${YOSYS:-yosys}
cells=${CELLS:-$(dirname "$(command -v "$yosys")")/../share/yosys/xilinx}
here=$(cd "$(dirname "$0")/.." && pwd)

mkdir -p "$work"

# $mem_v2 -> an instance of a black box per memory shape, keeping the cell
# name (the memory's name), so expose -evert cuts it into named ports.
cat > "$work/memcut.v" <<'EOF'
(* techmap_celltype = "$mem_v2" *)
module _memcut_ (RD_CLK, RD_EN, RD_ARST, RD_SRST, RD_ADDR, RD_DATA,
                 WR_CLK, WR_EN, WR_ADDR, WR_DATA);
    parameter MEMID = "";
    parameter WIDTH = 1;
    parameter ABITS = 1;
    parameter RD_PORTS = 1;
    parameter WR_PORTS = 1;
    parameter SIZE = 1;
    parameter OFFSET = 0;
    parameter INIT = 0;
    parameter RD_WIDE_CONTINUATION = 0;
    parameter RD_CLK_ENABLE = 0;
    parameter RD_CLK_POLARITY = 0;
    parameter RD_TRANSPARENCY_MASK = 0;
    parameter RD_COLLISION_X_MASK = 0;
    parameter RD_CE_OVER_SRST = 0;
    parameter RD_INIT_VALUE = 0;
    parameter RD_ARST_VALUE = 0;
    parameter RD_SRST_VALUE = 0;
    parameter WR_WIDE_CONTINUATION = 0;
    parameter WR_CLK_ENABLE = 0;
    parameter WR_CLK_POLARITY = 0;
    parameter WR_PRIORITY_MASK = 0;
    input [RD_PORTS-1:0] RD_CLK, RD_EN, RD_ARST, RD_SRST;
    input [RD_PORTS*ABITS-1:0] RD_ADDR;
    output [RD_PORTS*WIDTH-1:0] RD_DATA;
    input [WR_PORTS-1:0] WR_CLK;
    input [WR_PORTS*WIDTH-1:0] WR_EN;
    input [WR_PORTS*ABITS-1:0] WR_ADDR;
    input [WR_PORTS*WIDTH-1:0] WR_DATA;
    // A write port's address and data matter only while it writes: compare
    // them masked by its enables.
    wire [WR_PORTS*ABITS-1:0] wr_addr;
    genvar i;
    generate for (i = 0; i < WR_PORTS; i = i + 1) begin : wr
        assign wr_addr[i*ABITS +: ABITS] = WR_ADDR[i*ABITS +: ABITS] & {ABITS{|WR_EN[i*WIDTH +: WIDTH]}};
    end endgenerate
    _cec_mem_ #(.RD_PORTS(RD_PORTS), .WR_PORTS(WR_PORTS), .WIDTH(WIDTH), .ABITS(ABITS))
        _TECHMAP_REPLACE_ (.RD_EN(RD_EN), .RD_ARST(RD_ARST), .RD_SRST(RD_SRST),
        .RD_ADDR(RD_ADDR), .RD_DATA(RD_DATA), .WR_EN(WR_EN), .WR_ADDR(wr_addr),
        .WR_DATA(WR_DATA & WR_EN));
endmodule
EOF
cat > "$work/memcut_bb.v" <<'EOF'
(* blackbox *)
module _cec_mem_ (RD_EN, RD_ARST, RD_SRST, RD_ADDR, RD_DATA, WR_EN, WR_ADDR, WR_DATA);
    parameter RD_PORTS = 1;
    parameter WR_PORTS = 1;
    parameter WIDTH = 1;
    parameter ABITS = 1;
    input [RD_PORTS-1:0] RD_EN, RD_ARST, RD_SRST;
    input [RD_PORTS*ABITS-1:0] RD_ADDR;
    output [RD_PORTS*WIDTH-1:0] RD_DATA;
    input [WR_PORTS*WIDTH-1:0] WR_EN;
    input [WR_PORTS*ABITS-1:0] WR_ADDR;
    input [WR_PORTS*WIDTH-1:0] WR_DATA;
endmodule
EOF

# Elaborate one side ($1: gold | gate, $2: the read command, $3: commands
# run after proc, $4: after memory collection) and cut it.
prepare() {
    local side=$1 read=$2 after_proc=$3 after_mem=$4
    "$yosys" -q -l "$work/$side.log" -p "
        read_verilog -lib $cells/cells_xtra.v
        read_verilog $cells/cells_sim.v
        read_verilog -lib $work/memcut_bb.v
        $read
        hierarchy -check -top $top
        proc
        $after_proc
        flatten -wb
        rename -hide w:*.*
        memory -nomap
        $after_mem
        opt_clean -purge
        techmap -map $work/memcut.v t:\$mem_v2
        hierarchy -top $top
        async2sync
        dffunmap
        expose -evert-dff
        expose -evert t:* t:\$* %d
        opt_clean -purge
        rename $top $side
        write_rtlil $work/$side.il
        blackbox $side
        write_json $work/$side.ports.json
    " > /dev/null
}

# Name the gate's memory cells after their memories, as yosys's memory
# collection names the gold's, and mark each flop's register wire keep:
# oxibit names a flop oxsyn_ffN$REG[BIT], and its Q net carries REG among
# other names (aliases the source assigns from the register). keep, with
# opt_clean -purge, leaves REG as the one name expose cuts at.
cat > "$work/gate.jq" <<'EOF'
.modules[] |= (
    ([.cells | to_entries[] | select(.value.type | test("^FD[RSCP]E$"))
      | .key | capture("^oxsyn_ff[0-9]+[$](?<r>.*)\\[[0-9]+\\]$") | {(.r): true}] | add // {}) as $regs
    | .netnames |= with_entries(
        if $regs[.key] then .value.attributes.keep = "00000000000000000000000000000001" else . end)
    | .cells |= with_entries(
        if .value.type == "$mem_v2" then .key = (.value.parameters.MEMID | ltrimstr("\\")) else . end))
EOF
jq -c -f "$work/gate.jq" "$gate" > "$work/gate.json"

# The gate's memories, by name, are cut as black boxes on both sides; the
# source's other memories are the ones oxibit turned into registers, which
# memory_map does the same way. A memory oxibit merged write ports of
# (attribute oxibit_merged_writes) has fewer ports than the source's, so
# both sides compare it as memory_map's registers.
gate_mems=$(jq -r '[.modules[].cells[] | select(.type == "$mem_v2" and (.attributes.oxibit_merged_writes == null))
    | "c:" + (.parameters.MEMID | ltrimstr("\\"))] | join(" ")' "$work/gate.json")

# The source's flops keep their reg names: proc's flop drives the reg's own
# wire, and keep makes opt_clean choose that name over the aliases the
# source assigns from it (EHDL's _eq_N and the like). The gate's flops are
# named after the reg already (oxibit names the Q net after it).
keep_regs='select -set ffq t:$*dff* %co:+[Q] w:* %i; setattr -set keep 1 @ffq'
map_mems="select -set gatemems $gate_mems; memory_map t:\$mem_v2 @gatemems %d; $keep_regs"

# CEC_REUSE=1 keeps both sides from an earlier run in WORKDIR (for work on
# the miter steps below).
if [ "${CEC_REUSE:-0}" = 1 ] && [ -s "$work/gold.il" ] && [ -s "$work/gate.il" ]; then
    echo "reusing $work/gold.il and $work/gate.il"
else
    prepare gold "read_verilog -sv $src" "$keep_regs" "$map_mems" &
    gold_pid=$!
    # The gate's merged memories go to registers before flattening, while
    # the only flops in the module are the ones memory_map makes, which
    # keep marks.
    prepare gate "read_json $work/gate.json" "memory_map t:\$mem_v2 a:oxibit_merged_writes %i; $keep_regs" "" &
    gate_pid=$!
    wait "$gold_pid"
    wait "$gate_pid"
fi

# Ports on one side only: "SIDE_LACKING DIR WIDTH NAME" per line; a name
# on both sides with different widths stops the check.
jq -r -n --slurpfile g "$work/gold.ports.json" --slurpfile t "$work/gate.ports.json" '
    def ports($j; $m): $j[0].modules[$m].ports | map_values({d: .direction, w: (.bits | length)});
    ports($g; "gold") as $a | ports($t; "gate") as $b
    | ($a | keys[] | select($b[.] == null) | "gate \($a[.].d) \($a[.].w) \(.)"),
      ($b | keys[] | select($a[.] == null) | "gold \($b[.].d) \($b[.].w) \(.)"),
      ($a | keys[] | select($b[.] != null and $b[.] != $a[.]) | "width \(.) gold \($a[.]) gate \($b[.])")
' > "$work/unmatched.txt"
if grep '^width ' "$work/unmatched.txt"; then
    echo "ports differ in width (see $work/unmatched.txt)" >&2
    exit 2
fi
# Flops left in the miter (ones expose could not name on a side: private
# Q nets, or a Q that is part of another wire) become free inputs: a
# conservative cut, which can only fail compares, never prove one wrongly.
# Their count lands in WORKDIR/leftover_ffs.txt.
#
# One-sided outputs are not compared: they stop being ports on the side
# that has them (one selection, read from a file: there can be thousands).
# One-sided inputs get an unread stand-in on the other side.
awk '$2 == "output" { print ($1 == "gate" ? "gold" : "gate") "/" $4 }' "$work/unmatched.txt" > "$work/unported.sel"
{
    echo "read_rtlil $work/gold.il"
    echo "read_rtlil $work/gate.il"
    if [ -s "$work/unported.sel" ]; then
        echo "select -read $work/unported.sel"
        echo "delete -port"
        echo "select -clear"
    fi
    awk '$2 == "input" { printf "add -input \\%s %s %s\n", $4, $3, $1 }' "$work/unmatched.txt"
    cat <<EOF
miter -equiv -flatten -ignore_gold_x -make_outcmp gold gate miter
hierarchy -top miter
delete -port miter/w:trigger
select -set leftover miter/t:\$*dff*
tee -q -o $work/leftover_ffs.txt select -count @leftover
cutpoint @leftover
select -clear
setundef -undriven -anyseq
opt -fast -purge
techmap
opt -fast -purge
aigmap
write_aiger -map $work/miter.map $work/miter.aig
EOF
} > "$work/miter.ys"
"$yosys" -q -l "$work/miter.log" -s "$work/miter.ys" > /dev/null

cargo run --quiet --release --manifest-path "$here/Cargo.toml" -p oxibit-cec --example cec -- \
    "$work/miter.aig" "$work/miter.map" "$jobs" > "$work/result.txt"

{ grep -E '^(FAIL|OPEN) ' "$work/result.txt" || true; } > "$work/verdict.txt"
failed=$(grep -c '^FAIL' "$work/verdict.txt" || true)
open=$(grep -c '^OPEN' "$work/verdict.txt" || true)
echo "oxibit-cec: $(head -1 "$work/result.txt")"
echo "compared: failed $failed unproven $open; stand-ins: $(wc -l < "$work/unmatched.txt") ports (see $work/unmatched.txt)"
[ "$failed" -eq 0 ] && [ "$open" -eq 0 ]
