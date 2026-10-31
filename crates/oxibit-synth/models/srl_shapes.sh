#!/usr/bin/env bash
# Prove every srlcases pair equivalent: flop chain (ref) vs SRLC32E (dut).
#
#   models/srl_shapes.sh CASEDIR [YOSYS] [CELLS_SIM] [JOBS]
#
# Each side is elaborated on its own against yosys's xilinx simulation
# models (so the primitives are derived with the case's parameters), then
# the two meet in a miter proven by k-induction (sat -tempinduct; init
# attributes give the base state), deep enough to cover the chain. Prints
# one line per case and a total; exits 1 if any case fails.
set -u

casedir=$1
yosys=${2:-yosys}
cells_sim=${3:-$(dirname "$(command -v "$yosys")")/../share/yosys/xilinx/cells_sim.v}
jobs=${4:-1}

prove_case() {
    local dir=$1
    local n inv tap
    read -r n inv tap < "$dir/case.txt"
    local steps=$(( n + 4 ))

    # Elaborate each side.
    local side
    for side in ref dut; do
        "$yosys" -q -l "$dir/$side.log" -p "
            read_verilog -noautowire $cells_sim
            read_json $dir/$side.json
            hierarchy -check -top top
            proc
            flatten
            opt_clean
            write_rtlil $dir/$side.il
        " > /dev/null 2>&1
    done

    # Miter and k-induction.
    if "$yosys" -q -l "$dir/prove.log" -p "
        read_rtlil $dir/ref.il
        rename top gold
        read_rtlil $dir/dut.il
        rename top gate
        miter -equiv -flatten -make_assert gold gate miter
        hierarchy -top miter
        sat -verify -tempinduct -prove-asserts -maxsteps $steps miter
    " > /dev/null 2>&1; then
        echo "PASS $(basename "$dir") n=$n clk_inv=$inv tap=$tap"
    else
        echo "FAIL $(basename "$dir") n=$n clk_inv=$inv tap=$tap (see $dir/prove.log)"
    fi
}
export -f prove_case
export yosys cells_sim

results=$(ls -d "$casedir"/case*/ | sed 's:/$::' | xargs -P "$jobs" -I{} bash -c 'prove_case {}' | sort -k2)
echo "$results"

pass=$(grep -c '^PASS' <<< "$results")
fail=$(grep -c '^FAIL' <<< "$results")
echo "pass $pass fail $fail"
[ "$fail" -eq 0 ]
