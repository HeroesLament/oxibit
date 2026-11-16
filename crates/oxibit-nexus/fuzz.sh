#!/usr/bin/env bash
# Pack random database-valid FASM with oxibit-nexus and with prjoxide, and
# require byte-identical bitstreams (or both refusing). Odd seeds add IP
# writes; every seed runs uncompressed and compressed.
#
#   crates/oxibit-nexus/fuzz.sh [first_seed] [last_seed]
#
# PRJOXIDE: the reference binary, built from the same database checkout
# that oxibit-nexus reads (default ~/src/prjoxide build).
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
prjoxide=${PRJOXIDE:-$HOME/src/prjoxide/libprjoxide/target/release/prjoxide}
first=${1:-1}
last=${2:-50}
work=$root/target/nexus_fuzz
ex=$root/target/release/examples
mkdir -p "$work"

cargo build --release -q -p oxibit-nexus --examples

pass=0
fail=0
for seed in $(seq "$first" "$last"); do
    ip=""
    if (( seed % 2 )); then ip="--ip"; fi
    fasm=$work/$seed.fasm
    "$ex/fuzz_fasm" $ip "$seed" > "$fasm"
    for mode in plain compress; do
        flag=""
        if [[ $mode == compress ]]; then flag="--compress"; fi
        ref=$work/$seed.$mode.ref.bit
        ox=$work/$seed.$mode.ox.bit
        rm -f "$ref" "$ox"
        ref_ok=yes
        "$prjoxide" pack $flag "$fasm" "$ref" > "$work/$seed.$mode.ref.log" 2>&1 || ref_ok=no
        ox_ok=yes
        "$ex/nexus_pack" $flag "$fasm" "$ox" > "$work/$seed.$mode.ox.log" 2>&1 || ox_ok=no
        if [[ $ref_ok == no && $ox_ok == no ]]; then
            echo "seed $seed $mode: both refuse"
            pass=$((pass + 1))
        elif [[ $ref_ok != "$ox_ok" ]]; then
            echo "seed $seed $mode: FAIL prjoxide=$ref_ok oxibit=$ox_ok"
            fail=$((fail + 1))
        elif cmp -s "$ref" "$ox"; then
            pass=$((pass + 1))
        else
            echo "seed $seed $mode: FAIL $(cmp "$ref" "$ox" | head -1)"
            fail=$((fail + 1))
        fi
    done
done
echo "pass=$pass fail=$fail"
(( fail == 0 ))
