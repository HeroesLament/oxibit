#!/usr/bin/env bash
# Differential test: random Trellis configurations packed by ecppack and by
# oxibit-ecp5 must give byte-identical bitstreams, and ecpunpack and
# oxibit-ecp5 must read that bitstream back to the same text.
#
#   crates/oxibit-ecp5/fuzz.sh [DEVICE] [FIRST_SEED] [COUNT]
#
# Stops at the first mismatch and leaves its config and both bitstreams in
# the work directory. ECPPACK and OXIBIT_TRELLIS_DB override the defaults;
# FLAGS (e.g. "--compress") is passed to both packers.
set -euo pipefail

main() {
    local device="${1:-LFE5U-85F}"
    local first="${2:-1}"
    local count="${3:-50}"
    local ecppack="${ECPPACK:-$HOME/oss-cad-suite/bin/ecppack}"
    local ecpunpack="${ECPUNPACK:-$HOME/oss-cad-suite/bin/ecpunpack}"
    local -a flags
    read -r -a flags <<< "${FLAGS:-}"
    local work="${TMPDIR:-/tmp}/oxibit-ecp5-fuzz"
    local root
    root="$(cd "$(dirname "$0")/../.." && pwd)"

    # Build the two examples once, release.
    cargo build --release -p oxibit-ecp5 --example fuzz_config --example ecp5_pack --example ecp5_unpack \
        --manifest-path "$root/Cargo.toml"
    local bin="$root/target/release/examples"
    mkdir -p "$work"

    local seed
    local last=$((first + count - 1))
    for seed in $(seq "$first" "$last"); do
        local cfg="$work/s$seed.config"
        "$bin/fuzz_config" "$device" "$seed" "$cfg"
        "$ecppack" ${flags[@]+"${flags[@]}"} "$cfg" "$work/s$seed.ref.bit"
        "$bin/ecp5_pack" ${flags[@]+"${flags[@]}"} "$cfg" "$work/s$seed.oxi.bit"
        "$ecpunpack" "$work/s$seed.ref.bit" "$work/s$seed.ref.txt" 2>/dev/null
        "$bin/ecp5_unpack" "$work/s$seed.ref.bit" "$work/s$seed.oxi.txt"
        if cmp -s "$work/s$seed.ref.bit" "$work/s$seed.oxi.bit" \
            && cmp -s "$work/s$seed.ref.txt" "$work/s$seed.oxi.txt"; then
            echo "seed $seed: identical"
            rm -f "$cfg" "$work/s$seed".{ref,oxi}.{bit,txt}
        else
            echo "seed $seed: DIFFERENT (kept in $work)"
            cmp -l "$work/s$seed.ref.bit" "$work/s$seed.oxi.bit" | head -5 || true
            diff "$work/s$seed.ref.txt" "$work/s$seed.oxi.txt" | head -10 || true
            return 1
        fi
    done
    echo "$count seeds identical on $device ${FLAGS:-}"
}

main "$@"
