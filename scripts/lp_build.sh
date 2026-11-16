#!/bin/bash
# picoBEAM through this tree's back end on a virt host: pack + Nesterov
# placement (oxibit-place `pack --gp`), route + FASM (oxibit-pnr
# `standalone`), bitstream (`oxibit fasm2bit`).
#
#   lp_build.sh DIR STAGE [ENV=VAL ...]
#
# DIR holds synth.json. STAGE is place, route, bit or all. Each stage logs
# to DIR/<stage>.{out,err}; DIR/<stage>.rc holds its exit status once it
# has finished.
set -u

dir=$1
stage=$2
shift 2

readonly bin=/srv/dsp/pb/src/lutperm/target/release
readonly rt=/srv/dsp/rt
readonly db=$rt/db/prjxray-db/zynq7
readonly meta=$rt/db/nextpnr-xilinx/xilinx/external/nextpnr-xilinx-meta/zynq7
readonly part=xc7z020clg400-1

cd "$dir" || exit 1
# Address-space cap for every stage: the host also runs VMs.
ulimit -v $((10 * 1024 * 1024))

run_place() {
    # No package pins: the design talks to the PS only.
    : > empty.xdc
    env "$@" /usr/bin/time -f "wall=%es rss=%MKB" \
        "$bin/examples/pack" --gp "$db" "$part" empty.xdc "$meta" synth.json placed.json \
        > place.out 2> place.err
    local rc=$?
    echo "$rc" > place.rc
    return "$rc"
}

run_route() {
    env OXIBIT_TIMING="pb_clk=50000,axi_clk=8000" "$@" /usr/bin/time -f "wall=%es rss=%MKB" \
        "$bin/examples/standalone" "$db" "$part" "$meta" placed.json top.fasm \
        > route.out 2> route.err
    local rc=$?
    echo "$rc" > route.rc
    return "$rc"
}

run_bit() {
    "$bin/oxibit" fasm2bit --db-root "$db" --part "$part" top.fasm top.bit > bit.out 2> bit.err
    local rc=$?
    echo "$rc" > bit.rc
    sha256sum top.bit >> bit.out
    return "$rc"
}

case "$stage" in
    place) run_place "$@" ;;
    route) run_route "$@" ;;
    bit)   run_bit ;;
    all)   run_place "$@" && run_route "$@" && run_bit ;;
    *)     echo "unknown stage: $stage" >&2; exit 2 ;;
esac
