#!/usr/bin/env bash
# Cross-build oxibit for a Zynq-7000 running Nerves (armv7, glibc).
#
# Links with the Nerves armv7 toolchain so the binary is built against the
# same glibc as the target rootfs. Output: target/armv7-unknown-linux-gnueabihf/release/oxibit
set -euo pipefail

main() {
    local repo
    repo="$(cd "$(dirname "$0")/.." && pwd)"

    # 1. Find the Nerves toolchain's gcc.
    local gcc
    gcc="$(ls -d "$HOME"/.nerves/artifacts/nerves_toolchain_armv7_nerves_linux_gnueabihf-*/bin/armv7-nerves-linux-gnueabihf-gcc 2>/dev/null | sort | tail -n 1 || true)"
    if [[ -z "$gcc" ]]; then
        echo "no Nerves armv7 toolchain under ~/.nerves/artifacts; build a Nerves firmware once first" >&2
        exit 1
    fi

    # 2. Build with it as the linker.
    export CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER="$gcc"
    cd "$repo"
    cargo build --release --target armv7-unknown-linux-gnueabihf -p oxibit

    # 3. Report what was built.
    local out="$repo/target/armv7-unknown-linux-gnueabihf/release/oxibit"
    ls -la "$out"
    shasum -a 256 "$out"
}

main "$@"
