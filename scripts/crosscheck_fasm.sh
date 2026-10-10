#!/usr/bin/env bash
# crosscheck_fasm.sh FASM... -- build each FASM with prjxray (fasm2frames +
# xc7frames2bit) and with `oxibit fasm2bit`, and require identical .frm text
# and identical configuration data (everything from the bus-width pattern
# on; the .bit header's free-text records legitimately differ).
set -uo pipefail

oxc7=~/src/openxc7
db=$oxc7/prjxray-db-517d66a/zynq7
part=xc7z020clg400-1
oxibit=~/src/oxibit/target/release/oxibit
work=$(mktemp -d)
pass=0
fail=0

# Configuration data from the bus-width detection word onward, as hex.
config_hex() {
  local bit="$1"
  xxd -p "$bit" | tr -d '\n' | sed -E 's/^.*(000000bb11220044)/\1/'
}

n=0
for fasm in "$@"; do
  # Index prefix: designs in different build dirs share basenames.
  n=$((n + 1))
  name="$n-$(basename "$fasm" .fasm)"

  # Reference: prjxray.
  OXC7_ROOT=$oxc7 PYTHONWARNINGS=ignore ~/src/ehdl/scripts/bin/fasm2frames \
    --part $part --db-root $db "$fasm" > "$work/$name.ref.frm"
  $oxc7/prjxray/build/tools/xc7frames2bit --part_file $db/$part/part.yaml --part_name $part \
    --frm_file "$work/$name.ref.frm" --output_file "$work/$name.ref.bit"

  # Candidate: oxibit.
  $oxibit fasm2bit "$fasm" "$work/$name.ox.bit" --db-root $db --part $part --frames "$work/$name.ox.frm" 2> "$work/$name.ox.log"

  if ! cmp -s "$work/$name.ref.frm" "$work/$name.ox.frm"; then
    echo "$name: FRAMES DIFFER ($(diff "$work/$name.ref.frm" "$work/$name.ox.frm" | grep -c '^<') frames)"
    fail=$((fail + 1))
  elif [ "$(config_hex "$work/$name.ref.bit")" != "$(config_hex "$work/$name.ox.bit")" ]; then
    echo "$name: frames match, BITSTREAM DIFFERS"
    fail=$((fail + 1))
  else
    echo "$name: identical  ($(tail -1 "$work/$name.ox.log" | sed 's/.*(//; s/)//'))"
    pass=$((pass + 1))
  fi
done

echo "crosscheck: $pass identical, $fail different  (work dir $work)"
