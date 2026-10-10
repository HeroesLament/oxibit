# Formats

What oxibit reads, with the evidence each claim rests on.

## Containers

| Container | Detection | Word order |
|---|---|---|
| `.bit` | starts `00 09 0F F0` | big-endian |
| raw `.bin` | sync `AA 99 55 66` at a word boundary | big-endian |
| swapped `.bin` | sync `66 55 99 AA` at a word boundary | each word byte-reversed |
| Zynq boot image | `0xAA995566` at 0x20 and `XNLX` at 0x24, little-endian | partitions as stored (bootgen stores PL data swapped) |

### `.bit` header

```text
u16 9, 0F F0 0F F0 0F F0 0F F0 00     opaque preamble
u16 0x0001
'a' u16 len, NUL-terminated string   design; ";UserID=...;Version=..." (Vivado) or ";Generator=..." (xc7frames2bit)
'b' u16 len, string                  part: "xc7z020clg400-1" (openXC7), "7z020clg400" (Vivado, no "xc")
'c' u16 len, string                  date
'd' u16 len, string                  time
'e' u32 len, configuration data
```

Anything after the `e` record is ignored.

## Packets

After the sync word:

```text
Type 1:  [31:29]=001  [28:27] opcode  [26:13] register  [12:11] reserved  [10:0] word count
Type 2:  [31:29]=010  [28:27] opcode  [26:0] word count      register = last type 1's
```

Opcodes: 0 NOP, 1 read, 2 write, 3 reserved. A write of DESYNC (0x0D) to CMD
ends the synced region; words after it are padding until any further sync.

Typical openXC7 full bitstream (xc7z020, `inspect --packets`):

```text
      12  SYNC
      21  T1 WRITE CMD      RCRC
      31  T1 WRITE IDCODE   0x03727093
      33  T1 WRITE CMD      SWITCH
      52  T1 WRITE FAR      0x00000000
      54  T1 WRITE CMD      WCFG
      57  T1 WRITE FDRI
      58  T2 WRITE FDRI     1010808 words
 1010867  T1 WRITE CMD      RCRC
 1010871  T1 WRITE CMD      GRESTORE
 1010874  T1 WRITE CMD      LFRM
 1010976  T1 WRITE CMD      START
 1010985  T1 WRITE CMD      RCRC
 1010989  T1 WRITE CMD      DESYNC
```

Vivado's compressed bitstreams (`COMPRESS=TRUE`) replace most FDRI data with
MFW (multi-frame write) sequences: `CMD MFW`, then `FAR` / `MFWR` pairs.

## CRC

Per written word, `crc = icap_crc(register & 0x1F, word, crc)`, CRC-32C
reflected (0x82F63B78) over 37 bits, data then address (prjxray `icap_crc`).

State rules, established against a Vivado 2022.2 compressed bitstream where
both of its CRC checks must pass:

- zero at each sync word;
- every written word to any register except CRC is folded in;
- CMD RCRC resets to zero;
- a write to CRC is a check against the running value, **then resets to
  zero**. Without that reset the second check in the Vivado stream fails.

## Zynq-7000 boot image

Little-endian words. Offsets confirmed against the stock LibreSDR `BOOT.bin`:

| Offset | Field | Stock value |
|---|---|---|
| 0x020 | width detection | 0xAA995566 |
| 0x024 | signature `XNLX` | 0x584C4E58 |
| 0x030 | FSBL source offset | 0x1700 |
| 0x034 | FSBL length | 0x18008 |
| 0x048 | `!sum(0x020..=0x044)` | 0xFC164530 (matches) |
| 0x098 | image header table | 0x8C0 |
| 0x09C | partition header table | 0xC80 |

Image header table (0x8C0): version 0x01020000, 3 images, first partition
header at word 0x320, first image header at word 0x240. Each image header:
next (word offset), partition header (word offset), reserved, partition count,
then the name, four characters per word packed big-endian.

Partition header, 16 words:

| Word | Field | fsbl.elf | system_top.bit | u-boot.elf |
|---|---|---|---|---|
| 0 | encrypted length (words) | 0x6002 | 0x8C950 | 0x19467 |
| 1 | unencrypted length | 0x6002 | 0x8C950 | 0x19467 |
| 2 | total length | 0x6002 | 0x8C950 | 0x19467 |
| 3 | load address | 0 | 0 | 0x04000000 |
| 4 | exec address | 0 | 0 | 0x04000000 |
| 5 | data offset (words) | 0x5C0 | 0x65D0 | 0x92F20 |
| 6 | attributes, [5:4] destination | 0x10 (PS) | 0x20 (PL) | 0x10 (PS) |
| 7 | section count | 1 | 1 | 1 |
| 9 | image header (words) | 0x240 | 0x250 | 0x260 |
| 15 | `!sum(words 0..=14)` | matches | matches | matches |

The table ends with an all-zero header.
