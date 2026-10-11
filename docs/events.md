# Events: progress and results from oxibit tools

Long-running oxibit commands report through one event stream. A terminal
renders it, JSON Lines carries it to files and pipes, ports carry it to
Elixir (fabfarm, EEDA), and systemd shows its latest status. The schema
is the contract; `crates/oxibit-report` pins the wire form in its tests.

## Channels

| Channel | Carries |
|---|---|
| stdout | the result: an output file is written, and `--json` prints the final `done` event as one object |
| stderr | human progress (`--progress auto\|human\|plain\|json\|none`, `-q` = `none`) |
| `--events PATH` | the full JSON Lines stream, independent of what stderr shows |
| `NOTIFY_SOCKET` | systemd: `READY=1` at start, then `STATUS=` lines (below) |

These options are global (`oxibit --events run.jsonl fasm2bit ...` or after
the subcommand). `--progress auto` draws a live status line on a terminal
and plain lines otherwise (pipes, `CI` set, `TERM=dumb`). `json` puts the
JSON Lines stream on stderr instead of text.

Commands that had status lines before events existed (`fasm2bit`, `route`,
`verify`'s summary) print them unchanged in every mode but `json` and
`none`; scripts that read them keep working.

## Wire form

One JSON object per line, flushed per event:

```json
{"v":1,"t_ms":14400,"ev":"progress","phase":"route","iter":1,"overused":8874,"routed":6871,"unroutable":0,"pops":5659057,"iter_ms":3925}
```

Every event has:

| Field | Meaning |
|---|---|
| `v` | schema version, `1` |
| `t_ms` | milliseconds since the command started |
| `ev` | event type, below |

## Event types

| `ev` | Fields | When |
|---|---|---|
| `start` | `cmd`, `version`, `args` | first |
| `phase` | `phase`, `state` (`begin`/`end`), `ms` on `end` | around each phase |
| `progress` | `phase`, then phase-specific counters | during a phase |
| `warn` | `code`, `msg` | when found |
| `done` | `ok`, `exit`, `cancelled`, `phases` (name -> ms), `peak_rss_mb`, `stats`, `check`, `outputs` | last |

A consumer that only wants the outcome reads the last line.

## Rules

* Additive only: new event types and fields may appear in `v: 1`; existing
  fields never change meaning or type. A breaking change bumps `v`.
* Consumers ignore event types and fields they do not know.
* Key order is not significant.

## Exit status

| Code | Meaning |
|---|---|
| 0 | success, and every check passed |
| 1 | ran, but a check failed (overuse, unroutable or unreached sinks, new shorts) |
| 2 | usage |
| 3 | environment: database, input or output file |
| 130 | cancelled (SIGINT/SIGTERM); partial result written, `done` has `cancelled: true` |

`fasm2bit`: 0 written, 1 the FASM does not assemble, 3 environment (before
events existed, every error was 2). `verify`: 0 all pass, 1 any fails, 2
usage, as before.

## Signals (long commands: `pnr route`)

| Signal | Effect |
|---|---|
| SIGINT, SIGTERM | stop at the next check (between nets), write what is done, `done` with `cancelled: true`, exit 130; a second one exits at once with 130 |
| SIGUSR1; SIGINFO (Ctrl-T on macOS/BSD) | print the current status line on stderr and carry on |

## systemd

With `NOTIFY_SOCKET` set (`systemd-run -p Type=notify ...`), the command
sends `READY=1` at start, `STATUS=<phase>...` at each phase, `STATUS=` with
the progress counters at most once a second, and `STATUS=done`, `failed`
or `cancelled` at the end. `systemctl status UNIT` shows the latest.

## `pnr route`

Phases, in order: `db`, `graph`, `nets`, `route`, `fasm`, `check`, `write`.

| Phase | `progress` counters |
|---|---|
| `graph` | `nodes`, `edges` |
| `nets` | `nets`, `site_lines` (three-part FASM lines that are site settings, not pips) |
| `route` | per iteration: `iter`, `routed` (nets routed this iteration), `overused` (nodes on 2+ nets), `unroutable` (sinks), `pops` (A* heap pops), `iter_ms` |

`warn` codes: `pip_unresolved` (a design pip the graph has no usable edge
for), `multi_driven` (design nodes with two listed drivers), `write`.

`done.stats`: `router` (iterations, overused, routed_nets, unroutable_sinks,
heap_pops, seconds, cancelled), `classes` (nets and sinks per net class:
signal, constant, clock, default), `pips_removed`, `pips_added`.

`done.check`: `design` and `new`, each `sinks`, `sinks_missing`, `shorts`
(nodes reached from two sources over that FASM alone), and `new_shorts`
(shorts in the new routing the design's own routing does not have).

## `fasm2bit`, `verify`, `route`

* `fasm2bit`: phases `db`, `assemble`, `write`; `done.stats.bytes`;
  `warn` code `assemble` or `environment` on failure.
* `verify`: one `progress` event per bitstream (`file`, `ok`, `die`,
  `problems`); `done.stats` `checked`, `failed`.
* `route`: `start`, one phase `route` around the whole command, `done`.

## Planned (same schema, `v: 1`)

* Port frames: tag `0x02` + event JSON before the final `0x00`/`0x01`, on
  requests that ask for them; ops `0x01`-`0x12` of `oxibit port` never send them.
* `snapshot` events: per-tile congestion for live views (EEDA).
