//! `oxibit port`: the Erlang port front end.
//!
//! Framing is the BEAM's `{:packet, 4}`: every message in both directions is
//! a 4-byte big-endian length and that many bytes.
//!
//! ## Requests
//!
//! Ops 0x01 to 0x06 are byte-compatible with nervezynq's `fabric_port`, so an
//! existing driver can switch binaries without a protocol change.
//!
//! | Op | Name | Payload | Reply payload |
//! |---|---|---|---|
//! | 0x01 | READ32 | `u32 offset` | `u32` |
//! | 0x02 | WRITE32 | `u32 offset, u32 value` | - |
//! | 0x03 | READ_BLOCK | `u32 offset, u16 count` | `u32 x count` |
//! | 0x04 | WRITE_BLOCK | `u32 offset, u16 count, u32 x count` | - |
//! | 0x05 | INFO | - | `u64 base, u32 len` |
//! | 0x06 | READ_REPEAT | `u32 offset, u16 count` | `u32 x count` |
//! | 0x07 | EXEC | records: `0x01 u32 offset` (read32) or `0x02 u32 offset, u32 value` (write32) | `u32` per read, in order |
//! | 0x10 | LOAD | UTF-8 path, or JSON `{"path": .., "aperture": bool}` | JSON load report |
//! | 0x11 | STATUS | - | JSON status |
//! | 0x12 | CLEAR | - | JSON status |
//!
//! ## Replies
//!
//! `0x00` and the reply payload, or `0x01` and an error: UTF-8 text for ops
//! up to 0x06 (as `fabric_port`), a JSON object for 0x10 and up.
//!
//! ## What is different from `fabric_port`
//!
//! The gate is here, not in the caller. Every access op re-reads hardware
//! state first and is refused unless the PL is configured and the boundary is
//! open; LOAD and CLEAR run in this same process, under the same lock, so
//! nothing can reconfigure the PL between the check and the access.

use std::io::{self, Read, Write};
use std::path::Path;
use std::process::ExitCode;

use oxibit_guard::{Aperture, Backend, Error, Guard, LoadOptions};
use oxibit_zynq7::{Zynq7, ZynqAperture};

use crate::device;
use crate::input;

const OP_READ32: u8 = 0x01;
const OP_WRITE32: u8 = 0x02;
const OP_READ_BLOCK: u8 = 0x03;
const OP_WRITE_BLOCK: u8 = 0x04;
const OP_INFO: u8 = 0x05;
const OP_READ_REPEAT: u8 = 0x06;
/// A register program in one message: interleaved reads and writes, run in
/// order under one access check. A test sweep of hundreds of accesses costs
/// one port round trip instead of one each.
const OP_EXEC: u8 = 0x07;
const OP_LOAD: u8 = 0x10;
const OP_STATUS: u8 = 0x11;
const OP_CLEAR: u8 = 0x12;

const RESP_OK: u8 = 0x00;
const RESP_ERR: u8 = 0x01;

pub fn serve(base: u64, len: usize) -> ExitCode {
    // Exit codes follow fabric_port where they overlap: 69 = cannot map,
    // 70 = port loop failed. 75 = another process owns the PL.
    let mut guard = match device::guard(base, len) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("oxibit port: {e}");
            return ExitCode::from(if e.contains("held by") { 75 } else { 69 });
        }
    };

    match run(&mut guard) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("oxibit port: {e}");
            ExitCode::from(70)
        }
    }
}

fn run(guard: &mut Guard<Zynq7>) -> io::Result<()> {
    let mut stdin = io::stdin().lock();
    let mut stdout = io::stdout().lock();

    while let Some(request) = read_frame(&mut stdin)? {
        let (code, payload) = match dispatch(guard, &request) {
            Ok(p) => (RESP_OK, p),
            Err(p) => (RESP_ERR, p),
        };
        let mut frame = Vec::with_capacity(payload.len() + 1);
        frame.push(code);
        frame.extend_from_slice(&payload);
        write_frame(&mut stdout, &frame)?;
        stdout.flush()?;
    }
    Ok(()) // EOF: the VM closed the port.
}

type Reply = Result<Vec<u8>, Vec<u8>>;

fn dispatch(guard: &mut Guard<Zynq7>, req: &[u8]) -> Reply {
    let Some((&op, rest)) = req.split_first() else {
        return text_err("empty request");
    };

    match op {
        OP_INFO => {
            expect_len(rest, 0)?;
            let a = guard.backend().aperture();
            let mut out = a.base().to_be_bytes().to_vec();
            out.extend_from_slice(&(a.len() as u32).to_be_bytes());
            Ok(out)
        }
        OP_READ32 | OP_WRITE32 | OP_READ_BLOCK | OP_WRITE_BLOCK | OP_READ_REPEAT => {
            let a = guard.access().map_err(|e| e.to_string().into_bytes())?;
            access(a, op, rest).map_err(String::into_bytes)
        }
        OP_EXEC => {
            let a = guard.access().map_err(|e| e.to_string().into_bytes())?;
            exec(a, rest).map_err(String::into_bytes)
        }
        OP_LOAD => {
            let (path, options) = load_request(rest).map_err(|e| json_err(&io_error(&e)))?;
            let found = input::load_one(Path::new(&path)).map_err(|e| json_err(&io_error(&e)))?;
            let report = guard.load(&found.bitstream, &found.label, options).map_err(|e| json_err(&e))?;
            Ok(json(&report))
        }
        OP_STATUS => {
            expect_len(rest, 0)?;
            guard.status().map(|s| json(&s)).map_err(|e| json_err(&e))
        }
        OP_CLEAR => {
            expect_len(rest, 0)?;
            guard.clear().map(|s| json(&s)).map_err(|e| json_err(&e))
        }
        other => text_err(&format!("unknown opcode {other:#04x}")),
    }
}

/// LOAD payload: a bare UTF-8 path, or `{"path": "...", "aperture": false, "hash": false}`.
fn load_request(rest: &[u8]) -> Result<(String, LoadOptions), String> {
    let text = std::str::from_utf8(rest).map_err(|_| "LOAD payload is not UTF-8".to_string())?;
    if !text.starts_with('{') {
        return Ok((text.to_string(), LoadOptions::default()));
    }
    let v: serde_json::Value = serde_json::from_str(text).map_err(|e| format!("LOAD payload: {e}"))?;
    let path = v["path"].as_str().ok_or("LOAD payload has no \"path\"")?.to_string();
    let aperture = v.get("aperture").and_then(|a| a.as_bool()).unwrap_or(true);
    let hash = v.get("hash").and_then(|h| h.as_bool()).unwrap_or(true);
    Ok((path, LoadOptions { aperture, hash }))
}

/// `oxibit mem`: ops 0x01 to 0x06 over a PS-side window. No guard is needed
/// because `ZynqAperture::open_ps` refuses every range a PL slave could sit
/// behind; everything reachable here answers regardless of the PL.
pub fn serve_mem(base: u64, len: usize) -> ExitCode {
    let window = match ZynqAperture::open_ps(base, len) {
        Ok(w) => w,
        Err(e) => {
            eprintln!("oxibit mem: {e}");
            return ExitCode::from(69);
        }
    };

    let result = (|| -> io::Result<()> {
        let mut stdin = io::stdin().lock();
        let mut stdout = io::stdout().lock();
        while let Some(request) = read_frame(&mut stdin)? {
            let reply = match request.split_first() {
                Some((&OP_INFO, [])) => {
                    let mut out = window.base().to_be_bytes().to_vec();
                    out.extend_from_slice(&(window.len() as u32).to_be_bytes());
                    Ok(out)
                }
                Some((&op, rest)) if matches!(op, OP_READ32 | OP_WRITE32 | OP_READ_BLOCK | OP_WRITE_BLOCK | OP_READ_REPEAT) => {
                    access(&window, op, rest)
                }
                Some((&op, _)) => Err(format!("unknown opcode {op:#04x} (oxibit mem serves 0x01-0x06)")),
                None => Err("empty request".into()),
            };
            let mut frame = Vec::new();
            match reply {
                Ok(p) => {
                    frame.push(RESP_OK);
                    frame.extend_from_slice(&p);
                }
                Err(m) => {
                    frame.push(RESP_ERR);
                    frame.extend_from_slice(m.as_bytes());
                }
            }
            write_frame(&mut stdout, &frame)?;
            stdout.flush()?;
        }
        Ok(())
    })();

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("oxibit mem: {e}");
            ExitCode::from(70)
        }
    }
}

/// The five aperture ops, decoded exactly as `fabric_port` decodes them.
fn access(a: &impl Aperture, op: u8, rest: &[u8]) -> Result<Vec<u8>, String> {
    let words_out = |w: Vec<u32>| w.iter().flat_map(|v| v.to_be_bytes()).collect::<Vec<u8>>();
    match op {
        OP_READ32 => {
            len_is(rest, 4)?;
            Ok(a.read32(u32_at(rest, 0)? as usize).map_err(|e| e.to_string())?.to_be_bytes().to_vec())
        }
        OP_WRITE32 => {
            len_is(rest, 8)?;
            a.write32(u32_at(rest, 0)? as usize, u32_at(rest, 4)?).map_err(|e| e.to_string())?;
            Ok(Vec::new())
        }
        OP_READ_BLOCK => {
            len_is(rest, 6)?;
            a.read_block(u32_at(rest, 0)? as usize, u16_at(rest, 4)? as usize).map(words_out).map_err(|e| e.to_string())
        }
        OP_READ_REPEAT => {
            len_is(rest, 6)?;
            a.read_repeat(u32_at(rest, 0)? as usize, u16_at(rest, 4)? as usize).map(words_out).map_err(|e| e.to_string())
        }
        _ => {
            // OP_WRITE_BLOCK
            let offset = u32_at(rest, 0)? as usize;
            let count = u16_at(rest, 4)? as usize;
            len_is(rest, 6 + count * 4)?;
            let values = (0..count).map(|i| u32_at(rest, 6 + i * 4)).collect::<Result<Vec<_>, _>>()?;
            a.write_block(offset, &values).map_err(|e| e.to_string())?;
            Ok(Vec::new())
        }
    }
}

/// OP_EXEC: decode the whole program first, so a malformed request touches
/// nothing, then run it. Stops at the first failing access.
fn exec(a: &impl Aperture, rest: &[u8]) -> Result<Vec<u8>, String> {
    enum Step {
        Read(usize),
        Write(usize, u32),
    }
    let mut steps = Vec::new();
    let mut at = 0;
    while at < rest.len() {
        match rest[at] {
            0x01 => {
                steps.push(Step::Read(u32_at(rest, at + 1)? as usize));
                at += 5;
            }
            0x02 => {
                steps.push(Step::Write(u32_at(rest, at + 1)? as usize, u32_at(rest, at + 5)?));
                at += 9;
            }
            k => return Err(format!("EXEC: unknown record kind {k:#04x} at byte {at}")),
        }
    }

    let mut out = Vec::new();
    for (i, step) in steps.iter().enumerate() {
        match *step {
            Step::Read(off) => {
                let v = a.read32(off).map_err(|e| format!("EXEC step {i}: read32 0x{off:X}: {e}"))?;
                out.extend_from_slice(&v.to_be_bytes());
            }
            Step::Write(off, v) => {
                a.write32(off, v).map_err(|e| format!("EXEC step {i}: write32 0x{off:X}: {e}"))?;
            }
        }
    }
    Ok(out)
}

fn json<T: serde::Serialize>(v: &T) -> Vec<u8> {
    serde_json::to_vec(v).unwrap_or_else(|e| format!("{{\"error\":\"io\",\"reason\":\"{e}\"}}").into_bytes())
}

fn json_err(e: &Error) -> Vec<u8> {
    json(e)
}

fn io_error(reason: &str) -> Error {
    Error::Io { reason: reason.to_string() }
}

fn text_err(msg: &str) -> Reply {
    Err(msg.as_bytes().to_vec())
}

fn expect_len(rest: &[u8], want: usize) -> Result<(), Vec<u8>> {
    len_is(rest, want).map_err(String::into_bytes)
}

fn len_is(rest: &[u8], want: usize) -> Result<(), String> {
    if rest.len() == want { Ok(()) } else { Err(format!("payload is {} bytes, expected {want}", rest.len())) }
}

fn u32_at(b: &[u8], at: usize) -> Result<u32, String> {
    b.get(at..at + 4).map(|s| u32::from_be_bytes([s[0], s[1], s[2], s[3]])).ok_or_else(|| format!("truncated u32 at byte {at}"))
}

fn u16_at(b: &[u8], at: usize) -> Result<u16, String> {
    b.get(at..at + 2).map(|s| u16::from_be_bytes([s[0], s[1]])).ok_or_else(|| format!("truncated u16 at byte {at}"))
}

fn read_frame(r: &mut impl Read) -> io::Result<Option<Vec<u8>>> {
    let mut header = [0u8; 4];
    match r.read_exact(&mut header) {
        Ok(()) => {}
        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(e),
    }
    let mut body = vec![0u8; u32::from_be_bytes(header) as usize];
    r.read_exact(&mut body)?;
    Ok(Some(body))
}

fn write_frame(w: &mut impl Write, body: &[u8]) -> io::Result<()> {
    w.write_all(&(body.len() as u32).to_be_bytes())?;
    w.write_all(body)
}
