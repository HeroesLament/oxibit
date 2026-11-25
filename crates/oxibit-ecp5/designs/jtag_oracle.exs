# JTAG oracle for oxibit ECP5 test designs on the ULX3S.
#
# Reads the design's ER1 data register (IR 0x32, 128 bits) through
# openocd's TCL port and checks it against what the design must show.
# One JSON line per sample, then one line per check, then a summary.
# Exit status 0 when every check passes, 1 otherwise.
#
#   elixir jtag_oracle.exs [--design pll|soft|lutram|iol|clk|gear|dsp] [--bit F.bit] [--cfg F]
#       [--samples N] [--gap-ms MS] [--clk-hz HZ] [--pll-hz HZ] [--gp0 0|1] [--sign-pins 0..15]
#
# --design pll (default): pll_jtag.v (counters, PLL lock, TDI round trip).
# --design soft: soft_probe.v (x written over TDI, LUT / gate / MUX / ROM /
#   CCU2C outputs and eight flops checked against a model here).
# --design iol: iol_probe.v (IOLOGIC registers, latch, DDR, delays; pads looped
#   back through BBs).
# --design clk: clk_probe.v (OSCG, gated DCCA, DCSC, GSR, DTR, SEDGA).
# --design gear: gear_probe.v (ECLKSYNCB, CLKDIVF, ODDRX2F/IDDRX2F, ODDR71B/IDDR71B).
# --design dsp: dsp_probe.v (MULT18X18D unregistered/registered, plus the
#   9x9 split multiplier, 9-bit pre-adder and 24-bit ALU split).
# --design lutram: lutram_probe.v (DPR16X4C, SPR16X4C, TRELLIS_DPR16X4
#   written and read through x, checked against a model of their contents).
#
# openocd comes from $OPENOCD or ~/oss-cad-suite/bin/openocd. --bit loads
# SRAM through openocd first (no fujprog, no sudo); without it the read
# does not reconfigure the FPGA.

import Bitwise

defmodule Ocd do
  @eot <<0x1A>>

  def start(cfg, port) do
    exe = System.get_env("OPENOCD", Path.expand("~/oss-cad-suite/bin/openocd"))

    args = [
      "-f", cfg,
      "-c", "gdb_port disabled",
      "-c", "telnet_port disabled",
      "-c", "tcl_port #{port}",
      "-c", "init"
    ]

    p = Port.open({:spawn_executable, exe}, [:binary, :exit_status, :stderr_to_stdout, args: args])
    wait_listen(p, "")
    {:ok, s} = connect(port, 20)
    {p, s}
  end

  defp wait_listen(p, acc) do
    receive do
      {^p, {:data, d}} ->
        acc = acc <> d
        if acc =~ "Listening on port", do: acc, else: wait_listen(p, acc)

      {^p, {:exit_status, st}} ->
        raise "openocd exited (#{st}):\n#{acc}"
    after
      10_000 -> raise "openocd did not start:\n#{acc}"
    end
  end

  defp connect(_port, 0), do: {:error, :refused}

  defp connect(port, n) do
    case :gen_tcp.connect(~c"127.0.0.1", port, [:binary, active: false]) do
      {:ok, s} -> {:ok, s}
      _ -> Process.sleep(100) && connect(port, n - 1)
    end
  end

  def cmd(s, c, timeout \\ 5000) do
    :ok = :gen_tcp.send(s, c <> @eot)
    recv(s, "", timeout)
  end

  defp recv(s, acc, timeout) do
    {:ok, d} = :gen_tcp.recv(s, 0, timeout)
    acc = acc <> d
    if String.ends_with?(acc, @eot), do: String.trim_trailing(acc, @eot), else: recv(s, acc, timeout)
  end

  def stop({p, s}) do
    cmd(s, "shutdown")
    :gen_tcp.close(s)

    receive do
      {^p, {:exit_status, _}} -> :ok
    after
      2000 -> :ok
    end
  end
end

defmodule Oracle do
  @magic 0x0B1BEC55

  def gray(g), do: gray(g, g >>> 1)
  defp gray(b, 0), do: b
  defp gray(b, s), do: gray(bxor(b, s), s >>> 1)

  def field(v, lo, w), do: v >>> lo &&& (1 <<< w) - 1

  # One ER1 scan. `user` goes out in [127:112] and is latched on Update-DR.
  def read(s, user) do
    _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
    tdi = "0x" <> String.pad_leading(Integer.to_string(user <<< 112, 16), 32, "0")
    hex = Ocd.cmd(s, "drscan ecp5.tap 128 #{tdi}") |> String.trim()
    t = System.monotonic_time(:microsecond)
    v = String.to_integer(hex, 16)
    st = field(v, 96, 32)

    %{
      t_us: t,
      raw: hex,
      magic: field(v, 0, 32),
      c25: gray(field(v, 32, 32)),
      c48: gray(field(v, 64, 32)),
      lock: field(st, 0, 1),
      gp0: field(st, 1, 1),
      btn1: field(st, 2, 1),
      btn0: field(st, 3, 1),
      tag: field(st, 4, 4),
      ncap: field(st, 8, 8),
      user_echo: field(st, 16, 16),
      sent: user
    }
  end

  def wrap(a, b, bits), do: b - a &&& (1 <<< bits) - 1

  def checks(xs, o) do
    pairs = Enum.zip(xs, tl(xs))
    first = hd(xs)

    rate = fn key, hz ->
      Enum.map(pairs, fn {a, b} -> wrap(a[key], b[key], 32) / ((b.t_us - a.t_us) / 1.0e6) / hz end)
    end

    ratio =
      Enum.map(pairs, fn {a, b} -> wrap(a.c48, b.c48, 32) / max(wrap(a.c25, b.c25, 32), 1) end)

    want_ratio = o.pll_hz / o.clk_hz
    r25 = rate.(:c25, o.clk_hz)
    valid = o.idcode == "41113043" and Enum.all?(xs, &(&1.magic == @magic))

    [
      {"idcode", o.idcode == "41113043", o.idcode},
      {"magic", Enum.all?(xs, &(&1.magic == @magic)), "0x#{Integer.to_string(first.magic, 16)}"},
      {"tag", Enum.all?(xs, &(&1.tag == 0xA)), "#{first.tag}"},
      {"pll_lock", Enum.all?(xs, &(&1.lock == 1)), "#{Enum.map(xs, & &1.lock) |> Enum.join()}"},
      {"c25_rate", Enum.all?(r25, &(abs(&1 - 1) < 0.03)), fmt(r25, "x #{o.clk_hz}")},
      {"pll_ratio", Enum.all?(ratio, &(abs(&1 / want_ratio - 1) < 0.002)), fmt(ratio, "want #{want_ratio}")},
      {"ncap_step", Enum.all?(pairs, fn {a, b} -> wrap(a.ncap, b.ncap, 8) == 1 end),
       "#{Enum.map(xs, & &1.ncap) |> Enum.join(",")}"},
      {"tdi_update", Enum.all?(pairs, fn {a, b} -> b.user_echo == a.sent end),
       "#{Enum.map(xs, &Integer.to_string(&1.user_echo, 16)) |> Enum.join(",")}"},
      {"gp0", o.gp0 == nil or Enum.all?(xs, &(&1.gp0 == o.gp0)), "#{Enum.map(xs, & &1.gp0) |> Enum.join()}"}
    ]
    |> Enum.map(fn
      {n, ok, d} when n in ["idcode", "magic"] -> {n, ok, d}
      # Without a valid read (all ones from a dead chain) nothing else counts.
      {n, ok, d} -> {n, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end

  defp fmt(v, note), do: "#{Enum.map(v, &Float.round(&1, 4)) |> Enum.join(",")} (#{note})"

  def line(m) do
    if Code.ensure_loaded?(JSON), do: JSON.encode!(m), else: inspect(m)
  end
end

defmodule Soft do
  @moduledoc false
  # soft_probe.v: x in, y = combinational primitives of x and eight flops.
  @magic 0x50F7C0DE

  defp b(x, i), do: x >>> i &&& 1
  defp lut(init, idx), do: init >>> idx &&& 1
  defp par(x, lo, hi), do: Enum.reduce(lo..hi, 0, fn i, a -> bxor(a, b(x, i)) end)

  # y[23:0] for input x.
  def comb(x) do
    y0 = lut(0xCA35, x &&& 0xF)
    y1 = lut(0x6996, x >>> 4 &&& 0xF)
    y2 = lut(0xDEADBEEF, x >>> 8 &&& 0x1F)
    y3 = if b(x, 13) == 1, do: y0, else: y1
    y4 = if b(x, 14) == 1, do: y2, else: y3
    y5 = b(x, 0) &&& b(x, 1) &&& b(x, 2)
    y6 = 1 - (b(x, 3) &&& b(x, 4) &&& b(x, 5) &&& b(x, 6))
    y7 = Enum.reduce(7..11, 0, fn i, a -> a ||| b(x, i) end)
    y8 = 1 - (b(x, 12) ||| b(x, 13))
    y9 = par(x, 14, 16)
    y10 = 1 - bxor(b(x, 17), b(x, 18))
    y11 = 1 - b(x, 19)
    y12 = b(x >>> 20 &&& 0xF, b(x, 24) ||| b(x, 25) <<< 1)
    y13 = b(x &&& 0xFF, b(x, 26) ||| b(x, 27) <<< 1 ||| b(x, 28) <<< 2)
    y14 = lut(0x8BADF00D, x >>> 27 &&& 0x1F)
    sum = (x &&& 0xF) + (x >>> 4 &&& 0xF)
    bits = [y0, y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11, y12, y13, y14, 1] ++
             Enum.map(0..4, &b(sum, &1)) ++ [par(x, 0, 20), par(x, 10, 20), b(x, 31)]
    bits |> Enum.with_index() |> Enum.reduce(0, fn {v, i}, a -> a ||| v <<< i end)
  end

  # Flop state after the update that moved x from p to c; q is the state
  # before (nil = unknown). nil propagates where the flop holds.
  def flops(p, c, q) do
    hold = fn k, take, v -> if take, do: v, else: Enum.at(q, k) end
    [
      hold.(0, b(p, 1) == 1, b(p, 0)),
      hold.(1, b(p, 3) == 1, b(p, 2)),
      cond do
        b(c, 6) == 1 or b(p, 6) == 1 -> 0
        true -> hold.(2, b(p, 5) == 1, b(p, 4))
      end,
      cond do
        b(c, 9) == 1 or b(p, 9) == 1 -> 1
        true -> hold.(3, b(p, 8) == 1, b(p, 7))
      end,
      cond do
        b(c, 12) == 1 or b(p, 12) == 1 -> 1
        true -> hold.(4, b(p, 11) == 1, b(p, 10))
      end,
      cond do
        b(c, 15) == 1 or b(p, 15) == 1 -> 0
        true -> hold.(5, b(p, 14) == 1, b(p, 13))
      end,
      b(c, 16),
      hold.(7, b(p, 20) == 1, if(b(p, 19) == 1, do: b(p, 18), else: b(p, 17)))
    ]
  end

  def run(s, o) do
    :rand.seed(:exsss, {7, 11, 13})
    sent = for _ <- 1..o.samples, do: :rand.uniform(1 <<< 32) - 1
    reads =
      for x <- sent do
        _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
        tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 96, 16), 32, "0")
        v = Ocd.cmd(s, "drscan ecp5.tap 128 #{tdi}") |> String.trim() |> String.to_integer(16)
        r = %{magic: Oracle.field(v, 0, 32), x: Oracle.field(v, 32, 32), y: Oracle.field(v, 64, 32),
              tag: Oracle.field(v, 96, 8), ncap: Oracle.field(v, 104, 8)}
        IO.puts(Oracle.line(Map.merge(r, %{sent: x, raw: Integer.to_string(v, 16)})))
        r
      end
    check(sent, reads)
  end

  # Read k returns the state after update k-1: x = sent[k-1], flops after
  # the step from sent[k-2] to sent[k-1].
  def check(sent, reads) do
    pairs = Enum.zip(reads, tl(reads))
    comb_bad = Enum.reduce(reads, 0, fn r, a -> a ||| bxor(r.y &&& 0xFFFFFF, comb(r.x)) end)
    x_bad = Enum.count(Enum.zip(tl(reads), sent), fn {r, x} -> r.x != x end)
    {flop_bad, flop_checked, _} =
      reads
      |> Enum.zip(Enum.drop(reads, 1))
      |> Enum.reduce({0, 0, List.duplicate(nil, 8)}, fn {a, r}, {bad, n, q} ->
        e = flops(a.x, r.x, q)
        got = for k <- 0..7, do: b(r.y, 24 + k)
        {bad2, n2} =
          Enum.zip(e, got) |> Enum.with_index() |> Enum.reduce({bad, n}, fn
            {{nil, _}, _}, acc -> acc
            {{ev, gv}, k}, {bb, nn} -> {if(ev != gv, do: bb ||| 1 <<< k, else: bb), nn + 1}
          end)
        # Unknown expectations take the observed value.
        q2 = Enum.zip(e, got) |> Enum.map(fn {ev, gv} -> ev || gv end)
        {bad2, n2, q2}
      end)
    hex = &("0x" <> Integer.to_string(&1, 16))
    valid = Enum.all?(reads, &(&1.magic == @magic))
    [
      {"magic", valid, hex.(hd(reads).magic)},
      {"tag", Enum.all?(reads, &(&1.tag == 0xB0)), hex.(hd(reads).tag)},
      {"ncap_step", Enum.all?(pairs, fn {a, b} -> (b.ncap - a.ncap &&& 0xFF) == 1 end), "#{length(pairs)} steps"},
      {"x_roundtrip", x_bad == 0, "#{x_bad} of #{length(sent) - 1} wrong"},
      {"comb", comb_bad == 0, "bad bits #{hex.(comb_bad)} over #{length(reads)} vectors"},
      {"flops", flop_bad == 0 and flop_checked > 0, "bad #{hex.(flop_bad)}, #{flop_checked} checked"}
    ]
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {n, ok, d} -> {n, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Lutram do
  @moduledoc false
  # lutram_probe.v: three 16x4 RAMs written on the update edge from the x
  # before it, read asynchronously from the current x.
  @magic 0x1D7A3000
  import Bitwise

  defp f(x, lo, w), do: x >>> lo &&& (1 <<< w) - 1
  defp words(init), do: for(w <- 0..15, do: init >>> (4 * w) &&& 0xF)

  # {we, wad, di, rad} of each RAM for input x.
  defp ports(x) do
    [
      {f(x, 8, 1), f(x, 0, 4), f(x, 4, 4), f(x, 9, 4)},
      {f(x, 21, 1), f(x, 13, 4), f(x, 17, 4), f(x, 13, 4)},
      {f(x, 30, 1), f(x, 22, 4), f(x, 26, 4), f(x, 0, 4)}
    ]
  end

  def run(s, o) do
    :rand.seed(:exsss, {3, 5, 7})
    sent = for _ <- 1..o.samples, do: :rand.uniform(1 <<< 32) - 1
    reads =
      for x <- sent do
        _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
        tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 96, 16), 32, "0")
        v = Ocd.cmd(s, "drscan ecp5.tap 128 #{tdi}") |> String.trim() |> String.to_integer(16)
        r = %{magic: Oracle.field(v, 0, 32), x: Oracle.field(v, 32, 32), y: Oracle.field(v, 64, 32),
              tag: Oracle.field(v, 96, 8), ncap: Oracle.field(v, 104, 8)}
        IO.puts(Oracle.line(Map.merge(r, %{sent: x, raw: Integer.to_string(v, 16)})))
        r
      end
    check(sent, reads, o.bit != nil)
  end

  # Read k shows x[k-1] and the RAMs after the update into x[k-1], which
  # wrote with the ports of x[k-2]. Without a fresh load the contents start
  # unknown (nil) and take the first value read.
  def check(sent, reads, fresh) do
    init = [0x0123456789ABCDEF, 0xFEDCBA9876543210, 0x5A5AC3C30F0F9696]
    mem0 = for i <- init, do: if(fresh, do: words(i), else: List.duplicate(nil, 16))
    {bad, checked, _} =
      reads
      |> Enum.zip(tl(reads))
      |> Enum.reduce({0, 0, mem0}, fn {a, r}, {bad, n, mem} ->
        mem =
          for {{we, wad, di, _}, m} <- Enum.zip(ports(a.x), mem) do
            if we == 1, do: List.replace_at(m, wad, di), else: m
          end
        got = for k <- 0..2, do: f(r.y, 4 * k, 4)
        {bad, n, mem} =
          ports(r.x)
          |> Enum.zip(Enum.zip(mem, got))
          |> Enum.with_index()
          |> Enum.reduce({bad, n, []}, fn {{{_, _, _, rad}, {m, g}}, k}, {b, nn, acc} ->
            case Enum.at(m, rad) do
              nil -> {b, nn, acc ++ [List.replace_at(m, rad, g)]}
              e -> {if(e != g, do: b ||| 1 <<< k, else: b), nn + 1, acc ++ [m]}
            end
          end)
        {bad, n, mem}
      end)
    x_bad = Enum.count(Enum.zip(tl(reads), sent), fn {r, x} -> r.x != x end)
    pairs = Enum.zip(reads, tl(reads))
    valid = Enum.all?(reads, &(&1.magic == @magic))
    hex = &("0x" <> Integer.to_string(&1, 16))
    [
      {"magic", valid, hex.(hd(reads).magic)},
      {"tag", Enum.all?(reads, &(&1.tag == 0xC0)), hex.(hd(reads).tag)},
      {"ncap_step", Enum.all?(pairs, fn {a, b} -> (b.ncap - a.ncap &&& 0xFF) == 1 end), "#{length(pairs)} steps"},
      {"x_roundtrip", x_bad == 0, "#{x_bad} of #{length(sent) - 1} wrong"},
      {"rams", bad == 0 and checked > 0, "bad RAMs #{hex.(bad)} (A=1 B=2 C=4), #{checked} reads checked"}
    ]
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {n, ok, d} -> {n, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Iol do
  @moduledoc false
  # iol_probe.v: IO registers on JTCK from x, each pad a loopback through
  # its BB. Read k shows x[k-1] and y a few JTCK edges after that update.
  @magic 0x10106000
  import Bitwise

  defp b(x, i), do: x >>> i &&& 1

  def run(s, o) do
    :rand.seed(:exsss, {11, 13, 17})
    sent = for _ <- 1..o.samples, do: :rand.uniform(1 <<< 32) - 1
    reads =
      for x <- sent do
        _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
        tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 96, 16), 32, "0")
        v = Ocd.cmd(s, "drscan ecp5.tap 128 #{tdi}") |> String.trim() |> String.to_integer(16)
        r = %{magic: Oracle.field(v, 0, 32), x: Oracle.field(v, 32, 32), y: Oracle.field(v, 64, 32),
              tag: Oracle.field(v, 96, 8), ncap: Oracle.field(v, 104, 8)}
        IO.puts(Oracle.line(Map.merge(r, %{sent: x, raw: Integer.to_string(v, 16)})))
        r
      end
    check(sent, reads)
  end

  # A held value: nil until first seen.
  defp hold(nil, got), do: {got, true}
  defp hold(v, got), do: {v, v == got}

  # DDR pair: Q0/Q1 against D0/D1, in either order. Returns :same, :straight,
  # :swapped or :bad.
  defp ddr(d0, d1, q0, q1) do
    cond do
      d0 == d1 and q0 == d0 and q1 == d0 -> :same
      d0 == d1 -> :bad
      q0 == d0 and q1 == d1 -> :straight
      q0 == d1 and q1 == d0 -> :swapped
      true -> :bad
    end
  end

  def check(sent, reads) do
    rs = tl(reads)
    init = %{p1o: nil, p1t: nil, lat_hi: nil, lat_lo: nil, p6: nil, p6_inv: nil}
    {fails, _} =
      Enum.reduce(rs, {%{}, init}, fn r, {f, st} ->
        x = r.x
        y = &b(r.y, &1)
        bump = fn f, k, ok -> if ok, do: f, else: Map.update(f, k, 1, &(&1 + 1)) end
        # P1: out and tristate registers share the enable x20.
        {p1o, p1t} = if b(x, 20) == 1, do: {b(x, 0), b(x, 1)}, else: {st.p1o, st.p1t}
        p1 = if p1t == 1, do: 0, else: p1o
        f = if p1 == nil, do: f, else: bump.(bump.(f, "p1_ireg", y.(0) == p1), "p1_pad", y.(1) == p1)
        # P2 / P8: DDR out and back.
        k2 = ddr(b(x, 2), b(x, 3), y.(2), y.(3))
        k8 = ddr(b(x, 16), b(x, 17), y.(12), y.(13))
        f = Map.update(f, {:p2, k2}, 1, &(&1 + 1)) |> Map.update({:p8, k8}, 1, &(&1 + 1))
        # P3: input delay then input register; P4: output delay.
        f = bump.(bump.(f, "p3_dly_ireg", y.(4) == b(x, 4)), "p3_dly_z", y.(5) == b(x, 4))
        f = bump.(f, "p4_outdel", y.(6) == b(x, 5))
        f = Map.update(f, {:cflag, y.(7)}, 1, &(&1 + 1))
        # P5: latch, both gate polarities.
        {lh, okh} = if b(x, 9) == 1, do: {b(x, 8), y.(8) == b(x, 8)}, else: hold(st.lat_hi, y.(8))
        {ll, okl} = if b(x, 9) == 0, do: {b(x, 8), y.(8) == b(x, 8)}, else: hold(st.lat_lo, y.(8))
        f = bump.(bump.(f, "p5_latch_hi", okh), "p5_latch_lo", okl)
        # P6: sync clear x10 over enable x11, D x12; and with CD inverted.
        {p6, ok6} =
          cond do
            b(x, 10) == 1 -> {0, y.(9) == 0}
            b(x, 11) == 1 -> {b(x, 12), y.(9) == b(x, 12)}
            true -> hold(st.p6, y.(9))
          end
        {p6i, ok6i} =
          cond do
            b(x, 10) == 0 -> {0, y.(9) == 0}
            b(x, 11) == 1 -> {b(x, 12), y.(9) == b(x, 12)}
            true -> hold(st.p6_inv, y.(9))
          end
        f = bump.(bump.(f, "p6_sync_clear", ok6), "p6_clear_inverted", ok6i)
        # P7: sync preset x14 on both registers, D x13.
        p7 = b(x, 14) ||| b(x, 13)
        f = bump.(bump.(f, "p7_top_ireg", y.(10) == p7), "p7_top_pad", y.(11) == p7)
        {f, %{st | p1o: p1o, p1t: p1t, lat_hi: lh, lat_lo: ll, p6: p6, p6_inv: p6i}}
      end)
    n = length(rs)
    bad = fn k -> Map.get(fails, k, 0) end
    ddr_ok = fn p ->
      st = Map.get(fails, {p, :straight}, 0)
      sw = Map.get(fails, {p, :swapped}, 0)
      ok = Map.get(fails, {p, :bad}, 0) == 0 and (st == 0 or sw == 0) and Map.get(fails, {p, :same}, 0) > 0
      {ok, "same #{Map.get(fails, {p, :same}, 0)} straight #{st} swapped #{sw} bad #{Map.get(fails, {p, :bad}, 0)}"}
    end
    {p2ok, p2d} = ddr_ok.(:p2)
    {p8ok, p8d} = ddr_ok.(:p8)
    # IFS1S1D is transparent while SCLK is high.
    lat_ok = bad.("p5_latch_hi") == 0
    x_bad = Enum.count(Enum.zip(rs, sent), fn {r, x} -> r.x != x end)
    valid = Enum.all?(reads, &(&1.magic == @magic))
    pairs = Enum.zip(reads, rs)
    hex = &("0x" <> Integer.to_string(&1, 16))
    one = fn k -> {k, bad.(k) == 0, "#{bad.(k)} of #{n} wrong"} end
    [
      {"magic", valid, hex.(hd(reads).magic)},
      {"tag", Enum.all?(reads, &(&1.tag == 0xD0)), hex.(hd(reads).tag)},
      {"ncap_step", Enum.all?(pairs, fn {a, b} -> (b.ncap - a.ncap &&& 0xFF) == 1 end), "#{length(pairs)} steps"},
      {"x_roundtrip", x_bad == 0, "#{x_bad} of #{n} wrong"},
      one.("p1_ireg"),
      one.("p1_pad"),
      {"p2_ddr", p2ok, p2d},
      one.("p3_dly_ireg"),
      one.("p3_dly_z"),
      one.("p4_outdel"),
      {"p4_cflag", true, "0: #{Map.get(fails, {:cflag, 0}, 0)} 1: #{Map.get(fails, {:cflag, 1}, 0)}"},
      {"p5_latch", lat_ok, "gate high #{bad.("p5_latch_hi")} wrong, gate low #{bad.("p5_latch_lo")} wrong"},
      {"p6_sync_clear", bad.("p6_sync_clear") == 0, "#{bad.("p6_sync_clear")} wrong (CD inverted: #{bad.("p6_clear_inverted")} wrong)"},
      one.("p7_top_ireg"),
      one.("p7_top_pad"),
      {"p8_top_ddr", p8ok, p8d}
    ]
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {k, ok, d} -> {k, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Clk do
  @moduledoc false
  # clk_probe.v: 256-bit frame. A fixed sequence of x values, each held over
  # several reads so rates come from consecutive reads with the same x.
  @magic 0xC10C0000
  import Bitwise

  defp f(v, lo, w), do: v >>> lo &&& (1 <<< w) - 1

  defp scan(s, x) do
    _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
    tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 224, 16), 64, "0")
    v = Ocd.cmd(s, "drscan ecp5.tap 256 #{tdi}") |> String.trim() |> String.to_integer(16)
    t = System.monotonic_time(:microsecond)
    %{t_us: t, sent: x, magic: f(v, 0, 32), x: f(v, 32, 32), c25: Oracle.gray(f(v, 64, 32)),
      cosc: Oracle.gray(f(v, 96, 32)), cgate: Oracle.gray(f(v, 128, 32)),
      cdcs: Oracle.gray(f(v, 160, 32)), dtr: f(v, 192, 8), sed: f(v, 200, 3),
      tag: f(v, 224, 8), ncap: f(v, 232, 8)}
  end

  # {label, x, reads}
  defp plan do
    dcs = for m <- [0, 1], sel <- 0..3, do: {"dcs m#{m} sel#{sel}", 0x01 ||| sel <<< 1 ||| m <<< 7, 3}
    [{"gate on", 0x01, 3}, {"gate off", 0x00, 3}, {"gate on", 0x01, 3}] ++
      dcs ++
      [{"dtr start", 0x11, 1}, {"dtr", 0x01, 2}, {"sed enable", 0x21, 2}, {"sed start", 0x61, 1},
       {"sed run", 0x21, 12}, {"before gsr", 0x01, 2}, {"gsr", 0x09, 1}, {"after gsr", 0x01, 3}]
  end

  def run(s, o) do
    reads =
      for {label, x, n} <- plan(), _ <- 1..n do
        Process.sleep(o.gap_ms)
        r = scan(s, x) |> Map.put(:label, label)
        IO.puts(Oracle.line(Map.delete(r, :t_us)))
        r
      end
    check(reads, o)
  end

  defp wrap(a, b), do: b - a &&& 0xFFFFFFFF

  def check(reads, o) do
    # Read k shows x[k-1]: pair consecutive reads that show the same x.
    pairs =
      Enum.zip(reads, tl(reads))
      |> Enum.filter(fn {a, b} -> a.x == b.x and ((a.ncap + 1) &&& 0xFF) == b.ncap end)
    rate = fn {a, b}, k -> wrap(a[k], b[k]) / ((b.t_us - a.t_us) / 1.0e6) end
    by_x = Enum.group_by(pairs, fn {_, b} -> b.x end)
    mean = fn ps, k -> if ps == [], do: nil, else: Enum.sum(Enum.map(ps, &rate.(&1, k))) / length(ps) end
    r25 = mean.(pairs, :c25)
    rosc = mean.(pairs, :cosc)
    osc_nom = 310.0e6 / (o[:osc_div] || 26)
    on = Map.get(by_x, 0x01, [])
    off = Map.get(by_x, 0x00, [])
    class = fn r ->
      cond do
        r == nil -> "?"
        r < 1.0e3 -> "off"
        abs(r / o.clk_hz - 1) < 0.03 -> "clk0"
        rosc && abs(r / rosc - 1) < 0.03 -> "clk1"
        true -> "#{Float.round(r / 1.0e6, 3)}M"
      end
    end
    dcs =
      for m <- [0, 1], sel <- 0..3 do
        x = 0x01 ||| sel <<< 1 ||| m <<< 7
        {"m#{m}s#{sel}", class.(mean.(Map.get(by_x, x, []), :cdcs))}
      end
    dcs_txt = Enum.map_join(dcs, " ", fn {k, c} -> "#{k}=#{c}" end)
    dtr = reads |> Enum.filter(&(&1.label == "dtr")) |> Enum.map(& &1.dtr)
    sed_v = reads |> Enum.filter(&String.starts_with?(&1.label, "sed")) |> Enum.map(& &1.sed)
    sed = Enum.map(sed_v, &Integer.to_string(&1, 2))
    # GSR: the read after the pulse shows x, ncap and the 25 MHz count reset.
    i = Enum.find_index(reads, &(&1.label == "gsr"))
    {pre, post} = {Enum.at(reads, i), Enum.at(reads, i + 1)}
    gsr_ok = post.x == 0 and post.ncap <= 1 and post.c25 < pre.c25
    valid = Enum.all?(reads, &(&1.magic == @magic))
    fr = fn r -> if r, do: Float.round(r / 1.0e6, 4), else: nil end
    [
      {"magic", valid, "0x" <> Integer.to_string(hd(reads).magic, 16)},
      {"tag", Enum.all?(reads, &(&1.tag == 0xE0)), "#{hd(reads).tag}"},
      {"c25_rate", r25 != nil and abs(r25 / o.clk_hz - 1) < 0.03, "#{fr.(r25)} MHz"},
      {"oscg_rate", rosc != nil and abs(rosc / osc_nom - 1) < 0.25, "#{fr.(rosc)} MHz (nominal #{fr.(osc_nom)})"},
      {"dcca_on", on != [] and abs(mean.(on, :cgate) / o.clk_hz - 1) < 0.03, "#{fr.(mean.(on, :cgate))} MHz"},
      {"dcca_off", off != [] and Enum.all?(off, fn {a, b} -> a.cgate == b.cgate end), "#{length(off)} pairs frozen"},
      {"dcsc", Enum.any?(dcs, &(elem(&1, 1) == "clk0")) and Enum.any?(dcs, &(elem(&1, 1) == "clk1")), dcs_txt},
      {"dtr", true, "codes #{Enum.join(dtr, ",")}"},
      {"sed", Enum.any?(sed_v, &((&1 &&& 0b011) != 0)), "err/inprog/done #{Enum.join(sed, ",")}"},
      {"gsr", gsr_ok, "x #{post.x} ncap #{pre.ncap}->#{post.ncap} c25 #{pre.c25}->#{post.c25}"}
    ]
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {k, ok, d} -> {k, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Gear do
  @moduledoc false
  # gear_probe.v: X2 (4-bit) and 7:1 gearing loopbacks. Each read shows x of
  # the read before and the words the IDDRs deserialize from it.
  @magic 0x6EA20000
  import Bitwise

  defp f(v, lo, w), do: v >>> lo &&& (1 <<< w) - 1

  defp scan(s, x) do
    _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
    tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 96, 16), 32, "0")
    v = Ocd.cmd(s, "drscan ecp5.tap 128 #{tdi}") |> String.trim() |> String.to_integer(16)
    %{sent: x, magic: f(v, 0, 32), x: f(v, 32, 32), y: f(v, 64, 32), tag: f(v, 96, 8), ncap: f(v, 104, 8)}
  end

  # Every bit order the gearbox could deliver a w-bit word in: rotations,
  # straight or reversed.
  defp transforms(w), do: for(rev <- [false, true], k <- 0..(w - 1), do: {rev, k})

  defp apply_t(d, w, {rev, k}) do
    d = if rev, do: Enum.reduce(0..(w - 1), 0, fn i, a -> a ||| (d >>> i &&& 1) <<< (w - 1 - i) end), else: d
    (d <<< k ||| d >>> (w - k)) &&& (1 <<< w) - 1
  end

  def run(s, o) do
    :rand.seed(:exsss, {19, 23, 29})
    data = for _ <- 1..o.samples, do: (:rand.uniform(1 <<< 11) - 1) <<< 1
    # Run, then STOP (x0) with new data: the words must hold.
    stop = for _ <- 1..4, do: ((:rand.uniform(1 <<< 11) - 1) <<< 1) ||| 1
    plan = data ++ [hd(data)] ++ stop ++ [0, 0]
    reads =
      for x <- plan do
        Process.sleep(o.gap_ms)
        r = scan(s, x)
        IO.puts(Oracle.line(r))
        r
      end
    check(reads, length(data))
  end

  def check(reads, n) do
    run = reads |> Enum.drop(1) |> Enum.take(n)
    fits = fn w, sel_d, sel_y ->
      Enum.reduce(run, transforms(w), fn r, ts ->
        d = sel_d.(r.x)
        y = sel_y.(r.y)
        Enum.filter(ts, &(apply_t(d, w, &1) == y))
      end)
    end
    t2 = fits.(4, &f(&1, 1, 4), &f(&1, 0, 4))
    t7 = fits.(7, &f(&1, 5, 7), &f(&1, 4, 7))
    # STOP: the reads that show a stopped x hold the words of the first.
    stopped = reads |> Enum.drop(n + 2) |> Enum.take(4)
    held = stopped != [] and Enum.all?(stopped, &(f(&1.y, 0, 11) == f(hd(stopped).y, 0, 11)))
    fmt = fn ts -> Enum.map_join(ts, " ", fn {rev, k} -> "#{if rev, do: "rev", else: "fwd"}+#{k}" end) end
    valid = Enum.all?(reads, &(&1.magic == @magic))
    pairs = Enum.zip(reads, tl(reads))
    [
      {"magic", valid, "0x" <> Integer.to_string(hd(reads).magic, 16)},
      {"tag", Enum.all?(reads, &(&1.tag == 0xF0)), "#{hd(reads).tag}"},
      {"ncap_step", Enum.all?(pairs, fn {a, b} -> (b.ncap - a.ncap &&& 0xFF) == 1 end), "#{length(pairs)} steps"},
      {"x2_loop", t2 != [], "order #{fmt.(t2)} over #{length(run)} words"},
      {"x71_loop", t7 != [], "order #{fmt.(t7)} over #{length(run)} words"},
      {"eclk_stop", held, "y while stopped: #{Enum.map_join(stopped, ",", &Integer.to_string(f(&1.y, 0, 11), 16))}"}
    ]
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {k, ok, d} -> {k, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Dqs do
  @moduledoc false
  # dqs_probe.v (p37, p38: x17 RDMOVE, p39: x18 WRMOVE, p40: lat field = DQ words 1..4 cycles after DATAVALID): one LiteDRAM-style byte lane (DQS gp19, DQ gp20) with
  # the ECP5DDRPHYInit sequence in fabric. x0..3 the DQ word, x8 READ on,
  # x[14:12] READCLKSEL, x16 an edge re-runs init, x[23:20] the READ start
  # phase in the 16-cycle sys frame, x24 burst mode (else the strobe drives
  # always), x44 PAUSE request (synchronised to sclk), x45 READ 4 cycles,
  # x46 DQSBUFM RST held through init, x47 release PAUSE only on frame cycle
  # x[51:48], x52 a PAUSE pulse of x[63:60] cycles from frame cycle x[59:56]
  # on one frame in 2^x[55:53]. Scan bits 126 / 127 live dqs_rst /
  # pause_all; 144.. Gray event counters (DATAVALID cycles and rises,
  # BURSTDET / RDCFLAG / WRCFLAG rises), the release-to-first-DATAVALID
  # latency and (240..255) the PAUSE release count.
  #
  # Each measurement: enter the setting with PAUSE raised (c1), drop the
  # request (c2), wait, capture (c3): the steady events per PAUSE release.
  import Bitwise
  @magic 0xD9500000
  @period 7

  defp f(v, lo, w), do: v >>> lo &&& (1 <<< w) - 1

  defp ungray(g), do: ungray(g, g >>> 1)
  defp ungray(b, 0), do: b
  defp ungray(b, s), do: ungray(bxor(b, s), s >>> 1)

  defp scan(s, x) do
    _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
    tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 192, 16), 64, "0")
    v = Ocd.cmd(s, "drscan ecp5.tap 256 #{tdi}") |> String.trim() |> String.to_integer(16)
    %{sent: x, magic: f(v, 0, 32), x: f(v, 32, 64), y: f(v, 96, 16), pz: f(v, 112, 4),
      rst: f(v, 126, 1), pause: f(v, 127, 1), tag: f(v, 128, 8), ncap: f(v, 136, 8),
      dvc: ungray(f(v, 144, 16)), dvr: ungray(f(v, 160, 16)), bdr: ungray(f(v, 176, 16)),
      rdc: ungray(f(v, 192, 16)), wrc: ungray(f(v, 208, 16)), lat: f(v, 224, 16),
      pr: ungray(f(v, 240, 16))}
  end

  defp d16(a, b), do: b - a &&& 0xFFFF
  defp delta(a, b),
    do: %{dvc: d16(a.dvc, b.dvc), dvr: d16(a.dvr, b.dvr), bdr: d16(a.bdr, b.bdr),
          rdc: d16(a.rdc, b.rdc), wrc: d16(a.wrc, b.wrc), pr: d16(a.pr, b.pr)}

  defp measure(s, x, tag, wait \\ 5) do
    _ = scan(s, x ||| 1 <<< 44)
    c1 = scan(s, x)
    c2 = scan(s, x)
    Process.sleep(wait)
    c3 = scan(s, x)
    st = delta(c2, c3)
    m = %{tag: tag, x: x, c2: c2, c3: c3, steady: st}
    IO.puts(Oracle.line(%{tag: tag, dq: f(x, 0, 4), rds: f(x, 20, 4), sel: f(x, 12, 3), burst: f(x, 24, 1),
      fp: f(x, 52, 1), steady: st, lat: c3.lat, pause1: c1.pause, y: c3.y, pz: c3.pz,
      rdc_total: c3.rdc, magic: c3.magic, xok: c2.x == x and c3.x == x}))
    m
  end

  defp xw(d, rds, sel, burst, w4 \\ 0),
    do: d ||| 0x100 ||| sel <<< 12 ||| rds <<< 20 ||| burst <<< 24 ||| w4 <<< 45

  defp rot(v, r), do: (v <<< r ||| v >>> (4 - r)) &&& 0xF
  defp loop_ok(m), do: m.steady.dvr > 0 and Enum.any?(0..3, &(rot(f(m.x, 0, 4), &1) == f(m.c3.y, 0, 4)))

  # Step the read-side DQS delay with RDMOVE pulses (RDLOADN high) until
  # RDCFLAG rises or 64 steps; at each step measure 16 READ phases at
  # READCLKSEL 0 and 4 in burst mode, steady state (no PAUSE pulses).
  defp steps(s, name, ctl, n, acc, rdc0) do
    ms =
      for rds <- 0..15, sel <- [0, 4] do
        measure(s, xw(Enum.random([0x1, 0x2, 0x5, 0xA, 0xC]), rds, sel, 1) ||| ctl, "#{name}#{n}")
      end
    last = List.last(ms)
    acc = [{n, ms} | acc]
    if n >= 63 or d16(rdc0, last.c3.rdc) > 0 do
      Enum.reverse(acc)
    else
      x = 0x100 ||| ctl
      _ = scan(s, x ||| 1 <<< 17)
      _ = scan(s, x)
      steps(s, name, ctl, n + 1, acc, rdc0)
    end
  end

  def run(s, _o) do
    Process.sleep(200)
    # Read delay loaded, then 25 steps down (the read-leveled point found in
    # p38: READ phase 6, READCLKSEL 4).
    rd = 1 <<< 40
    _ = scan(s, 0x100)
    _ = scan(s, 0x100)
    _ = scan(s, 0x100 ||| rd)
    for _ <- 1..25 do
      _ = scan(s, 0x100 ||| rd ||| 1 <<< 17)
      _ = scan(s, 0x100 ||| rd)
    end
    lp = for d <- 1..15, _ <- 1..2, do: measure(s, xw(d, 6, 4, 1) ||| rd, "lp")
    # The write-side DQSW270 delay (which clocks DQ out) each way until
    # WRCFLAG, every nonzero DQ word at each step.
    wsw =
      for {name, dbit} <- [{"winc", 1 <<< 43}, {"wdec", 0}] do
        ctl = rd ||| 1 <<< 41 ||| dbit
        _ = scan(s, 0x100 ||| rd ||| dbit)
        _ = scan(s, 0x100 ||| rd ||| dbit)
        r = scan(s, 0x100 ||| ctl)
        {name, wsteps(s, name, ctl, 0, [], r.wrc)}
      end
    check(lp, wsw)
  end

  defp wsteps(s, name, ctl, n, acc, wrc0) do
    ms = for d <- 1..15, do: measure(s, xw(d, 6, 4, 1) ||| ctl, "#{name}#{n}")
    acc = [{n, ms} | acc]
    if n >= 63 or d16(wrc0, List.last(ms).c3.wrc) > 0 do
      Enum.reverse(acc)
    else
      x = 0x100 ||| ctl
      _ = scan(s, x ||| 1 <<< 18)
      _ = scan(s, x)
      wsteps(s, name, ctl, n + 1, acc, wrc0)
    end
  end

  defp rev4(v), do: (v &&& 1) <<< 3 ||| (v &&& 2) <<< 1 ||| (v &&& 4) >>> 1 ||| (v &&& 8) >>> 3

  # Transforms of the sent word: rotations (r0..r3) and rotations of its bit
  # reversal (v0..v3).
  defp xf(w), do: (for k <- 0..3, do: {"r#{k}", rot(w, k)}) ++ (for k <- 0..3, do: {"v#{k}", rot(rev4(w), k)})

  # The DQ word captured o sclk cycles after DATAVALID (o 0: on it).
  defp cap(m, 0), do: f(m.c3.y, 0, 4)
  defp cap(m, o), do: f(m.c3.lat, 4 * (o - 1), 4)

  # Best {offset, transform, fits, total} over the DATAVALID measurements.
  defp fit(ms) do
    dv = Enum.filter(ms, &(&1.steady.dvr > 0))
    best =
      for o <- 0..4, t <- 0..7 do
        n = Enum.count(dv, fn m -> {_, v} = Enum.at(xf(f(m.x, 0, 4)), t); v == cap(m, o) end)
        {o, elem(Enum.at(xf(1), t), 0), n}
      end
      |> Enum.max_by(&elem(&1, 2))
    {o, t, n} = best
    {o, t, n, length(dv)}
  end

  defp caps_of(m), do: Enum.map_join(0..4, "/", &Integer.to_string(cap(m, &1), 16))

  def check(lp, wsw) do
    all = lp ++ Enum.flat_map(wsw, fn {_, st} -> Enum.flat_map(st, fn {_, ms} -> ms end) end)
    caps = Enum.flat_map(all, &[&1.c2, &1.c3])
    valid = Enum.all?(caps, &(&1.magic == @magic))
    b = fn r, i -> f(r.y, i, 1) == 1 end
    {lo, lt, ln, ltot} = fit(lp)
    lpdv = Enum.filter(lp, &(&1.steady.dvr > 0))
    lpshow = lp |> Enum.take_every(2) |> Enum.map_join(" ", &"#{Integer.to_string(f(&1.x, 0, 4), 16)}:#{caps_of(&1)}")
    wfits = for {name, st} <- wsw, {n, ms} <- st, do: {name, n, fit(ms)}
    {wn, wi, {wo, wt, wf, wtot}} = Enum.max_by(wfits, fn {_, _, {_, _, nf, _}} -> nf end)
    wshow = wfits |> Enum.filter(fn {_, _, {_, _, nf, _}} -> nf >= 12 end) |> Enum.map_join(" ", fn {nm, n, {o, t, nf, tt}} -> "#{nm}#{n}:o#{o}#{t} #{nf}/#{tt}" end)
    [
      {"magic", valid, "0x" <> Integer.to_string(hd(caps).magic, 16)},
      {"x_roundtrip", Enum.all?(caps, &(&1.x == &1.sent)), "#{length(caps)} captures after the PAUSE entry"},
      {"locks", Enum.all?(caps, &(b.(&1, 14) and b.(&1, 5) and b.(&1, 15))), "pll / dll / init on all"},
      {"lp_datavalid", length(lpdv) == length(lp), "DATAVALID on #{length(lpdv)} of #{length(lp)}, BURSTDET on #{Enum.count(lp, &(&1.steady.bdr > 0))}"},
      {"lp_capture", true, "best offset #{lo} #{lt}: #{ln} of #{ltot}; word:caps o0..o4 " <> lpshow},
      {"wr_sweep", true, Enum.map_join(wsw, " | ", fn {nm, st} -> "#{nm}: #{length(st)} steps" end) <> "; steps fitting >= 12: " <> if(wshow == "", do: "none", else: wshow)},
      {"dq_loopback", (ln == ltot and ltot == 30) or (wf == 15 and wtot == 15),
       "read-leveled point: offset #{lo} #{lt} fits #{ln} of #{ltot}; best write step #{wn} #{wi}: offset #{wo} #{wt} fits #{wf} of #{wtot}"}
    ]
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {k, ok, d} -> {k, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Pad do
  @moduledoc false
  # pad_probe.v: pull-ups, pull-downs and open drain, each pad reading
  # itself back. Read k shows x[k-1].
  import Bitwise
  @magic 0xFADE0000

  defp f(v, lo, w), do: v >>> lo &&& (1 <<< w) - 1

  defp scan(s, x) do
    _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
    tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 192, 16), 64, "0")
    v = Ocd.cmd(s, "drscan ecp5.tap 256 #{tdi}") |> String.trim() |> String.to_integer(16)
    %{sent: x, magic: f(v, 0, 32), x: f(v, 32, 64), y: f(v, 96, 4), tag: f(v, 128, 8), ncap: f(v, 136, 8)}
  end

  def run(s, o) do
    # Every combination of the five control bits, twice, with gaps for
    # the pulls to settle.
    xs = for _ <- 1..2, v <- 0..31, do: v
    reads =
      for x <- [0 | xs] do
        Process.sleep(div(o.gap_ms, 10))
        r = scan(s, x)
        IO.puts(Oracle.line(r))
        r
      end
    check(reads)
  end

  def check(reads) do
    pairs = Enum.zip(tl(reads), reads)
    ok_x = Enum.all?(pairs, fn {r, p} -> r.x == p.sent end)
    rs = tl(reads)
    bit = fn r, i -> f(r.y, i, 1) end
    xb = fn r, i -> f(r.x, i, 1) end
    count = fn pred -> Enum.count(rs, &(not pred.(&1))) end
    pu_rel = count.(fn r -> xb.(r, 1) == 0 or bit.(r, 0) == 1 end)
    pu_drv = count.(fn r -> xb.(r, 1) == 1 or bit.(r, 0) == xb.(r, 0) end)
    pd_rel = count.(fn r -> xb.(r, 3) == 0 or bit.(r, 1) == 0 end)
    pd_drv = count.(fn r -> xb.(r, 3) == 1 or bit.(r, 1) == xb.(r, 2) end)
    od = count.(fn r -> bit.(r, 2) == 0 end)
    od_echo = Enum.count(rs, fn r -> bit.(r, 2) == xb.(r, 4) end)
    ibpu = count.(fn r -> bit.(r, 3) == 1 end)
    valid = Enum.all?(reads, &(&1.magic == @magic))
    n = length(rs)
    [
      {"magic", valid, "0x" <> Integer.to_string(hd(reads).magic, 16)},
      {"tag", Enum.all?(reads, &(&1.tag == 0xFD)), "#{hd(reads).tag}"},
      {"x_roundtrip", ok_x, "#{n} reads"},
      {"bbpu_release", pu_rel == 0, "#{pu_rel} of #{n} wrong (reads 1)"},
      {"bbpu_drive", pu_drv == 0, "#{pu_drv} of #{n} wrong"},
      {"bbpd_release", pd_rel == 0, "#{pd_rel} of #{n} wrong (reads 0)"},
      {"bbpd_drive", pd_drv == 0, "#{pd_drv} of #{n} wrong"},
      {"opendrain", od == 0, "#{od} of #{n} wrong (reads 0; it echoed I #{od_echo} times)"},
      {"ibpu", ibpu == 0, "#{ibpu} of #{n} wrong (reads 1)"}
    ]
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {k, ok, d} -> {k, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Bram do
  @moduledoc false
  # bram_probe.v: an instantiated DP16KD (x18, port A writes) and a
  # PDPW16KD (x36), both preloaded by INITVAL_00. Read k shows the RAMs
  # after x[k-1] was applied (written on the TCK edges between scans).
  import Bitwise
  @magic 0xB0B00000

  defp f(v, lo, w), do: v >>> lo &&& (1 <<< w) - 1

  def init_word(j) when j < 16, do: 0x2A5A5 + j * 0x0F0F1 &&& 0x3FFFF
  def init_word(_), do: 0

  defp scan(s, x) do
    _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
    tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 192, 16), 64, "0")
    v = Ocd.cmd(s, "drscan ecp5.tap 256 #{tdi}") |> String.trim() |> String.to_integer(16)
    %{sent: x, magic: f(v, 0, 32), x: f(v, 32, 64), doa: f(v, 96, 18), dob: f(v, 114, 18),
      do: f(v, 132, 36), tag: f(v, 168, 8), ncap: f(v, 176, 8)}
  end

  defp xw(adda, addb, d, we, pwe, adw, adr),
    do: adda ||| addb <<< 10 ||| d <<< 20 ||| we <<< 38 ||| pwe <<< 39 ||| adw <<< 40 ||| adr <<< 49

  def run(s, o) do
    :rand.seed(:exsss, {59, 61, 67})
    r = fn n -> :rand.uniform(n) - 1 end
    # The preload first (no writes), then random writes and reads in a
    # small address range so reads hit written words.
    pre = for a <- 0..15, do: xw(15 - a, a, 0, 0, 0, 0, rem(a, 8))
    mix = for _ <- 1..o.samples * 2, do: xw(r.(24), r.(24), r.(1 <<< 18), r.(2), r.(2), r.(12), r.(12))
    reads =
      for x <- [0 | pre ++ mix] do
        rd = scan(s, x)
        IO.puts(Oracle.line(rd))
        rd
      end
    check(reads)
  end

  # The models before the read that shows x: x18 words and x36 words, the
  # x36 word r as the x18 pair (2r, 2r + 1) in either half order.
  def check(reads) do
    pairs = Enum.zip(tl(reads), reads)
    ok_x = Enum.all?(pairs, fn {r, p} -> r.x == p.sent end)
    init18 = Map.new(0..1023, &{&1, init_word(&1)})
    lo_hi = fn m, a -> m[2 * a] ||| m[2 * a + 1] <<< 18 end
    hi_lo = fn m, a -> m[2 * a + 1] ||| m[2 * a] <<< 18 end
    {res, _, _} =
      Enum.reduce(tl(pairs), {[], init18, init18}, fn {rd, _}, {acc, m18, m36} ->
        x = rd.x
        adda = f(x, 0, 10)
        addb = f(x, 10, 10)
        d = f(x, 20, 18)
        we = f(x, 38, 1)
        pwe = f(x, 39, 1)
        adw = f(x, 40, 9)
        adr = f(x, 49, 9)
        m18 = if we == 1, do: Map.put(m18, adda, d), else: m18
        m36 = if pwe == 1, do: m36 |> Map.put(2 * adw, d) |> Map.put(2 * adw + 1, d), else: m36
        e = %{
          dob: rd.dob == m18[addb],
          doa: we == 1 or rd.doa == m18[adda],
          pdp_lo_hi: rd.do == lo_hi.(m36, adr),
          pdp_hi_lo: rd.do == hi_lo.(m36, adr),
          pre: we == 0 and pwe == 0 and addb < 16
        }
        {[{rd, e} | acc], m18, m36}
      end)
    res = Enum.reverse(res)
    count = fn k -> Enum.count(res, fn {_, e} -> not e[k] end) end
    pre = Enum.filter(res, fn {_, e} -> e.pre end)
    bad_pre = Enum.count(pre, fn {_, e} -> not e.dob end)
    order = cond do
      count.(:pdp_lo_hi) == 0 -> "DO = {word 2r+1, word 2r}"
      count.(:pdp_hi_lo) == 0 -> "DO = {word 2r, word 2r+1}"
      true -> "lo-hi #{count.(:pdp_lo_hi)} wrong, hi-lo #{count.(:pdp_hi_lo)} wrong"
    end
    ex = case Enum.find(res, fn {_, e} -> not e.dob end) do
      {rd, _} -> " e.g. x=#{Integer.to_string(rd.x, 16)} dob=#{Integer.to_string(rd.dob, 16)}"
      nil -> ""
    end
    valid = Enum.all?(reads, &(&1.magic == @magic))
    [
      {"magic", valid, "0x" <> Integer.to_string(hd(reads).magic, 16)},
      {"tag", Enum.all?(reads, &(&1.tag == 0xB5)), "#{hd(reads).tag}"},
      {"x_roundtrip", ok_x, "#{length(reads) - 1} reads"},
      {"dp_initval", pre != [] and bad_pre == 0, "#{bad_pre} of #{length(pre)} preload reads wrong"},
      {"dp_port_b", count.(:dob) == 0, "#{count.(:dob)} of #{length(res)} wrong#{ex}"},
      {"dp_port_a", count.(:doa) == 0, "#{count.(:doa)} of #{length(res)} wrong"},
      {"pdp_x36", count.(:pdp_lo_hi) == 0 or count.(:pdp_hi_lo) == 0, order}
    ]
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {k, ok, d} -> {k, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Alu do
  @moduledoc false
  # alu_probe.v: ALU54B on two MULT18X18D products. OP[9:0] is x[63:54].
  # Silicon (p21): OP[1:0] and OP[3:2] select the two adder inputs (01 MA /
  # MB, 10 A, 11 zero) at OP[9:4] = 0x22: OP 0x225 must give R = MA + MB.
  # Every opcode is reported by which function of the inputs R matched.
  import Bitwise
  @magic 0xA1A10000
  @add 0x225
  @m54 0x3FFFFFFFFFFFFF

  defp f(v, lo, w), do: v >>> lo &&& (1 <<< w) - 1

  defp scan(s, x) do
    _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
    tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 192, 16), 64, "0")
    v = Ocd.cmd(s, "drscan ecp5.tap 256 #{tdi}") |> String.trim() |> String.to_integer(16)
    %{sent: x, magic: f(v, 0, 32), x: f(v, 32, 64), r: f(v, 96, 54), tag: f(v, 160, 8), ncap: f(v, 168, 8)}
  end

  # The candidate functions of x, by name.
  def cands(x) do
    p0 = f(x, 0, 18) * f(x, 18, 18)
    p1 = f(x, 36, 9) * f(x, 45, 9)
    a = f(x, 0, 36)
    b = f(x, 36, 9) ||| f(x, 45, 9) <<< 18
    c = f(x, 0, 54)
    [
      {"MA+MB", p0 + p1}, {"MA-MB", p0 - p1}, {"MB-MA", p1 - p0}, {"MA", p0}, {"MB", p1},
      {"A+B", a + b}, {"A-B", a - b}, {"A", a}, {"B", b}, {"-MA-MB", -p0 - p1}, {"0", 0},
      {"C", c}, {"MA+MB+C", p0 + p1 + c}, {"C-MA-MB", c - p0 - p1}, {"MA+MB-C", p0 + p1 - c}, {"A+B+C", a + b + c}, {"MA+C", p0 + c}, {"MB+C", p1 + c}
    ]
    |> Enum.map(fn {k, v} -> {k, v &&& @m54} end)
  end

  def run(s, o) do
    :rand.seed(:exsss, {43, 47, 53})
    rnd = fn -> :rand.uniform(1 <<< 54) - 1 end
    adds = for _ <- 1..o.samples, do: rnd.() ||| @add <<< 54
    ops = for op <- 0..1023, rem(op, 16) == 0 or div(op, 16) == 0x22 or op in [0x205, 0x215, 0x235, 0x025, 0x125, 0x325], do: rnd.() ||| op <<< 54
    reads =
      for x <- [0 | adds ++ ops] do
        r = scan(s, x)
        IO.puts(Oracle.line(r))
        r
      end
    check(reads)
  end

  def check(reads) do
    rs = Enum.zip(tl(reads), reads) |> Enum.map(fn {r, p} -> Map.put(r, :ok_x, r.x == p.sent) end) |> tl()
    rs = Enum.filter(rs, & &1.ok_x)
    op = &f(&1.x, 54, 10)
    add = Enum.filter(rs, &(op.(&1) == @add))
    bad = Enum.reject(add, &(&1.r == elem(hd(cands(&1.x)), 1)))
    table =
      rs
      |> Enum.group_by(op)
      |> Enum.sort()
      |> Enum.map_join(" ", fn {k, v} ->
        names = for r <- v, {n, c} <- cands(r.x), c == r.r, do: n
        m = names |> Enum.frequencies() |> Enum.filter(fn {_, c} -> c == length(v) end) |> Enum.map(&elem(&1, 0))
        "#{Integer.to_string(k, 16)}=#{if m == [], do: "?" <> Integer.to_string(hd(v).r, 16), else: Enum.join(m, "|")}"
      end)
    IO.puts("opcodes: " <> table)
    ex = case bad do
      [r | _] -> " e.g. x=#{Integer.to_string(r.x, 16)} got #{Integer.to_string(r.r, 16)} want #{Integer.to_string(elem(hd(cands(r.x)), 1), 16)}"
      [] -> ""
    end
    valid = Enum.all?(reads, &(&1.magic == @magic))
    [
      {"magic", valid, "0x" <> Integer.to_string(hd(reads).magic, 16)},
      {"tag", Enum.all?(reads, &(&1.tag == 0xA5)), "#{hd(reads).tag}"},
      {"x_roundtrip", length(rs) == length(reads) - 2, "#{length(rs)} of #{length(reads) - 2}"},
      {"alu_add", add != [] and bad == [], "#{length(bad)} of #{length(add)} wrong (OP 0x225, R = MA + MB)#{ex}"}
    ]
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {k, ok, d} -> {k, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Mul do
  @moduledoc false
  # mul_probe.v: `*` inferred to MULT18X18D. pu = x[11:0] * x[28:12]
  # (unsigned, 29 bits), ps = signed x[45:30] * signed x[63:46] (34 bits),
  # pr = x[7:0] * x[15:8] in a JTCK flop (16 bits).
  @magic 0x3A170000
  import Bitwise

  defp f(v, lo, w), do: v >>> lo &&& (1 <<< w) - 1
  defp sext(v, w), do: if((v >>> (w - 1) &&& 1) == 1, do: v - (1 <<< w), else: v)

  defp scan(s, x) do
    _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
    tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 192, 16), 64, "0")
    v = Ocd.cmd(s, "drscan ecp5.tap 256 #{tdi}") |> String.trim() |> String.to_integer(16)
    %{sent: x, magic: f(v, 0, 32), x: f(v, 32, 64), pu: f(v, 96, 29), ps: f(v, 128, 34),
      pr: f(v, 192, 16), tag: f(v, 224, 8), ncap: f(v, 232, 8)}
  end

  def want(x) do
    %{pu: f(x, 0, 12) * f(x, 12, 17),
      ps: sext(f(x, 30, 16), 16) * sext(f(x, 46, 18), 18) &&& (1 <<< 34) - 1,
      pr: f(x, 0, 8) * f(x, 8, 8)}
  end

  def run(s, o) do
    :rand.seed(:exsss, {43, 47, 53})
    ones = (1 <<< 64) - 1
    signs = (1 <<< 45) ||| (1 <<< 63)
    edges = [0, ones, signs, 0xFFF ||| 0x1FFFF <<< 12, 1 <<< 30 ||| 1 <<< 46, 0x8000 <<< 30 ||| 0x20000 <<< 46]
    xs = edges ++ for(_ <- 1..o.samples, do: :rand.uniform(1 <<< 64) - 1)
    reads =
      for x <- xs do
        r = scan(s, x)
        IO.puts(Oracle.line(r))
        r
      end
    check(reads)
  end

  def check(reads) do
    rs = tl(reads)
    bad = fn k -> Enum.filter(rs, &(Map.fetch!(&1, k) != Map.fetch!(want(&1.x), k))) end
    ex = fn
      [r | _], k -> " e.g. x=#{Integer.to_string(r.x, 16)} got #{Integer.to_string(Map.fetch!(r, k), 16)} want #{Integer.to_string(Map.fetch!(want(r.x), k), 16)}"
      [], _ -> ""
    end
    valid = Enum.all?(reads, &(&1.magic == @magic))
    [
      {"magic", valid, "0x" <> Integer.to_string(hd(reads).magic, 16)},
      {"tag", Enum.all?(reads, &(&1.tag == 0xD5)), "#{hd(reads).tag}"},
      {"x_roundtrip", Enum.all?(Enum.zip(rs, reads), fn {r, p} -> r.x == p.sent end), "#{length(rs)} reads"}
    ] ++
      for {k, what} <- [pu: "12x17 unsigned", ps: "16x18 signed", pr: "8x8 into a flop"] do
        b = bad.(k)
        {"mul_#{k}", b == [], "#{length(b)} of #{length(rs)} wrong (#{what})#{ex.(b, k)}"}
      end
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {k, ok, d} -> {k, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Mulw do
  @moduledoc false
  # mulw_probe.v: `*` split over 18-bit chunk MULT18X18Ds and adders.
  # pw = x[31:0] * x[63:32] (unsigned, 64 bits), sw = signed x[23:0] *
  # signed x[53:24] (54 bits).
  @magic 0x3A1D0000
  import Bitwise

  defp f(v, lo, w), do: v >>> lo &&& (1 <<< w) - 1
  defp sext(v, w), do: if((v >>> (w - 1) &&& 1) == 1, do: v - (1 <<< w), else: v)

  defp scan(s, x) do
    _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
    tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 192, 16), 64, "0")
    v = Ocd.cmd(s, "drscan ecp5.tap 256 #{tdi}") |> String.trim() |> String.to_integer(16)
    %{sent: x, magic: f(v, 0, 32), x: f(v, 32, 64), pw: f(v, 96, 64), sw: f(v, 160, 54),
      tag: f(v, 224, 8), ncap: f(v, 232, 8)}
  end

  def want(x) do
    %{pw: f(x, 0, 32) * f(x, 32, 32),
      sw: sext(f(x, 0, 24), 24) * sext(f(x, 24, 30), 30) &&& (1 <<< 54) - 1}
  end

  def run(s, o) do
    :rand.seed(:exsss, {59, 61, 67})
    ones = (1 <<< 64) - 1
    edges = [0, ones, 0xFFFFFFFF, 0xFFFFFFFF <<< 32, 1 <<< 23 ||| 1 <<< 53, 0x7FFFFF ||| 0x1FFFFFFF <<< 24,
             0x3FFFF ||| 0x3FFFF <<< 32, 1 <<< 18 ||| 1 <<< 50]
    xs = edges ++ for(_ <- 1..o.samples, do: :rand.uniform(1 <<< 64) - 1)
    reads =
      for x <- xs do
        r = scan(s, x)
        IO.puts(Oracle.line(r))
        r
      end
    check(reads)
  end

  def check(reads) do
    rs = tl(reads)
    bad = fn k -> Enum.filter(rs, &(Map.fetch!(&1, k) != Map.fetch!(want(&1.x), k))) end
    ex = fn
      [r | _], k -> " e.g. x=#{Integer.to_string(r.x, 16)} got #{Integer.to_string(Map.fetch!(r, k), 16)} want #{Integer.to_string(Map.fetch!(want(r.x), k), 16)}"
      [], _ -> ""
    end
    valid = Enum.all?(reads, &(&1.magic == @magic))
    [
      {"magic", valid, "0x" <> Integer.to_string(hd(reads).magic, 16)},
      {"tag", Enum.all?(reads, &(&1.tag == 0xD5)), "#{hd(reads).tag}"},
      {"x_roundtrip", Enum.all?(Enum.zip(rs, reads), fn {r, p} -> r.x == p.sent end), "#{length(rs)} reads"}
    ] ++
      for {k, what} <- [pw: "32x32 unsigned, 4 MULTs", sw: "24x30 signed, 4 MULTs"] do
        b = bad.(k)
        {"mul_#{k}", b == [], "#{length(b)} of #{length(rs)} wrong (#{what})#{ex.(b, k)}"}
      end
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {k, ok, d} -> {k, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Misc do
  @moduledoc false
  # misc_probe.v: ECLKBRIDGECS (SEL x0: clk_25mhz or OSCG) through the
  # bridge ECLKSYNCB and CLKDIVF 2.0 (counter cbr), OSHX2A on gn20 (D0 x1,
  # D1 x2, pad sampled on JTCK), SGSR (GSR ~x3 on clk_25mhz).
  @magic 0x415C0000
  import Bitwise

  defp f(v, lo, w), do: v >>> lo &&& (1 <<< w) - 1
  defp wrap(a, b), do: b - a &&& 0xFFFFFFFF

  defp scan(s, x, label) do
    _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
    tdi = "0x" <> String.pad_leading(Integer.to_string(x <<< 224, 16), 64, "0")
    v = Ocd.cmd(s, "drscan ecp5.tap 256 #{tdi}") |> String.trim() |> String.to_integer(16)
    t = System.monotonic_time(:microsecond)
    %{t_us: t, label: label, sent: x, magic: f(v, 0, 32), x: f(v, 32, 32), c25: Oracle.gray(f(v, 64, 32)),
      cbr: Oracle.gray(f(v, 96, 32)), pz: f(v, 160, 2), tag: f(v, 192, 8), ncap: f(v, 200, 8)}
  end

  def run(s, o) do
    plan =
      for(sel <- [0, 1], _ <- 1..6, do: {sel, "br#{sel}"}) ++
        for(d <- 0..3, _ <- 1..4, do: {d <<< 1, "osh#{d}"}) ++
        [{0, "pre"}, {0x08, "gsr"}, {0, "post"}, {0, "post2"}]
    reads =
      for {x, label} <- plan do
        Process.sleep(div(o.gap_ms, 5))
        r = scan(s, x, label)
        IO.puts(Oracle.line(Map.delete(r, :t_us)))
        r
      end
    check(reads, o)
  end

  def check(reads, o) do
    pairs =
      Enum.zip(reads, tl(reads))
      |> Enum.filter(fn {a, b} -> a.x == b.x and ((a.ncap + 1) &&& 0xFF) == b.ncap end)
    rate = fn {a, b}, k -> wrap(a[k], b[k]) / ((b.t_us - a.t_us) / 1.0e6) end
    mean = fn ps, k -> if ps == [], do: nil, else: Enum.sum(Enum.map(ps, &rate.(&1, k))) / length(ps) end
    by_x = Enum.group_by(pairs, fn {_, b} -> b.x end)
    r25 = mean.(pairs, :c25)
    b0 = mean.(Map.get(by_x, 0, []), :cbr)
    b1 = mean.(Map.get(by_x, 1, []), :cbr)
    osc_half = 310.0e6 / 26 / 2
    fr = fn r -> if r, do: Float.round(r / 1.0e6, 4), else: nil end
    # A read shows the pad samples since the capture before it, mostly
    # from the scan two back: the third and later reads of each setting.
    osh =
      for d <- 0..3 do
        rs = reads |> Enum.filter(&(&1.label == "osh#{d}")) |> Enum.drop(2)
        {d, Enum.map(rs, & &1.pz) |> Enum.uniq()}
      end
    want = %{0 => [1], 3 => [2], 1 => [3], 2 => [3]}
    osh_ok = Enum.all?(osh, fn {d, ps} -> ps == want[d] end)
    i = Enum.find_index(reads, &(&1.label == "gsr"))
    {pre, post} = {Enum.at(reads, i), Enum.at(reads, i + 1)}
    gsr_ok = post.x == 0 and post.ncap <= 1 and post.c25 < pre.c25
    valid = Enum.all?(reads, &(&1.magic == @magic))
    [
      {"magic", valid, "0x" <> Integer.to_string(hd(reads).magic, 16)},
      {"tag", Enum.all?(reads, &(&1.tag == 0xE1)), "#{hd(reads).tag}"},
      {"c25_rate", r25 != nil and abs(r25 / o.clk_hz - 1) < 0.03, "#{fr.(r25)} MHz"},
      {"bridge_sel0", b0 != nil and abs(b0 / (o.clk_hz / 2) - 1) < 0.03, "#{fr.(b0)} MHz (clk_25mhz / 2)"},
      {"bridge_sel1", b1 != nil and abs(b1 / osc_half - 1) < 0.25 and abs(b1 / (o.clk_hz / 2) - 1) > 0.1,
       "#{fr.(b1)} MHz (OSCG / 2, nominal #{fr.(osc_half)})"},
      {"oshx2", osh_ok, Enum.map_join(osh, " ", fn {d, ps} -> "D1D0=#{Integer.to_string(d, 2)}:#{inspect(ps)}" end) <> " (1 low, 2 high, 3 both)"},
      {"sgsr", gsr_ok, "x #{post.x} ncap #{pre.ncap}->#{post.ncap} c25 #{pre.c25}->#{post.c25}"}
    ]
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {k, ok, d} -> {k, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

defmodule Dsp do
  @moduledoc false
  # dsp_probe.v: 18x18 products, unregistered (p0) and registered (p1),
  # and the split modes, all read over the probe's EIO register bus.
  # EIO VERSION register: {major, minor, "IO"}.
  @magic 0x0005494F
  import Bitwise

  defp f(v, lo, w), do: v >>> lo &&& (1 <<< w) - 1
  defp sext(v, w), do: if((v >>> (w - 1) &&& 1) == 1, do: v - (1 <<< w), else: v)

  # One 49-bit EIO scan (ER1 stays selected): returns the captured data word.
  defp dr(s, v) do
    tdi = "0x" <> String.pad_leading(Integer.to_string(v, 16), 13, "0")
    Ocd.cmd(s, "drscan ecp5.tap 49 #{tdi}") |> String.trim() |> String.to_integer(16) |> f(0, 32)
  end

  defp wr(s, a, d), do: dr(s, 1 <<< 48 ||| a <<< 32 ||| d)

  # Reads the registers at `addrs`: each scan selects the next address and
  # captures the one selected before it.
  defp rd(s, [a0 | rest]) do
    _ = dr(s, a0 <<< 32)
    for a <- rest ++ [0], do: dr(s, a <<< 32)
  end

  # VERSION, IN_W and OUT_W, read once per run.
  defp ident(s) do
    [ver, in_w, out_w] = rd(s, [0x0, 0x4, 0x8])
    %{magic: ver, tag: in_w, out_w: out_w}
  end

  defp scan(s, x, id) do
    _ = wr(s, 0x100, f(x, 0, 32))
    _ = wr(s, 0x104, f(x, 32, 32))
    ws = rd(s, [0x10, 0x14, 0x18, 0x1C, 0x20, 0x24])
    v = ws |> Enum.with_index() |> Enum.reduce(0, fn {w, i}, acc -> acc ||| w <<< (32 * i) end)
    %{sent: x, magic: id.magic, tag: id.tag, out_w: id.out_w, x: f(v, 123, 64),
      p0: f(v, 0, 36), p1: f(v, 36, 36), p2: f(v, 72, 18), po: f(v, 90, 9),
      r4: f(v, 99, 24), r4hi: f(v, 115, 8)}
  end

  # The product the MULT18 should give for x: A, B and their signedness.
  def want(x) do
    a = f(x, 0, 18)
    b = f(x, 18, 18)
    a = if f(x, 36, 1) == 1, do: sext(a, 18), else: a
    b = if f(x, 37, 1) == 1, do: sext(b, 18), else: b
    a * b &&& (1 <<< 36) - 1
  end

  # The product the MULT9 should give: 9-bit A and B with their signedness
  # flags at x[54] and x[55].
  def want9(x) do
    a = f(x, 0, 9)
    b = f(x, 9, 9)
    a = if f(x, 54, 1) == 1, do: sext(a, 9), else: a
    b = if f(x, 55, 1) == 1, do: sext(b, 9), else: b
    a * b &&& (1 <<< 18) - 1
  end

  # The pre-adder sum PA+PB at 9 bits (mod 512). PO is the paired
  # multiplier's A operand (dedicated, not fabric-routable): the probe
  # observes constant 0 this revision.
  def want_po(_x), do: 0

  # The ALU24 sum's top byte: r4 = MA + MB, the paired multipliers'
  # unsigned products (m5: x[8:0]*x[17:9], m6: x[26:18]*x[35:27]),
  # CIN grounded per the pairing rule.
  # m5: x[8:0] * x[17:9], m6: x[26:18] * x[35:27]; an operand is signed
  # when its sign pin is high (x[60] m5 A, x[61] m5 B, x[62] m6 A, x[63] m6 B).
  def want_r4(x) do
    op = fn lo, pin -> if f(x, pin, 1) == 1, do: sext(f(x, lo, 9), 9), else: f(x, lo, 9) end
    (op.(0, 60) * op.(9, 61) + op.(18, 62) * op.(27, 63)) &&& 0xFFFFFF
  end
  def want_r4hi(x), do: f(want_r4(x), 16, 8)

  def run(s, o) do
    :rand.seed(:exsss, {31, 37, 41})
    # Edge operands first (signed and unsigned extremes), then random.
    edges = for a <- [0, 1, 0x1FFFF, 0x20000, 0x3FFFF], b <- [0, 1, 0x1FFFF, 0x20000, 0x3FFFF], sg <- 0..3,
                do: a ||| b <<< 18 ||| sg <<< 36
    xs = edges ++ for(_ <- 1..o.samples, do: :rand.uniform(1 <<< 38) - 1)
    # --sign-pins: x[63:60], the m5/m6 SIGNEDA/SIGNEDB pins (m5 A, m5 B, m6 A, m6 B).
    xs = Enum.map(xs, &(&1 ||| o.sign_pins <<< 60))
    _ = Ocd.cmd(s, "irscan ecp5.tap 0x32")
    id = ident(s)
    reads =
      for x <- xs do
        r = scan(s, x, id)
        IO.puts(Oracle.line(Map.put(r, :want_r4, want_r4(r.x))))
        r
      end
    check(reads)
  end

  def check(reads) do
    rs = reads
    bad0 = Enum.filter(rs, &(&1.p0 != want(&1.x)))
    bad1 = Enum.filter(rs, &(&1.p1 != &1.p0))
    sgn = fn r -> f(r.x, 36, 2) end
    by_sign = Enum.group_by(bad0, sgn) |> Enum.map_join(" ", fn {k, v} -> "s#{k}:#{length(v)}" end)
    ex = case bad0 do
      [r | _] -> " e.g. x=#{Integer.to_string(r.x, 16)} got #{Integer.to_string(r.p0, 16)} want #{Integer.to_string(want(r.x), 16)}"
      [] -> ""
    end
    valid = Enum.all?(reads, &(&1.magic == @magic))
    [
      {"magic", valid, "0x" <> Integer.to_string(hd(reads).magic, 16)},
      {"tag", Enum.all?(reads, &(&1.tag == 187 and &1.out_w == 64)), "in #{hd(reads).tag} out #{hd(reads).out_w}"},
      {"x_roundtrip", Enum.all?(rs, &(&1.x == &1.sent)), "#{length(rs)} reads"},
      {"mult18_comb", bad0 == [], "#{length(bad0)} of #{length(rs)} wrong #{by_sign}#{ex}"},
      {"mult18_reg", bad1 == [], "#{length(bad1)} of #{length(rs)} differ from comb"},
      {"mult9_comb", Enum.all?(rs, &(&1.p2 == want9(&1.x))),
       "#{Enum.count(rs, &(&1.p2 != want9(&1.x)))} of #{length(rs)} wrong"},
      {"pradd9", Enum.all?(rs, &(&1.po == want_po(&1.x))),
       "#{Enum.count(rs, &(&1.po != want_po(&1.x)))} of #{length(rs)} wrong"},
      {"alu24", Enum.all?(rs, &(&1.r4hi == want_r4hi(&1.x))),
       "#{Enum.count(rs, &(&1.r4hi != want_r4hi(&1.x)))} of #{length(rs)} wrong (all 24 bits: #{Enum.count(rs, &(&1.r4 != want_r4(&1.x)))} wrong)"}
    ]
    |> Enum.map(fn
      {"magic", ok, d} -> {"magic", ok, d}
      {k, ok, d} -> {k, ok and valid, if(valid, do: d, else: "no valid read; " <> d)}
    end)
  end
end

{o, _, _} =
  OptionParser.parse(System.argv(),
    strict: [design: :string, bit: :string, osc_div: :integer, cfg: :string, samples: :integer, gap_ms: :integer, clk_hz: :float, pll_hz: :float, gp0: :integer, port: :integer, sign_pins: :integer]
  )

o = %{
  cfg: o[:cfg] || "/tmp/ecp5t/ulx3s-ocd.cfg",
  design: o[:design] || "pll",
  samples: o[:samples] || if(o[:design] in ["soft", "lutram", "iol", "gear", "dsp", "alu", "bram", "dqs", "dqsm", "mul", "mulw", "misc"], do: 64, else: 8),
  gap_ms: o[:gap_ms] || 250,
  clk_hz: o[:clk_hz] || 25.0e6,
  pll_hz: o[:pll_hz] || 48.0e6,
  gp0: o[:gp0],
  port: o[:port] || 6667,
  osc_div: o[:osc_div],
  sign_pins: o[:sign_pins] || 0,
  bit: o[:bit]
}

ocd = Ocd.start(o.cfg, o.port)
{_, s} = ocd

if o.bit do
  # SRAM load through the lattice pld driver, then LSC_READ_STATUS (DONE is bit 8).
  # (`pld create` is a configuration command, so ulx3s-ocd.cfg holds it.)
  load = "pld load ecp5.pld {#{o.bit}}"

  IO.puts(Oracle.line(%{load: o.bit, result: Ocd.cmd(s, load, 180_000)}))
  st = Ocd.cmd(s, "irscan ecp5.tap 0x3C; drscan ecp5.tap 32 0") |> String.trim() |> String.to_integer(16)
  IO.puts(Oracle.line(%{status: Integer.to_string(st, 16), done: (st >>> 8) &&& 1}))
  Process.sleep(500)
end

idcode = Ocd.cmd(s, "irscan ecp5.tap 0xE0; drscan ecp5.tap 32 0") |> String.trim()
IO.puts(Oracle.line(%{idcode: idcode}))

results =
  case o.design do
    "soft" ->
      r = Soft.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    "lutram" ->
      r = Lutram.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    "iol" ->
      r = Iol.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    "clk" ->
      r = Clk.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    "gear" ->
      r = Gear.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    "mul" ->
      r = Mul.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    "mulw" ->
      r = Mulw.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    "misc" ->
      r = Misc.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    "dsp" ->
      r = Dsp.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    "alu" ->
      r = Alu.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    "bram" ->
      r = Bram.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    "pad" ->
      r = Pad.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    d when d in ["dqs", "dqsm"] ->
      r = Dqs.run(s, o)
      Ocd.stop(ocd)
      [{"idcode", idcode == "41113043", idcode} | r]

    _ ->
      xs =
        for i <- 1..o.samples do
          if i > 1, do: Process.sleep(o.gap_ms)
          x = Oracle.read(s, 0xC000 + i)
          IO.puts(Oracle.line(Map.delete(x, :t_us)))
          x
        end

      Ocd.stop(ocd)
      Oracle.checks(xs, Map.put(o, :idcode, idcode))
  end

for {name, ok, detail} <- results do
  IO.puts("#{if ok, do: "PASS", else: "FAIL"} #{String.pad_trailing(name, 11)} #{detail}")
end

fails = Enum.count(results, fn {_, ok, _} -> not ok end)
IO.puts("oracle: #{length(results) - fails}/#{length(results)} checks passed")
System.halt(if fails == 0, do: 0, else: 1)
