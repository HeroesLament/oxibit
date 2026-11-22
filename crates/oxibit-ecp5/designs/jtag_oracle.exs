# JTAG oracle for oxibit ECP5 test designs on the ULX3S.
#
# Reads the design's ER1 data register (IR 0x32, 128 bits, see
# pll_jtag.v) through openocd's TCL port and checks it against what the
# design must show. One JSON line per sample, then one line per check,
# then a summary. Exit status 0 when every check passes, 1 otherwise.
#
#   elixir jtag_oracle.exs [--cfg F] [--samples N] [--gap-ms MS]
#       [--clk-hz HZ] [--pll-hz HZ] [--gp0 0|1] [--bit F.bit]
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

{o, _, _} =
  OptionParser.parse(System.argv(),
    strict: [bit: :string, cfg: :string, samples: :integer, gap_ms: :integer, clk_hz: :float, pll_hz: :float, gp0: :integer, port: :integer]
  )

o = %{
  cfg: o[:cfg] || "/tmp/ecp5t/ulx3s-ocd.cfg",
  samples: o[:samples] || 8,
  gap_ms: o[:gap_ms] || 250,
  clk_hz: o[:clk_hz] || 25.0e6,
  pll_hz: o[:pll_hz] || 48.0e6,
  gp0: o[:gp0],
  port: o[:port] || 6667,
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

xs =
  for i <- 1..o.samples do
    if i > 1, do: Process.sleep(o.gap_ms)
    x = Oracle.read(s, 0xC000 + i)
    IO.puts(Oracle.line(Map.delete(x, :t_us)))
    x
  end

Ocd.stop(ocd)

results = Oracle.checks(xs, Map.put(o, :idcode, idcode))

for {name, ok, detail} <- results do
  IO.puts("#{if ok, do: "PASS", else: "FAIL"} #{String.pad_trailing(name, 11)} #{detail}")
end

fails = Enum.count(results, fn {_, ok, _} -> not ok end)
IO.puts("oracle: #{length(results) - fails}/#{length(results)} checks passed")
System.halt(if fails == 0, do: 0, else: 1)
