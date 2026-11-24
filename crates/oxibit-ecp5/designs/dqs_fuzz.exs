# DQS read-side silicon fuzzer for the ULX3S-85F (SRAM loads only).
#
# Takes the routed dqs_probe config, injects Trellis-unmapped bits into the
# four DQS tiles of the group at R15..R18 C126, packs each variant with
# ecp5_pack --compress and runs the JTAG oracle on it. A variant that makes
# DATAVALID or BURSTDET fire is a hit.
#
# Usage (from ~/src/oxibit):
#   elixir crates/oxibit-ecp5/designs/dqs_fuzz.exs --plan baseline
#   elixir crates/oxibit-ecp5/designs/dqs_fuzz.exs --plan tiles
#   elixir crates/oxibit-ecp5/designs/dqs_fuzz.exs --plan single --tile DQS0
#   elixir crates/oxibit-ecp5/designs/dqs_fuzz.exs --plan set --bits DQS0:F2B10,DQS3:F4B11
#
# Results append to /tmp/ecp5s/fuzz/summary.txt, one line per variant.

defmodule DqsFuzz do
  @base "/tmp/ecp5s/dqs_probe.config"
  @out "/tmp/ecp5s/fuzz"
  @pack "target/release/examples/ecp5_pack"
  @oracle "crates/oxibit-ecp5/designs/jtag_oracle.exs"

  @tiles %{
    "DQS0" => "MIB_R15C126:PICR1_DQS0",
    "DQS1" => "MIB_R16C126:PICR2_DQS1",
    "DQS2" => "MIB_R17C126:PICR0_DQS2",
    "DQS3" => "MIB_R18C126:PICR1_DQS3"
  }

  # Bits with no entry in the Trellis bits.db of each tile type
  # (10 frames x 12 bits per tile), B10/B11 columns first.
  @unmapped %{
    "DQS0" => ~w(F0B10 F0B11 F1B10 F1B11 F2B10 F2B11 F3B10 F3B11 F4B10 F4B11
                 F5B10 F5B11 F6B10 F7B10 F8B10 F8B11 F9B10 F9B11
                 F0B5 F0B9 F1B9 F2B4 F3B3 F4B3 F4B9 F5B8 F6B2 F6B8 F8B0 F8B1
                 F8B7 F9B0 F9B6),
    "DQS1" => ~w(F1B8 F2B8 F2B9 F3B8 F4B3 F4B8 F5B3 F5B4 F6B1 F7B7 F8B0 F8B5
                 F9B0 F9B2),
    "DQS2" => ~w(F6B10 F2B8 F3B8 F3B9 F4B3 F4B8 F5B3 F5B4 F6B3 F7B7 F8B0 F8B5
                 F9B0 F9B2),
    "DQS3" => ~w(F0B11 F1B11 F2B10 F2B11 F3B10 F3B11 F4B10 F4B11 F5B10 F5B11
                 F6B10 F7B10 F8B10 F9B10 F9B11
                 F0B5 F0B9 F1B9 F2B4 F3B3 F4B3 F4B9 F5B8 F6B2 F6B8 F8B0 F8B1
                 F8B7 F9B0 F9B6)
  }

  def main(argv) do
    {opts, _, _} =
      OptionParser.parse(argv, strict: [plan: :string, tile: :string, bits: :string])

    File.mkdir_p!(@out)
    base = File.read!(@base)

    variants =
      case opts[:plan] do
        "baseline" -> [{"baseline", []}]
        "tiles" -> tile_groups()
        "single" -> singles(opts[:tile])
        "all" -> [{"all", for(t <- ~w(DQS0 DQS1 DQS2 DQS3), b <- @unmapped[t], do: {t, b})}]
        "hi_singles" -> for(t <- ~w(DQS0 DQS1 DQS2 DQS3), b <- @unmapped[t], hi?(b), do: {"#{t}_#{b}", [{t, b}]})
        "set" -> [{"set", parse_bits(opts[:bits])}]
        other -> raise "unknown --plan #{inspect(other)}"
      end

    for {name, bits} <- variants, do: run(base, name, bits)
  end

  # One variant per tile: its B10/B11 candidates, then the rest.
  defp tile_groups do
    for t <- ~w(DQS0 DQS1 DQS2 DQS3), {label, sel} <- [{"hi", &hi?/1}, {"lo", &(not hi?(&1))}],
        bits <- [Enum.filter(@unmapped[t], sel)],
        bits != [] do
      {"#{t}_#{label}", Enum.map(bits, &{t, &1})}
    end
  end

  defp singles(t) when is_map_key(@unmapped, t),
    do: for(b <- @unmapped[t], do: {"#{t}_#{b}", [{t, b}]})

  defp singles(t), do: raise("--tile must be one of DQS0..DQS3, got #{inspect(t)}")

  defp hi?(bit), do: String.ends_with?(bit, ["B10", "B11"])

  defp parse_bits(s) do
    for item <- String.split(s || "", ",", trim: true) do
      [t, b] = String.split(item, ":")
      unless b in Map.fetch!(@unmapped, t), do: raise("#{item} is not an unmapped candidate")
      {t, b}
    end
  end

  defp run(base, name, bits) do
    cfg = Path.join(@out, "#{name}.config")
    bit = Path.join(@out, "#{name}.bit")
    File.write!(cfg, inject(base, bits))
    IO.puts("== #{name} (#{length(bits)} bits)")

    {pack_out, 0} = System.cmd(Path.expand(@pack), ["--compress", cfg, bit], stderr_to_stdout: true)
    IO.write(pack_out)
    IO.puts("bit size #{File.stat!(bit).size} bytes")

    t0 = System.monotonic_time(:millisecond)
    port = Port.open({:spawn_executable, System.find_executable("elixir")},
      [:binary, :exit_status, :stderr_to_stdout, args: [@oracle, "--design", "dqs", "--bit", bit]])
    lines = collect(port, "", [])
    secs = div(System.monotonic_time(:millisecond) - t0, 1000)

    dv = Enum.find(lines, "", &String.contains?(&1, "datavalid"))
    hit = dv =~ ~r/[1-9][0-9]* of 64/ or dv =~ ~r/burstdet [1-9]/

    line = "#{name} bits=#{fmt(bits)} secs=#{secs} hit=#{hit} | #{String.trim(dv)}"
    File.write!(Path.join(@out, "summary.txt"), line <> "\n", [:append])
    IO.puts(if hit, do: "HIT #{name}", else: "miss #{name}")
  end

  defp collect(port, acc, lines) do
    receive do
      {^port, {:data, d}} ->
        IO.write(d)
        {done, rest} = split(acc <> d)
        collect(port, rest, lines ++ done)

      {^port, {:exit_status, _}} ->
        lines ++ if(acc == "", do: [], else: [acc])
    end
  end

  defp split(s) do
    parts = String.split(s, "\n")
    {Enum.drop(parts, -1), List.last(parts)}
  end

  defp fmt([]), do: "-"
  defp fmt(bits), do: Enum.map_join(bits, ",", fn {t, b} -> "#{t}:#{b}" end)

  # Add `unknown: FxBy` lines right after each tile header.
  defp inject(base, bits) do
    by_tile = Enum.group_by(bits, &elem(&1, 0), &elem(&1, 1))

    Enum.reduce(by_tile, base, fn {t, bs}, acc ->
      hdr = ".tile " <> Map.fetch!(@tiles, t)
      unless String.contains?(acc, hdr <> "\n"), do: raise("#{hdr} not in base config")
      extra = Enum.map_join(bs, "", &"unknown: #{&1}\n")
      String.replace(acc, hdr <> "\n", hdr <> "\n" <> extra, global: false)
    end)
  end
end

DqsFuzz.main(System.argv())
