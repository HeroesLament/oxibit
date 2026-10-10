# Cross-check `oxibit convert --to swapped-bin` against ehdl's
# Hw.Xilinx.Bit2Bin.to_bin!/1, byte for byte, on every .bit under a tree.
#
# Run from the ehdl checkout, so Hw.Xilinx.Bit2Bin is loaded:
#
#   cd ~/src/ehdl
#   mix run ~/src/oxibit/scripts/crosscheck_bit2bin.exs [ROOT] [OXIBIT]
#
# ROOT defaults to designs/, OXIBIT to ~/src/oxibit/target/release/oxibit.

[root, oxibit] =
  case System.argv() do
    [r, o] -> [r, o]
    [r] -> [r, Path.expand("~/src/oxibit/target/release/oxibit")]
    [] -> ["designs", Path.expand("~/src/oxibit/target/release/oxibit")]
  end

tmp = Path.join(System.tmp_dir!(), "oxibit_crosscheck.bin")
bits = Path.wildcard(Path.join(root, "**/*.bit"))

results =
  for bit <- bits do
    elixir =
      try do
        {:ok, Hw.Xilinx.Bit2Bin.to_bin!(File.read!(bit))}
      rescue
        e -> {:error, Exception.message(e)}
      end

    rust =
      case System.cmd(oxibit, ["convert", bit, tmp], stderr_to_stdout: true) do
        {_, 0} -> {:ok, File.read!(tmp)}
        {out, _} -> {:error, String.trim(out)}
      end

    verdict =
      case {elixir, rust} do
        {{:ok, a}, {:ok, b}} when a == b -> :same
        {{:ok, _}, {:ok, _}} -> :DIFFERENT
        {{:error, _}, {:error, _}} -> :both_reject
        _ -> :DISAGREE
      end

    IO.puts("#{verdict}\t#{bit}")
    if verdict in [:DIFFERENT, :DISAGREE], do: IO.puts("\telixir=#{inspect(elixir, limit: 3)}\n\trust=#{inspect(rust, limit: 3)}")
    verdict
  end

IO.puts("\n#{length(bits)} files: " <> inspect(Enum.frequencies(results)))
