# gs/ — ground station (not built in v1)

Reserved for the GS daemon. It will link `libmabur_common` (RC codec, FEC
decode, profile/ladder/probe rules) so drone and GS cannot drift. Until it
exists, `tools/bench/` (devourer duplex + Python decoders) is the proto-GS.
