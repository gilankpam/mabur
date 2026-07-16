# txagcbench — TXAGC gain-curve validation pair (spec: docs/superpowers/
# specs/2026-07-16-txagcbench-design.md). tx sweeps SetTxPowerIndexOverride
# and stamps frames with the index; rx records per-frame RSSI as JSONL;
# analyze_sweep.py (host, no build step) judges shape vs kTxagcGainDb.
if(TARGET devourer)
  add_executable(txagcbench-tx tx_main.cpp)
  target_link_libraries(txagcbench-tx PRIVATE devourer pthread)
endif()
