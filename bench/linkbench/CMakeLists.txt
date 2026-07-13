# linkbench — iperf-like link bench (spec: docs/superpowers/specs/
# 2026-07-13-linkbench-design.md). linkbench_core is header-only and
# devourer-free so the whole pipeline is host-testable; the two binaries
# (added below when devourer is in the build) wire it to the radio.
add_library(linkbench_core INTERFACE)
target_include_directories(linkbench_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(linkbench_core INTERFACE mabur_common)
