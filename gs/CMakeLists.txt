add_library(mabur_gs_core STATIC
  src/channel_plan.cpp
  src/boot_pick.cpp
  src/channel_ranker.cpp
  src/hop_verdict.cpp
  src/hop_ranker.cpp
  src/hop_controller.cpp
  src/channel_scout.cpp
  src/pair_pick.cpp
  src/inflight_scout.cpp
  src/channel_core.cpp
  src/config.cpp
  src/card_scan.cpp
  src/aggregator.cpp
  src/udp_sink.cpp
  src/frame_file_source.cpp
  src/rendezvous.cpp
  src/vrx_controller.cpp
  src/ladder_controller.cpp
  src/rung_store.cpp
  src/ctl_log.cpp
  src/scan_log.cpp
  src/cal_analysis.cpp
  src/cal_plan.cpp
  src/cal_session.cpp
  src/cal_log.cpp
  src/debug_session.cpp
  src/log_writer.cpp
  src/probe_log.cpp
  src/fec_log.cpp
  src/ladder_residual.cpp
  src/tx_selector.cpp
  src/rcf_slot.cpp
  src/msp_sink.cpp
  src/frame_stream.cpp
  src/au_ring.cpp
  src/au_log.cpp
  src/au_doorbell.cpp
  src/probe_track.cpp
  src/link_health.cpp
  src/rtt_estimator.cpp
  src/stats_exporter.cpp
  src/stats_sink.cpp)
target_include_directories(mabur_gs_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
# chanmig (ScanScheduler, SurveyDwell) is header-only and pure; no link.
target_include_directories(mabur_gs_core PUBLIC ${DEVOURER_DIR}/src)
target_link_libraries(mabur_gs_core PUBLIC mabur_common)

# shm_open/mmap need librt on glibc hosts; the musl aarch64 cross-build folds
# it into libc and has no separate librt. NOT CMAKE_CROSSCOMPILING is required:
# find_library otherwise resolves the HOST glibc librt.so and tries to
# static-link that (wrong-arch, dynamic) into the musl binary, which fails
# with "attempted static link of dynamic object".
find_library(RT_LIB rt)
if(RT_LIB AND NOT CMAKE_CROSSCOMPILING)
  target_link_libraries(mabur_gs_core PUBLIC ${RT_LIB})
endif()

# Pure dot11 helpers (build_control_frame, fill_rx_body): shared with the
# web GS (web/), which cannot link radio_frontend's threads/libusb device code.
add_library(mabur_gs_dot11 STATIC src/dot11.cpp)
target_include_directories(mabur_gs_dot11 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mabur_gs_dot11 PUBLIC mabur_common devourer)

# mabur-relay v3 client core (CPE510 remote card): pure codec + state machine,
# shared by the web GS and (later) maburgs' RemoteCard.
add_library(mabur_gs_relay STATIC src/relay_wire.cpp src/relay_client.cpp src/relay_transport.cpp)
target_include_directories(mabur_gs_relay PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mabur_gs_relay PUBLIC mabur_gs_dot11 mabur_common)

add_library(mabur_gs_radio STATIC src/radio_frontend.cpp src/card_scan_usb.cpp src/remote_card.cpp)
target_include_directories(mabur_gs_radio PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mabur_gs_radio PUBLIC mabur_gs_core mabur_gs_dot11 mabur_gs_relay mabur_common devourer)

add_executable(maburgs src/main.cpp)
target_include_directories(maburgs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(maburgs PRIVATE mabur_gs_core mabur_gs_radio mabur_common pthread)

# Bench per-stream Gilbert-Elliott loss injector (gs/src/loss_sim.h,
# loss_control.h), mainlined 2026-08-29 behind this flag instead of living on
# an unmerged branch. Default OFF: prod maburgs contains none of it. main.cpp
# compiles as part of the maburgs target itself, but aggregator.cpp (the
# other hook site) lives in mabur_gs_core, so the definition goes on that
# library PUBLIC -- every consumer (maburgs, and any test linking
# mabur_gs_core, e.g. test_loss_sim) must see the same macro state as
# aggregator.h/.cpp were compiled with, or Aggregator's layout disagrees
# across translation units.
option(MABUR_LOSS_SIM "Compile the bench per-stream loss injector into maburgs (never for prod)" OFF)
if(MABUR_LOSS_SIM)
  target_compile_definitions(mabur_gs_core PUBLIC MABUR_LOSS_SIM)
endif()

# In-flight hop e2e injection seam (Task 15, 2026-09-14-inflight-channel-hop):
# MABUR_HOP_INJECT, read only under this macro, drives gs_e2e's hop scenario
# (tests/integration/run_gs_e2e.sh) through the real dry-run binary. Follows
# the MABUR_LOSS_SIM precedent above -- PUBLIC on mabur_gs_core so main.cpp
# (compiled directly into the maburgs target, not the library) sees it too.
# Auto-ON whenever tests are being built (MABUR_BUILD_TESTS, default ON) so
# `ctest -R gs_e2e` works out of the box; every device/cross build passes
# -DMABUR_BUILD_TESTS=OFF explicitly (tools/build-arm.sh, build-arm64.sh),
# so this is never compiled into a shipped binary. The explicit option
# still exists so it can be forced independently of MABUR_BUILD_TESTS.
option(MABUR_TEST "Compile the in-flight hop e2e injection seam into maburgs (never for prod)" OFF)
if(MABUR_BUILD_TESTS)
  set(MABUR_TEST ON)
endif()
if(MABUR_TEST)
  target_compile_definitions(mabur_gs_core PUBLIC MABUR_TEST)
endif()
