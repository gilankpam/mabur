add_library(mabur_gs_core STATIC
  src/config.cpp
  src/aggregator.cpp
  src/udp_sink.cpp
  src/frame_file_source.cpp
  src/rendezvous.cpp
  src/vrx_controller.cpp
  src/ladder_controller.cpp
  src/rung_store.cpp
  src/ctl_log.cpp
  src/debug_session.cpp
  src/probe_log.cpp
  src/ladder_residual.cpp
  src/tx_selector.cpp
  src/rcf_slot.cpp
  src/msp_sink.cpp
  src/frame_stream.cpp
  src/au_ring.cpp
  src/au_doorbell.cpp
  src/probe_track.cpp
  src/rtt_estimator.cpp
  src/stats_exporter.cpp)
target_include_directories(mabur_gs_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
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

add_library(mabur_gs_radio STATIC src/radio_frontend.cpp)
target_include_directories(mabur_gs_radio PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mabur_gs_radio PUBLIC mabur_gs_core mabur_common devourer)

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
