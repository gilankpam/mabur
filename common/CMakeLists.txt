add_library(mabur_common STATIC src/crc16.cpp src/gf256.cpp src/sbi.cpp src/nal.cpp src/frag.cpp src/uep_encoder.cpp src/uep_decoder.cpp src/rc_proto.cpp src/profile.cpp src/node.cpp src/sw_wire.cpp src/sw_encoder.cpp src/sw_decoder.cpp src/msp_dp.cpp src/msp_source.cpp src/msp_status.cpp src/fec_worker.cpp src/frame_wire.cpp src/probe_wire.cpp src/toml.cpp)

# Speed, not size, for the hot path (frag -> SwEncoder -> SbiPacker -> CRC):
# Buildroot's toolchain file pins CMAKE_CXX_FLAGS to -Os and empties the
# Release flags, so the image-builder maburd was -Os while tools/build-arm.sh
# was -O3 -- an 8.5 % difference on the drone's producer thread, measured
# 2026-09-22 (docs/bitrate-ceiling-findings-2026-09-21.md). A per-target
# option comes after CMAKE_CXX_FLAGS on the command line, so it wins under
# either builder. Debug builds keep their -O0.
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(mabur_common PRIVATE -O3)
endif()
target_include_directories(mabur_common PUBLIC include)
find_package(Threads REQUIRED)
target_link_libraries(mabur_common PUBLIC Threads::Threads)

# ARMv7 (drone SSC338Q, Cortex-A7 = NEON-VFPv4): the cross stdenv's default
# fpu doesn't define __ARM_NEON__, which gates gf256's vtbl lincomb — the
# ~10x path that keeps sliding-window repair generation off the hot thread's
# critical budget. AArch64 has ASIMD unconditionally.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
  set_source_files_properties(src/gf256.cpp PROPERTIES
    COMPILE_OPTIONS "-mfpu=neon-vfpv4")
endif()
