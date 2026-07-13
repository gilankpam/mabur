add_library(mabur_common STATIC src/crc16.cpp src/gf256.cpp src/rs_encoder.cpp src/rs_decoder.cpp src/sbi.cpp src/nal.cpp src/frag.cpp src/frag_reassembler.cpp src/uep_encoder.cpp src/uep_decoder.cpp src/rc_proto.cpp src/profile.cpp src/node.cpp)
target_include_directories(mabur_common PUBLIC include)

# ARMv7 (drone SSC338Q, Cortex-A7 = NEON-VFPv4): the cross stdenv's default
# fpu doesn't define __ARM_NEON__, which gates gf256's vtbl lincomb — the
# ~10x path that keeps RS parity generation off the hot thread's critical
# budget. AArch64 has ASIMD unconditionally.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "arm")
  set_source_files_properties(src/gf256.cpp PROPERTIES
    COMPILE_OPTIONS "-mfpu=neon-vfpv4")
endif()
