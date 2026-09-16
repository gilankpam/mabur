# aarch64 glibc cross toolchain file: the Buildroot SDK that builds the GS
# image (../sbc-groundstations-gilankpam, radxa_zero3_defconfig). Used for
# maburplay ONLY (tools/build-arm64.sh stage 6): its burned-DVR colortrans
# stage links Mesa EGL/GLESv2/GBM + librga, which exist only as glibc shared
# objects, so maburplay is glibc-dynamic from 2026-09-16 on. maburgs stays on
# cmake/aarch64-musl.cmake (static). See docs/colortrans.md.
#
# MABUR_BR_HOST = <buildroot output>/host, exported by build-arm64.sh.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(_br_host "$ENV{MABUR_BR_HOST}")
if(NOT _br_host)
  message(FATAL_ERROR
    "aarch64-buildroot.cmake: export MABUR_BR_HOST=<sbc-groundstations output>/host "
    "(tools/build-arm64.sh does this).")
endif()
set(CMAKE_C_COMPILER "${_br_host}/bin/aarch64-none-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "${_br_host}/bin/aarch64-none-linux-gnu-g++")
set(CMAKE_SYSROOT "${_br_host}/aarch64-buildroot-linux-gnu/sysroot")

set(CMAKE_FIND_ROOT_PATH "${CMAKE_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# The Buildroot external toolchain's kernel headers predate the v2 GPIO uAPI
# rec_button.cpp speaks; package/mabur/mabur.mk force-includes the same
# guarded fallback. Inert once the toolchain's headers catch up.
set(CMAKE_C_FLAGS_INIT "-include ${CMAKE_CURRENT_LIST_DIR}/gpio_v2_compat.h")
set(CMAKE_CXX_FLAGS_INIT "-include ${CMAKE_CURRENT_LIST_DIR}/gpio_v2_compat.h")
