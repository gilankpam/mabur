# ARM (armv7-a, hard-float) musl cross-compilation toolchain file.
#
# Deviation from the original plan: instead of the Bootlin prebuilt toolchain
# (which is dynamically linked against glibc's /lib64/ld-linux-x86-64.so.2 and
# therefore cannot even exec on a NixOS host — there is no FHS loader path),
# this file is toolchain-agnostic: it takes the compiler executables from
# environment variables exported by tools/build-arm.sh, which on this host
# resolve to Nix's own cross stdenv:
#   pkgsCross.armv7l-hf-multiplatform.pkgsStatic.stdenv.cc
# (target triple armv7l-unknown-linux-musleabihf). Nothing below is
# Bootlin-specific or Nix-specific — any arm-*-musl*-gcc/g++ pair works as
# long as MABUR_CROSS_CC / MABUR_CROSS_CXX (or CC / CXX) point at it.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Compiler: prefer explicit MABUR_CROSS_CC/CXX, then fall back to CC/CXX, so
# this file works whether build-arm.sh exports the Mabur-specific names or a
# caller just did `CC=... CXX=... cmake -DCMAKE_TOOLCHAIN_FILE=...`.
if(DEFINED ENV{MABUR_CROSS_CC})
  set(CMAKE_C_COMPILER "$ENV{MABUR_CROSS_CC}")
else()
  set(CMAKE_C_COMPILER "$ENV{CC}")
endif()
if(DEFINED ENV{MABUR_CROSS_CXX})
  set(CMAKE_CXX_COMPILER "$ENV{MABUR_CROSS_CXX}")
else()
  set(CMAKE_CXX_COMPILER "$ENV{CXX}")
endif()

if(NOT CMAKE_C_COMPILER OR NOT CMAKE_CXX_COMPILER)
  message(FATAL_ERROR
    "arm-musl.cmake: no cross compiler found. Export MABUR_CROSS_CC/"
    "MABUR_CROSS_CXX (or CC/CXX) to the arm-*-musl*-gcc/g++ pair before "
    "configuring, e.g. via tools/build-arm.sh.")
endif()

# CMAKE_FIND_ROOT_PATH: where to look for target libraries (libusb's static
# archive + headers), staged by build-arm.sh under toolchain/staging via a
# `nix-build` GC root — NOT the Bootlin buildroot --host-copied staging tree
# the original plan assumed. libusb itself is not built from source in this
# environment; see build-arm.sh for why (pkgsCross ... pkgsStatic.libusb1
# already provides a static armv7l/musl libusb).
#
# LIBRARY/INCLUDE mode is BOTH rather than the original plan's ONLY: Nix
# doesn't stage packages into a single FHS-shaped sysroot (each package is
# its own /nix/store/<hash>-name path), so devourer's
# pkg_check_modules(libusb ... IMPORTED_TARGET libusb-1.0) resolves
# libusb_LIBRARY_DIRS/libusb_INCLUDEDIR to those absolute out-of-root store
# paths directly. Under ONLY mode, find_library()/find_path() silently drop
# any candidate path outside CMAKE_FIND_ROOT_PATH instead of falling back to
# it verbatim, so pkg-config's correct absolute paths get thrown away and
# the imported target ends up with a bare `-lusb-1.0` (no -L) that the
# linker can't resolve. BOTH still root-prefixes the toolchain/staging
# convenience symlinks for anything that *does* do a relative/system lookup,
# while also trying paths as given.
set(CMAKE_FIND_ROOT_PATH "$ENV{MABUR_STAGING}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)  # BOTH is safe on NixOS (no populated /usr/lib) but risks host-library contamination on FHS distros
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)

set(CMAKE_EXE_LINKER_FLAGS_INIT "-static")
