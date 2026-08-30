#!/usr/bin/env bash
# Cross-build maburgs for ARM (aarch64) / musl, fully static.
#
# Deviations from the original Bootlin-based plan (see
# .superpowers/sdd/task-16-brief.md and task-16-report.md for the full
# rationale): this host is NixOS, where prebuilt toolchains like Bootlin's
# are dynamically linked against /lib64/ld-linux-x86-64.so.2 — a path that
# does not exist on NixOS, so the downloaded compiler cannot even execute.
# Instead this script uses Nix's own cross stdenv
# (pkgsCross.aarch64-multiplatform.pkgsStatic), which targets
# aarch64-unknown-linux-musl and works natively on NixOS. It also
# provides a prebuilt *static* libusb1 for that target, so — unlike the
# draft — this script does NOT configure/build libusb from source; it just
# resolves the Nix package and points pkg-config at it.
#
# Both the compiler and libusb are realized via `nix-build` with an output
# symlink under toolchain/ (gitignored). That symlink is a GC root, so the
# store paths survive `nix-collect-garbage` between runs and re-running this
# script is a fast no-op (nix-build short-circuits, cmake/make do incremental
# rebuilds).
set -euo pipefail
cd "$(dirname "$0")/.."

mkdir -p toolchain out/arm64

TARGET_TRIPLE=aarch64-unknown-linux-musl

# --- 1. Cross compiler (Nix's musl-static aarch64 stdenv) ---------
if [ ! -e toolchain/cc-arm64 ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkgsCross.aarch64-multiplatform.pkgsStatic.stdenv.cc' \
    -o toolchain/cc-arm64
fi
CC_BINDIR="$(readlink -f toolchain/cc-arm64)/bin"
export MABUR_CROSS_CC="$CC_BINDIR/${TARGET_TRIPLE}-gcc"
export MABUR_CROSS_CXX="$CC_BINDIR/${TARGET_TRIPLE}-g++"
export PATH="$CC_BINDIR:$PATH"

if [ ! -x "$MABUR_CROSS_CC" ]; then
  echo "error: expected cross gcc at $MABUR_CROSS_CC — tuple prefix may have" >&2
  echo "changed upstream; check $CC_BINDIR for the actual *-gcc name and fix" >&2
  echo "TARGET_TRIPLE above." >&2
  exit 1
fi

# --- 1b. pkg-config itself. Not present on a bare NixOS PATH outside a
#         nix-shell (mirrors why the host build needs
#         `nix-shell -p pkg-config libusb1`); staged the same way as the
#         cross compiler above so this script is self-contained.
if [ ! -e toolchain/pkg-config ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkg-config' \
    -o toolchain/pkg-config
fi
PKG_CONFIG_BINDIR="$(readlink -f toolchain/pkg-config)/bin"
export PATH="$PKG_CONFIG_BINDIR:$PATH"

# --- 2. Static libusb for the same target (prebuilt by Nix, not built from
#        source: pkgsCross...pkgsStatic.libusb1 already ships a static
#        libusb-1.0.a + libusb-1.0.pc for aarch64-unknown-linux-musl).
# NB: `nix-build -o toolchain/libusb-out` on a multi-output derivation's
# `.dev` output always *appends* "-dev" to whatever symlink name is given
# (so "-o toolchain/libusb-out" for the .dev output actually creates
# toolchain/libusb-out-dev, not toolchain/libusb-out) — hence reading back
# "toolchain/libusb-out-dev" below rather than the name passed to -o.
if [ ! -e toolchain/libusb-arm64-out ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkgsCross.aarch64-multiplatform.pkgsStatic.libusb1' \
    -o toolchain/libusb-arm64-out
fi
if [ ! -e toolchain/libusb-arm64-out-dev ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkgsCross.aarch64-multiplatform.pkgsStatic.libusb1.dev' \
    -o toolchain/libusb-arm64-out
fi

# CMAKE_FIND_ROOT_PATH staging dir: a minimal fake sysroot with lib/ and
# include/ symlinked straight at the two Nix libusb outputs. Also see
# cmake/aarch64-musl.cmake for why CMAKE_FIND_ROOT_PATH_MODE_{LIBRARY,INCLUDE}
# is BOTH, not the original plan's ONLY: pkg-config resolves libusb's
# location to its real (out-of-root) /nix/store path, and ONLY mode drops
# any find_library()/find_path() candidate outside CMAKE_FIND_ROOT_PATH
# instead of falling back to it verbatim — silently losing the -L flag.
export MABUR_STAGING="$PWD/toolchain/staging"
mkdir -p "$MABUR_STAGING"
ln -sfn "$(readlink -f toolchain/libusb-arm64-out)/lib" "$MABUR_STAGING/lib"
ln -sfn "$(readlink -f toolchain/libusb-arm64-out-dev)/include" "$MABUR_STAGING/include"

# Unset any host PKG_CONFIG_LIBDIR/PATH from the environment so devourer's
# pkg_check_modules(libusb REQUIRED IMPORTED_TARGET libusb-1.0) cannot
# resolve against the host's libusb (e.g. from `nix-shell -p libusb1` used
# for the host build) instead of the staged aarch64 static one.
unset PKG_CONFIG_PATH || true
PKG_CONFIG_LIBDIR="$(readlink -f toolchain/libusb-arm64-out-dev)/lib/pkgconfig"
export PKG_CONFIG_LIBDIR
export PKG_CONFIG_SYSROOT_DIR=""

# devourer's UsbOpen.cpp/UsbDeviceLock.cpp do
# `#include <libusb-1.0/libusb.h>` on Linux, which needs the *parent*
# include/ dir on the search path — not include/libusb-1.0, which is what
# libusb-1.0.pc's `Cflags:` alone provides (-I.../include/libusb-1.0, so a
# plain `#include <libusb.h>` resolves but the versioned-subdir spelling
# doesn't). On the host build this second path comes for free: Nix's
# cc-wrapper auto-injects `-isystem <pkg>/include` for every package pulled
# in via `nix-shell -p libusb1` (see its NIX_CFLAGS_COMPILE). We're driving
# the cross gcc directly via `nix-build` instead of `nix-shell -p`, so that
# auto-injection doesn't happen — CPATH below reproduces it explicitly.
CPATH="$(readlink -f toolchain/libusb-arm64-out-dev)/include${CPATH:+:$CPATH}"
export CPATH

# --- 3. Static libdrm for the same target (prebuilt by Nix, not built from
#        source: pkgsCross...pkgsStatic.libdrm already ships a static
#        libdrm.a + headers for aarch64-unknown-linux-musl — same reasoning
#        and the same out/.dev-suffix quirk as the libusb block above). Used
#        by maburplay's DrmPresenter (MABUR_PLAYER_HW, Task 9).
if [ ! -e toolchain/libdrm-arm64-out ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkgsCross.aarch64-multiplatform.pkgsStatic.libdrm' \
    -o toolchain/libdrm-arm64-out
fi
if [ ! -e toolchain/libdrm-arm64-out-dev ]; then
  nix-build -E \
    'with import <nixpkgs> {}; pkgsCross.aarch64-multiplatform.pkgsStatic.libdrm.dev' \
    -o toolchain/libdrm-arm64-out
fi
# Merge the two Nix outputs into one MABUR_DRM_ROOT-shaped dir (lib/ + include/
# symlinks), the same convenience-staging trick as MABUR_STAGING above, so
# gs/player/CMakeLists.txt's MABUR_DRM_ROOT can address both with a single
# cache var instead of the raw out-of-root /nix/store paths.
mkdir -p toolchain/drm-arm64
ln -sfn "$(readlink -f toolchain/libdrm-arm64-out)/lib" toolchain/drm-arm64/lib
ln -sfn "$(readlink -f toolchain/libdrm-arm64-out-dev)/include" toolchain/drm-arm64/include
export MABUR_DRM_ROOT="$PWD/toolchain/drm-arm64"

# --- 4. rockchip-mpp, built from source (no nixpkgs package exists for it).
#        Clone (skip if already present) then configure/build with the same
#        cross cc resolved in step 1, static-only, into toolchain/mpp-arm64.
#
#        Ref: pinned by exact commit, NOT a release tag. Every semver tag
#        1.0.0..1.0.11 on this repo was probed and all of them fail to even
#        *configure*: mpp/codec/dec/h265/CMakeLists.txt references
#        h265d_parser.c/h265d_ps.c/h265d_codec.h that do not exist in that
#        tag's tree (an upstream tagging defect — verified independent of
#        musl/cross-compilation; a plain `cmake -S .` on any host hits the
#        same "Cannot find source file" error at every tag). `develop` HEAD
#        has a complete, consistent tree (full h265 decoder rewrite), so
#        that's what's pinned here — commit df4864bd1e907cbfd427c397348976c5b2b05ee9
#        ("fix[mpi_enc_utils]: Restore ref_cfg setup", 2026-07-28). Bump by
#        changing MPP_REF below; re-verify h265/CMakeLists.txt's file list
#        still matches the tree before trusting a new ref.
MPP_REF=df4864bd1e907cbfd427c397348976c5b2b05ee9

# Desired-state guard, not directory-existence: a bare `[ ! -d ... ]` check
# has two failure modes — (a) an interrupted fetch/checkout can leave
# toolchain/mpp-src existing-but-empty/partial, and every later run would
# then skip the clone and die confusingly at the cmake step below with no
# way to self-heal short of manually rm -rf'ing it; (b) bumping MPP_REF (the
# documented re-pin procedure above) would silently keep building the OLD
# checkout forever, since a directory-existence check can't tell the repo is
# now pinned to a different commit. Comparing HEAD to MPP_REF fixes both.
MPP_HEAD=$(git -C toolchain/mpp-src rev-parse HEAD 2>/dev/null || echo none)
if [ "$MPP_HEAD" != "$MPP_REF" ]; then
  rm -rf toolchain/mpp-src
  # Clone into a temp dir and mv into place only after checkout succeeds, so
  # an interrupted fetch/checkout can never leave a half-cloned directory
  # that a future run's HEAD check would trust as complete.
  mpp_tmp="$(mktemp -d toolchain/mpp-src.XXXXXX)"
  git -C "$mpp_tmp" init -q
  git -C "$mpp_tmp" remote add origin https://github.com/rockchip-linux/mpp
  git -C "$mpp_tmp" fetch --depth 1 origin "$MPP_REF"
  git -C "$mpp_tmp" checkout -q FETCH_HEAD
  mv "$mpp_tmp" toolchain/mpp-src
fi

# Ref-aware artifact cache: toolchain/mpp-arm64/.ref records which MPP_REF
# the staged librockchip_mpp.a was built from, so a re-pin (MPP_REF bumped
# above) correctly triggers a rebuild instead of silently reusing a stale
# archive built from the old commit. .ref is written only AFTER a successful
# build (see below), so an interrupted build also self-heals: no matching
# .ref means the next run rebuilds rather than trusting a partial artifact.
if [ ! -e toolchain/mpp-arm64/lib/librockchip_mpp.a ] \
   || [ "$(cat toolchain/mpp-arm64/.ref 2>/dev/null)" != "$MPP_REF" ]; then
  # -DBUILD_TEST=OFF: test/demo binaries have glibc-isms and aren't needed.
  # -DBUILD_SHARED_LIBS=OFF: mpp's own CMakeLists always builds BOTH
  #   rockchip_mpp (SHARED) and rockchip_mpp_static regardless of this
  #   flag, and the shared link fails on this musl-static toolchain
  #   ("cannot find -lmvec", a glibc math-vector lib with no musl
  #   equivalent) — so below we build ONLY the `rockchip_mpp_static`
  #   target explicitly rather than the default `all`, sidestepping the
  #   shared lib entirely instead of patching mpp's CMakeLists.
  # -DCMAKE_POSITION_INDEPENDENT_CODE=ON: matches the brief's guidance for
  #   musl-static cross builds; no -Werror was hit so no warnings-as-errors
  #   flag was needed here (contra the brief's anticipated risk).
  rm -rf toolchain/mpp-build toolchain/mpp-arm64
  cmake -S toolchain/mpp-src -B toolchain/mpp-build \
    -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
    -DCMAKE_C_COMPILER="$MABUR_CROSS_CC" -DCMAKE_CXX_COMPILER="$MABUR_CROSS_CXX" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DBUILD_TEST=OFF \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
  cmake --build toolchain/mpp-build -j"$(nproc)" --target rockchip_mpp_static

  # No `make install` (its `install(TARGETS rockchip_mpp ...)` line would
  # pull the broken shared-lib target back in as a dependency); copy the
  # static archive + public headers by hand instead. Upstream's own
  # OUTPUT_NAME for the static target is "rockchip_mpp" (not
  # "rockchip_mpp_static"), so the artifact really is librockchip_mpp.a as
  # planned — a single merged relocatable object (mpp's own post-build `ar`
  # step folds every internal component lib into it), so linking it needs
  # no --whole-archive dance.
  mkdir -p toolchain/mpp-arm64/include/rockchip toolchain/mpp-arm64/lib
  cp toolchain/mpp-src/inc/*.h toolchain/mpp-arm64/include/rockchip/
  cp toolchain/mpp-build/mpp/librockchip_mpp.a toolchain/mpp-arm64/lib/
  # Written last, only on success: marks the artifact cache as valid for
  # this exact MPP_REF (see the skip condition above).
  echo "$MPP_REF" > toolchain/mpp-arm64/.ref
fi
export MABUR_MPP_ROOT="$PWD/toolchain/mpp-arm64"

# --- 5. Configure + build. maburgs + maburplay (MABUR_PLAYER_HW=ON, mpp +
#        drm backends wired via step 3/4's roots): devourer's example
#        binaries (rxdemo, txdemo, ...) are not needed on the ground station
#        and some pull in extra libusb example plumbing not worth static-linking
#        here. MABUR_BUILD_TESTS=OFF: the host test suite needs a host-runnable
#        libusb + GoogleTest et al; irrelevant for a cross artifact.
cmake -S . -B build-arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-musl.cmake \
  -DCMAKE_BUILD_TYPE=Release -DMABUR_BUILD_TESTS=OFF -DMABUR_BUILD_DRONE=OFF \
  -DMABUR_BUILD_GS=ON -DDEVOURER_JAGUAR1=OFF -DDEVOURER_8814=OFF \
  -DDEVOURER_JAGUAR2_8822B=OFF -DDEVOURER_JAGUAR2_8821C=OFF \
  -DDEVOURER_JAGUAR3_8822C=OFF -DDEVOURER_JAGUAR3_8822E=ON \
  -DDEVOURER_KESTREL_8852B=OFF -DDEVOURER_KESTREL_8852C=OFF \
  -DDEVOURER_LOG_MAX_LEVEL=WARN \
  -DMABUR_PLAYER_HW=ON -DMABUR_MPP_ROOT="$MABUR_MPP_ROOT" -DMABUR_DRM_ROOT="$MABUR_DRM_ROOT"

cmake --build build-arm64 -j"$(nproc)" --target maburgs linkbench-rx txagcbench-rx maburplay encosd

"${TARGET_TRIPLE}-strip" build-arm64/gs/maburgs -o out/arm64/maburgs
"${TARGET_TRIPLE}-strip" build-arm64/bench/linkbench/linkbench-rx -o out/arm64/linkbench-rx
"${TARGET_TRIPLE}-strip" build-arm64/bench/txagcbench/txagcbench-rx -o out/arm64/txagcbench-rx
"${TARGET_TRIPLE}-strip" build-arm64/gs/player/maburplay -o out/arm64/maburplay
"${TARGET_TRIPLE}-strip" build-arm64/bench/encosd/encosd -o out/arm64/encosd

# maburplay's OSD glyph atlas is a runtime asset, not a linked-in blob (that
# was the point of retiring the generated msp_font_btfl.cpp): stage the
# committed bundle copy next to the binaries so the deploy step has one
# directory holding everything it must push. Installs on the GS as
# /usr/local/share/mabur/font_btfl.mfont -- the path maburplay.default.json's
# osd.font points at. Regenerate with tools/msp/gen_font.py (see its header).
cp gs/player/bundle/font_btfl.mfont out/arm64/font_btfl.mfont

# The GS link-status overlay's own atlas, staged the same way. Installs as
# /usr/local/share/mabur/gs_osd.gfont -- the path maburplay.default.json's
# osd.gs.font points at. Unlike the MSP .mfont (pre-coloured ARGB glyphs at
# one size), this is a two-channel coverage+shadow MASK baked at all 30
# sizes the responsive layout can ask for across 720p..2160p, which is why
# it is the larger of the two files. Regenerate with tools/msp/gen_gsfont.py
# (see its header for the exact command and the expected byte count).
cp gs/player/bundle/gs_osd.gfont out/arm64/gs_osd.gfont

# maburplay's startup splash, staged the same way. Installs as
# /usr/local/share/mabur/splash.bin -- the path splash_image.h hardcodes
# (there is deliberately no config key for it). Raw XRGB8888 with a 16-byte
# header, so the player needs no image decoder; regenerate with
# tools/gen_splash.py (see its header for the exact command).
cp gs/player/bundle/splash.bin out/arm64/splash.bin

# `file` itself isn't on a bare NixOS PATH either; stage it like pkg-config.
if [ ! -e toolchain/file ]; then
  nix-build -E \
    'with import <nixpkgs> {}; file' \
    -o toolchain/file
fi
"$(readlink -f toolchain/file)/bin/file" out/arm64/maburgs out/arm64/linkbench-rx out/arm64/txagcbench-rx out/arm64/maburplay out/arm64/encosd
