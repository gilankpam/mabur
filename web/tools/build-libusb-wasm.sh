#!/usr/bin/env bash
# Upstream libusb with the Emscripten WebUSB backend, into a repo-local
# EM_CACHE sysroot (emcmake overrides PKG_CONFIG_LIBDIR and -L to the
# sysroot, so it has to live there). Pinned to the commit the 2026-09-27
# spike validated. Run inside: nix-shell -p emscripten autoconf automake libtool pkg-config
set -euo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd)
DEPS="$HERE/.deps"; export EM_CACHE="$DEPS/emcache"
REV=a45bb16
mkdir -p "$DEPS"
[ -d "$DEPS/libusb" ] || git clone https://github.com/libusb/libusb "$DEPS/libusb"
cd "$DEPS/libusb" && git checkout -q "$REV"
[ -x configure ] || ./autogen.sh --help >/dev/null
emconfigure ./configure --host=wasm32-unknown-emscripten --prefix="$DEPS/wasm-prefix" \
  --disable-shared --enable-static "CFLAGS=-O2 -pthread" "CXXFLAGS=-O2 -pthread"
emmake make -j"$(nproc)" install
SYS="$EM_CACHE/sysroot"
mkdir -p "$SYS/lib/wasm32-emscripten" "$SYS/local/lib/pkgconfig"
cp "$DEPS/wasm-prefix/lib/libusb-1.0.a" "$SYS/lib/wasm32-emscripten/"
# Hand-written .pc (the spike's): the installed one only exports
# include/libusb-1.0, but devourer includes <libusb-1.0/libusb.h>. Link flags
# (--bind, ASYNCIFY) are set on the targets in web/CMakeLists.txt.
cat > "$SYS/local/lib/pkgconfig/libusb-1.0.pc" <<PC
prefix=$DEPS/wasm-prefix
Name: libusb-1.0
Description: libusb wasm (WebUSB backend)
Version: 1.0.30
Libs: -L\${prefix}/lib -lusb-1.0
Cflags: -I\${prefix}/include/libusb-1.0 -I\${prefix}/include
PC
echo "libusb (wasm) ready; export EM_CACHE=$EM_CACHE before emcmake"
