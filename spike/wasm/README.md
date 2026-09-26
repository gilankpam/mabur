# THROWAWAY SPIKE — devourer RX in the browser (WebUSB + WASM)

Question: can devourer, compiled to WASM and run in Chrome over WebUSB, bring
up the RTL8812EU in usable time and keep up with live mabur video?
Answer (2026-09-27, bench drone ch136 HT40): yes. Parity with native on init,
throughput and loss at 16, 38 and 54 Mb/s on air; the jitter tail is worse:
periodic ~9 s spikes of 8-14 ms under load (native max 1.7 ms). `rxprobe ... bench`
walks linkbench-tx frames instead of maburd video. Numbers in the commit messages.

Spike cause (Chrome trace, `trace.mjs`): Blink's Oilpan incremental sweep
(`cppgc::Sweeper::IncrementalSweepTask`, 4-8 ms slices) of the per-transfer
WebUSB objects, after a major GC every ~8-13 s at 38 Mb/s. It runs on the
thread that owns the WebUSB objects — moving the runtime into a dedicated
worker (`index.html?worker=1`, `worker.js`) moves the sweep with it, spikes
unchanged. Lever left: fewer transfers (chip RX aggregation), a latency trade.

`rxprobe.cpp` builds unchanged both ways:

    # native (host, NixOS)
    nix-shell -p pkg-config libusb1 cmake --run "cmake -S . -B build-native -DCMAKE_BUILD_TYPE=Release && cmake --build build-native -j"
    ./build-native/rxprobe 136 40 60

    # wasm: upstream libusb (has os/emscripten_webusb.cpp) built with
    #   emconfigure ./configure --host=wasm32-unknown-emscripten CFLAGS=-pthread CXXFLAGS=-pthread
    # then its libusb-1.0.a into $EM_CACHE/sysroot/lib/wasm32-emscripten and a
    # libusb-1.0.pc into $EM_CACHE/sysroot/local/lib/pkgconfig (emcmake
    # overrides PKG_CONFIG_LIBDIR and -L to the sysroot), then:
    nix-shell -p emscripten cmake pkg-config --run "emcmake cmake -S . -B build-wasm -DCMAKE_BUILD_TYPE=Release && cmake --build build-wasm -j"
    python3 serve.py 8808   # COOP/COEP headers, required for pthreads
    # open http://127.0.0.1:8808/index.html?s=60 in Chrome, click, pick the card

The WebUSB chooser can't be answered over CDP (DeviceAccess is Bluetooth-only):
pick the card once by hand in a persistent Chrome profile; after that
`getDevices()` returns it and `node drive.mjs <page-ws-url> <secs>` automates
runs (`peek.mjs` dumps the page log). Chrome keeps the interface claimed until
the tab closes — native runs fail BUSY meanwhile.

## gsweb live check (native)

    ./build-native/gsweb live 136 40 20

Pass: after init, every STAT line shows aus ≈ the drone's fps (60 ± 5),
trunc ≤ 1 % of aus, bad_cfg=0, qdrop=0.
