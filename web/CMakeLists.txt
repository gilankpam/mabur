# Web GS (spec 2026-09-27-web-gs). Two entry points:
#  - included from the root build (host/native): mabur_web_core + webgs CLI,
#    so the core's tests run in host ctest;
#  - standalone under emcmake (web/ as the source dir): the WASM page build,
#    see the EMSCRIPTEN branch below.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  # Standalone: the WASM page build under emcmake. Needs the WebUSB libusb in
  # the repo-local EM_CACHE sysroot first (web/tools/build-libusb-wasm.sh):
  #   export EM_CACHE=$PWD/web/.deps/emcache
  #   emcmake cmake -S web -B web/build-wasm -DCMAKE_BUILD_TYPE=Release
  #   cmake --build web/build-wasm -j
  cmake_minimum_required(VERSION 3.16)
  project(webgs CXX C)
  if(NOT EMSCRIPTEN)
    message(FATAL_ERROR "standalone web/ build is WASM-only: use emcmake")
  endif()
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(MABUR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH "mabur checkout")
  set(DEVOURER_DIR "${MABUR_DIR}/../devourer" CACHE PATH "devourer checkout")
  # RTL8812EU (Jaguar3 8822E) + RTL8812AU/8811AU/8821AU (Jaguar1).
  foreach(opt 8814 JAGUAR2_8822B JAGUAR2_8821C JAGUAR3_8822C 8733B KESTREL_8852B KESTREL_8852C)
    set(DEVOURER_${opt} OFF CACHE BOOL "" FORCE)
  endforeach()
  set(DEVOURER_JAGUAR3_8822E ON CACHE BOOL "" FORCE)
  set(DEVOURER_JAGUAR1 ON CACHE BOOL "" FORCE)
  set(DEVOURER_LOG_MAX_LEVEL WARN CACHE STRING "" FORCE)
  # Native WASM exceptions: devourer's teardown/de-init paths catch failed
  # USB writes (a lost card); without this a throw aborts the module.
  add_compile_options(-pthread -fwasm-exceptions)
  add_link_options(-fwasm-exceptions)
  add_subdirectory(${DEVOURER_DIR} ${CMAKE_BINARY_DIR}/devourer EXCLUDE_FROM_ALL)
  # Receive + control subset. NEVER fec_worker / sw_encoder / uep_encoder
  # (pthread_setaffinity_np). Undefined mabur symbol at link -> add the
  # common/src or gs/src file that defines it here.
  add_library(webgs_core STATIC
    src/web_gs.cpp src/osd_screen.cpp
    ${MABUR_DIR}/common/src/crc16.cpp ${MABUR_DIR}/common/src/gf256.cpp
    ${MABUR_DIR}/common/src/sbi.cpp ${MABUR_DIR}/common/src/frag.cpp
    ${MABUR_DIR}/common/src/nal.cpp ${MABUR_DIR}/common/src/uep_decoder.cpp
    ${MABUR_DIR}/common/src/sw_wire.cpp ${MABUR_DIR}/common/src/sw_decoder.cpp
    ${MABUR_DIR}/common/src/frame_wire.cpp ${MABUR_DIR}/common/src/rc_proto.cpp
    ${MABUR_DIR}/common/src/profile.cpp ${MABUR_DIR}/common/src/node.cpp
    ${MABUR_DIR}/common/src/hevc_params.cpp ${MABUR_DIR}/common/src/toml.cpp
    ${MABUR_DIR}/common/src/dvr_mux.cpp ${MABUR_DIR}/common/src/raw_dvr.cpp
    ${MABUR_DIR}/common/src/probe_wire.cpp ${MABUR_DIR}/common/src/msp_dp.cpp
    ${MABUR_DIR}/gs/src/aggregator.cpp ${MABUR_DIR}/gs/src/frame_stream.cpp
    ${MABUR_DIR}/gs/src/frame_file_source.cpp ${MABUR_DIR}/gs/src/config.cpp
    ${MABUR_DIR}/gs/src/link_health.cpp ${MABUR_DIR}/gs/src/probe_track.cpp
    ${MABUR_DIR}/gs/src/ladder_residual.cpp ${MABUR_DIR}/gs/src/vrx_controller.cpp
    ${MABUR_DIR}/gs/src/rendezvous.cpp ${MABUR_DIR}/gs/src/ladder_controller.cpp
    ${MABUR_DIR}/gs/src/rung_store.cpp ${MABUR_DIR}/gs/src/rcf_slot.cpp
    ${MABUR_DIR}/gs/src/rtt_estimator.cpp ${MABUR_DIR}/gs/src/dot11.cpp
    ${MABUR_DIR}/gs/src/msp_sink.cpp
    ${MABUR_DIR}/gs/src/relay_wire.cpp ${MABUR_DIR}/gs/src/relay_client.cpp)
  target_include_directories(webgs_core PUBLIC src ${MABUR_DIR}/common/include
    ${MABUR_DIR}/gs/src ${MABUR_DIR}/third_party ${DEVOURER_DIR}/src)
  target_compile_options(webgs_core PRIVATE -O3)
  target_link_libraries(webgs_core PUBLIC devourer)

  # The page build: live only (WebUSB), default config embedded.
  add_executable(webgs src/web_main.cpp src/opfs_file.cpp src/relay_link.cpp src/relay_transport.cpp)
  target_compile_definitions(webgs PRIVATE WEBGS_LIVE=1 WEBGS_CONFIG_PATH="/maburgs.toml")
  target_link_libraries(webgs PRIVATE webgs_core)
  target_link_options(webgs PRIVATE
    -pthread -sPROXY_TO_PTHREAD -sPTHREAD_POOL_SIZE=8 -sINITIAL_MEMORY=268435456
    -sASYNCIFY -sMODULARIZE -sEXPORT_NAME=createWebGs -sEXIT_RUNTIME
    -sENVIRONMENT=web,worker --bind
    -sEXPORTED_FUNCTIONS=_main,_free,_malloc,_webgs_stop,_webgs_set_rec,_webgs_set_local_rec,_webgs_request_idr
    -sEXPORTED_RUNTIME_METHODS=FS,stringToNewUTF8
    "--embed-file=${MABUR_DIR}/gs/bundle/maburgs.default.toml@/maburgs.toml")
  set_target_properties(webgs PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dist)

  # Replay CLI only, for Node (host FS via NODERAWFS): the native/WASM parity
  # gate (web/tests/test_wasm_parity.sh).
  add_executable(webgs_node src/web_main.cpp)
  target_compile_definitions(webgs_node PRIVATE WEBGS_CONFIG_PATH="${MABUR_DIR}/gs/bundle/maburgs.default.toml")
  target_link_libraries(webgs_node PRIVATE webgs_core)
  target_link_options(webgs_node PRIVATE -pthread -sENVIRONMENT=node -sNODERAWFS=1
    -sALLOW_MEMORY_GROWTH -sEXIT_RUNTIME --bind -sASYNCIFY)

  # Page UI (web/ui, Vite + Svelte) built into web/dist next to webgs.{js,wasm}.
  # npm ci re-runs when the lockfile changes or node_modules is gone (its
  # OUTPUT is node_modules/.package-lock.json, which npm itself writes --
  # a build-dir stamp would survive rm -rf node_modules); vite build re-runs when any UI
  # source changes OR dist/index.html is missing (deleting dist rebuilds it).
  # dist/assets is cleared first so stale hashed bundles don't pile up; Vite
  # itself runs with emptyOutDir=false so the Emscripten outputs survive.
  set(WEB_UI ${CMAKE_CURRENT_SOURCE_DIR}/ui)
  set(WEB_DIST ${CMAKE_CURRENT_SOURCE_DIR}/dist)
  find_program(NPM npm REQUIRED)
  set(NPM_STAMP ${WEB_UI}/node_modules/.package-lock.json)
  add_custom_command(OUTPUT ${NPM_STAMP}
    COMMAND ${NPM} ci --no-audit --no-fund
    # Touch: the output must be newer than package-lock.json even if npm
    # leaves the hidden lockfile's mtime alone.
    COMMAND ${CMAKE_COMMAND} -E touch ${NPM_STAMP}
    WORKING_DIRECTORY ${WEB_UI}
    DEPENDS ${WEB_UI}/package.json ${WEB_UI}/package-lock.json
    COMMENT "npm ci (web/ui)")
  file(GLOB_RECURSE WEB_UI_SRC CONFIGURE_DEPENDS ${WEB_UI}/src/* ${WEB_UI}/public/*)
  add_custom_command(OUTPUT ${WEB_DIST}/index.html
    COMMAND ${CMAKE_COMMAND} -E rm -rf ${WEB_DIST}/assets
    COMMAND ${NPM} run build
    WORKING_DIRECTORY ${WEB_UI}
    DEPENDS ${NPM_STAMP} ${WEB_UI_SRC} ${WEB_UI}/index.html ${WEB_UI}/vite.config.js
            ${WEB_UI}/sw/sw.js ${WEB_UI}/sw/plugin.mjs webgs
    COMMENT "vite build -> web/dist")
  add_custom_target(webgs_ui ALL DEPENDS ${WEB_DIST}/index.html)
else()
  # Included from the root build: native core + tests.
  add_library(mabur_web_core STATIC src/web_gs.cpp src/osd_screen.cpp)
  target_include_directories(mabur_web_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
    PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
  target_link_libraries(mabur_web_core PUBLIC mabur_gs_core mabur_common)

  # webgs CLI: replay (the web_au_parity gate) + live (libusb + devourer,
  # the same loop the WASM build runs, or --relay over UDP to a CPE510
  # mabur-relay instead of the USB card).
  add_executable(webgs src/web_main.cpp src/relay_link.cpp src/relay_transport.cpp)
  target_link_libraries(webgs PRIVATE mabur_web_core mabur_gs_dot11 mabur_gs_relay devourer pthread)
  target_include_directories(webgs PRIVATE ${DEVOURER_DIR}/src ${CMAKE_SOURCE_DIR}/gs/src)
  target_compile_definitions(webgs PRIVATE WEBGS_LIVE=1
    MABUR_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
endif()
