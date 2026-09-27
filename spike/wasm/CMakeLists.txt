# THROWAWAY SPIKE (branch wasm-spike). Standalone: builds rxprobe natively or,
# under emcmake, as rxprobe.js/.wasm for the browser. See spike/wasm/README.md.
cmake_minimum_required(VERSION 3.16)
project(rxprobe CXX C)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(DEVOURER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../devourer" CACHE PATH "devourer checkout")
foreach(opt JAGUAR1 8814 JAGUAR2_8822B JAGUAR2_8821C JAGUAR3_8822C 8733B KESTREL_8852B KESTREL_8852C)
  set(DEVOURER_${opt} OFF CACHE BOOL "" FORCE)
endforeach()
set(DEVOURER_JAGUAR3_8822E ON CACHE BOOL "" FORCE)
set(DEVOURER_LOG_MAX_LEVEL WARN CACHE STRING "" FORCE)
if(EMSCRIPTEN)
  add_compile_options(-pthread)
endif()
add_subdirectory(${DEVOURER_DIR} ${CMAKE_BINARY_DIR}/devourer EXCLUDE_FROM_ALL)
add_executable(rxprobe rxprobe.cpp)
target_include_directories(rxprobe PRIVATE ${DEVOURER_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/../../common/include)
target_link_libraries(rxprobe PRIVATE devourer)
if(EMSCRIPTEN)
  target_link_options(rxprobe PRIVATE
    -pthread -sPROXY_TO_PTHREAD -sPTHREAD_POOL_SIZE=8 -sALLOW_MEMORY_GROWTH
    -sASYNCIFY -sMODULARIZE -sEXPORT_NAME=createRxProbe -sEXIT_RUNTIME
    -sENVIRONMENT=web,worker --bind)
endif()

# --- gsweb: mabur receive core (replay + live) ------------------------------
set(MABUR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." CACHE PATH "mabur checkout")
# Receive subset of mabur_common + FrameStream. Never sw_encoder / uep_encoder
# / fec_worker (pthread_setaffinity_np breaks the WASM build). If the link
# reports an undefined mabur symbol, add the common/src file that defines it.
add_library(gsweb_core STATIC
  gsweb_core.cpp
  ${MABUR_DIR}/common/src/crc16.cpp
  ${MABUR_DIR}/common/src/gf256.cpp
  ${MABUR_DIR}/common/src/sbi.cpp
  ${MABUR_DIR}/common/src/frag.cpp
  ${MABUR_DIR}/common/src/nal.cpp
  ${MABUR_DIR}/common/src/uep_decoder.cpp
  ${MABUR_DIR}/common/src/sw_wire.cpp
  ${MABUR_DIR}/common/src/sw_decoder.cpp
  ${MABUR_DIR}/common/src/frame_wire.cpp
  ${MABUR_DIR}/common/src/rc_proto.cpp
  ${MABUR_DIR}/common/src/profile.cpp
  ${MABUR_DIR}/common/src/node.cpp
  ${MABUR_DIR}/common/src/hevc_params.cpp
  ${MABUR_DIR}/gs/src/frame_stream.cpp
  # control uplink (pinned rung): VrxController + rendezvous; the ladder is
  # constructed but never ticked in pin mode.
  ${MABUR_DIR}/gs/src/vrx_controller.cpp
  ${MABUR_DIR}/gs/src/rendezvous.cpp
  ${MABUR_DIR}/gs/src/ladder_controller.cpp
  ${MABUR_DIR}/gs/src/rung_store.cpp
  ${MABUR_DIR}/gs/src/rcf_slot.cpp)
target_include_directories(gsweb_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR} ${MABUR_DIR}/common/include ${MABUR_DIR}/gs/src)
target_compile_options(gsweb_core PRIVATE -O3)

add_executable(gsweb gsweb.cpp)
target_include_directories(gsweb PRIVATE ${DEVOURER_DIR}/src)
target_link_libraries(gsweb PRIVATE gsweb_core devourer)

if(NOT EMSCRIPTEN)
  add_subdirectory(${MABUR_DIR}/common ${CMAKE_BINARY_DIR}/mabur_common EXCLUDE_FROM_ALL)
  add_executable(mkbodies mkbodies.cpp)
  target_link_libraries(mkbodies PRIVATE mabur_common)
endif()

if(EMSCRIPTEN)
  # Replay-only build for Node: direct host FS, no WebUSB, no pthreads needed
  # at runtime (objects are still -pthread compiled; Node supports that).
  add_executable(gsweb_node gsweb.cpp)
  target_link_libraries(gsweb_node PRIVATE gsweb_core)
  target_link_options(gsweb_node PRIVATE
    -pthread -sENVIRONMENT=node -sNODERAWFS=1 -sALLOW_MEMORY_GROWTH -sEXIT_RUNTIME
    --bind -sASYNCIFY)  # libusb WebUSB backend (linked via devourer, never called in replay)
  target_link_options(gsweb PRIVATE
    -pthread -sPROXY_TO_PTHREAD -sPTHREAD_POOL_SIZE=8 -sINITIAL_MEMORY=268435456
    -sASYNCIFY -sMODULARIZE -sEXPORT_NAME=createGsweb -sEXIT_RUNTIME
    -sENVIRONMENT=web,worker --bind -sEXPORTED_FUNCTIONS=_main,_free,_malloc)
  target_link_libraries(gsweb_node PRIVATE devourer)
  target_include_directories(gsweb_node PRIVATE ${DEVOURER_DIR}/src)
endif()
