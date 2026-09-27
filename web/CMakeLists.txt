# Web GS (spec 2026-09-27-web-gs). Two entry points:
#  - included from the root build (host/native): mabur_web_core + webgs CLI,
#    so the core's tests run in host ctest;
#  - standalone under emcmake (web/ as the source dir): the WASM page build,
#    see the EMSCRIPTEN branch below.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  # Standalone: the WASM page build under emcmake (filled in by a later task).
  cmake_minimum_required(VERSION 3.16)
  project(webgs CXX C)
  message(FATAL_ERROR "web/: standalone WASM build not implemented yet")
else()
  # Included from the root build: native core + tests.
  add_library(mabur_web_core STATIC src/web_gs.cpp)
  target_include_directories(mabur_web_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
    PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
  target_link_libraries(mabur_web_core PUBLIC mabur_gs_core mabur_common)

  # webgs CLI: replay (the web_au_parity gate) + live (libusb + devourer,
  # the same loop the WASM build runs).
  add_executable(webgs src/web_main.cpp)
  target_link_libraries(webgs PRIVATE mabur_web_core mabur_gs_dot11 devourer pthread)
  target_include_directories(webgs PRIVATE ${DEVOURER_DIR}/src ${CMAKE_SOURCE_DIR}/gs/src)
  target_compile_definitions(webgs PRIVATE WEBGS_LIVE=1
    MABUR_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
endif()
