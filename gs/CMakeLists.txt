add_library(mabur_gs_core STATIC
  src/config.cpp
  src/aggregator.cpp
  src/udp_sink.cpp
  src/frame_file_source.cpp
  src/energy.cpp
  src/op_table.cpp
  src/score.cpp
  src/controller.cpp
  src/rendezvous.cpp
  src/vrx_controller.cpp
  src/tx_selector.cpp
  src/msp_sink.cpp
  src/msp_font_btfl.cpp
  src/shm_surface.cpp
  src/msp_renderer.cpp
  src/frame_stream.cpp)
target_include_directories(mabur_gs_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
target_link_libraries(mabur_gs_core PUBLIC mabur_common)

# shm_open/mmap need librt on glibc hosts; the musl aarch64 cross-build folds
# it into libc and has no separate librt. NOT CMAKE_CROSSCOMPILING is required:
# find_library otherwise resolves the HOST glibc librt.so and tries to
# static-link that (wrong-arch, dynamic) into the musl binary, which fails
# with "attempted static link of dynamic object".
find_library(RT_LIB rt)
if(RT_LIB AND NOT CMAKE_CROSSCOMPILING)
  target_link_libraries(mabur_gs_core PUBLIC ${RT_LIB})
endif()

add_library(mabur_gs_radio STATIC src/radio_frontend.cpp)
target_include_directories(mabur_gs_radio PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mabur_gs_radio PUBLIC mabur_gs_core mabur_common devourer)

add_executable(maburgs src/main.cpp)
target_include_directories(maburgs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(maburgs PRIVATE mabur_gs_core mabur_gs_radio mabur_common pthread)
