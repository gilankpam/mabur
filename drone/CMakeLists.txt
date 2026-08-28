add_library(mabur_drone_core STATIC
  src/config.cpp
  src/waybeam_client.cpp
  src/frame_source.cpp
  src/frame_pipeline.cpp
  src/radio_tx.cpp
  src/rc_agent.cpp
  src/msp_serial.cpp
  src/telemetry.cpp
  vendor/venc_frame_ring.c)
target_include_directories(mabur_drone_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/vendor
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
target_link_libraries(mabur_drone_core PUBLIC mabur_common rt devourer)
# Unconditional (not just ARM cross-builds): a later task's config header
# includes a drone/venc header on host builds too. drone/vendor is already
# on mabur_drone_core above.
target_include_directories(mabur_drone_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/venc)

add_executable(maburd src/main.cpp)
target_include_directories(maburd PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(maburd PRIVATE mabur_drone_core mabur_common devourer pthread)

# Venc core: waybeam_venc star6e encoder folded in (spec
# docs/superpowers/specs/2026-08-28-venc-foldin-design.md). ARM-only: it
# drives SigmaStar MI via dlopen and has no host-runnable pipeline.
if(CMAKE_CROSSCOMPILING)
  file(GLOB VENC_CORE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/venc/*.c)
  add_library(venc_core STATIC ${VENC_CORE_SRC})
  target_include_directories(venc_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/venc ${CMAKE_CURRENT_SOURCE_DIR}/vendor)
  target_compile_definitions(venc_core PRIVATE
    PLATFORM_STAR6E HAVE_BACKEND_STAR6E=1 _GNU_SOURCE)
  target_link_libraries(venc_core PUBLIC dl pthread rt m)
endif()
