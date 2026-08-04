add_library(mabur_player_core STATIC
  src/player_config.cpp
  src/hevc_params.cpp
  src/dvr_mux.cpp
  src/ring_client.cpp
  src/osd_font.cpp
  src/gs_font.cpp
  src/gs_draw.cpp
  src/gs_snapshot.cpp
  src/gs_source.cpp
  src/gs_overlay.cpp
  src/gs_metrics.cpp
  src/osd_layout.cpp
  src/osd_raster.cpp
  src/osd_source.cpp
  src/osd_palette.cpp
  src/in_formats.cpp)
target_include_directories(mabur_player_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
target_link_libraries(mabur_player_core PUBLIC mabur_gs_core)

# Hardware backends (rockchip mpp decode + libdrm KMS presenter): aarch64
# cross build only. OFF by default -- the host has neither SDK, and the
# host e2e only ever exercises the null backend. When ON, two cache vars
# must point at the toolchain/ artifacts tools/build-arm64.sh stages:
#   MABUR_MPP_ROOT - install prefix with lib/librockchip_mpp.a (static;
#     built from source since nixpkgs has no rockchip-mpp package) and
#     include/rockchip/*.h.
#   MABUR_DRM_ROOT - prefix with lib/libdrm.a (static, prebuilt by Nix via
#     pkgsCross.aarch64-multiplatform.pkgsStatic.libdrm) and include/
#     (xf86drm.h, xf86drmMode.h, plus the libdrm/ subdir for driver headers).
option(MABUR_PLAYER_HW "rockchip mpp + drm hardware backends (aarch64 cross build only); needs -DMABUR_MPP_ROOT= and -DMABUR_DRM_ROOT=, see tools/build-arm64.sh" OFF)

# maburplay: main.cpp + the null backend are always built (host and cross).
# Hardware backends (mpp_backend.cpp, drm_presenter.cpp) are appended here
# behind MABUR_PLAYER_HW starting Task 7.
add_executable(maburplay
  src/main.cpp
  src/null_backend.cpp)
target_link_libraries(maburplay PRIVATE mabur_player_core)

if(MABUR_PLAYER_HW)
  if(NOT MABUR_MPP_ROOT OR NOT MABUR_DRM_ROOT)
    message(FATAL_ERROR
      "MABUR_PLAYER_HW=ON requires -DMABUR_MPP_ROOT=<mpp install prefix> "
      "and -DMABUR_DRM_ROOT=<libdrm install prefix>; see tools/build-arm64.sh")
  endif()
  target_sources(maburplay PRIVATE
    src/mpp_backend.cpp
    src/mpp_encoder.cpp
    src/burn_recorder.cpp
    src/drm_presenter.cpp
    src/osd_surface.cpp)
  target_compile_definitions(maburplay PRIVATE MABUR_PLAYER_HW=1)
  target_include_directories(maburplay PRIVATE
    "${MABUR_MPP_ROOT}/include"
    "${MABUR_DRM_ROOT}/include"
    "${MABUR_DRM_ROOT}/include/libdrm")
  target_link_libraries(maburplay PRIVATE
    "${MABUR_MPP_ROOT}/lib/librockchip_mpp.a"
    "${MABUR_DRM_ROOT}/lib/libdrm.a"
    pthread)
endif()
