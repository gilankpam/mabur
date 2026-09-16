add_library(mabur_player_core STATIC
  src/player_config.cpp
  src/hevc_params.cpp
  src/dvr_mux.cpp
  src/dvr_name.cpp
  src/rec_button.cpp
  src/ring_client.cpp
  src/osd_font.cpp
  src/gs_font.cpp
  src/gs_draw.cpp
  src/gs_snapshot.cpp
  src/gs_source.cpp
  src/gs_layer.cpp
  src/gs_overlay.cpp
  src/gs_compact.cpp
  src/gs_metrics.cpp
  src/osd_layout.cpp
  src/osd_raster.cpp
  src/osd_compose.cpp
  src/osd_source.cpp
  src/osd_palette.cpp
  src/in_formats.cpp
  src/frame_regulator.cpp
  src/lat_tracker.cpp
  src/lat_log.cpp
  src/splash_image.cpp
  src/colortrans.cpp)
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

# Burned-DVR colortrans on the GPU (docs/colortrans.md): Mesa EGL/GLESv2/GBM
# (panfrost) + librga from the Buildroot sysroot. Only ever ON in the
# Buildroot glibc build (cmake/aarch64-buildroot.cmake); the musl-static
# build cannot link Mesa and stays the rollback build without this stage.
option(MABUR_PLAYER_GPU "GPU colortrans stage for the burned DVR (needs MABUR_PLAYER_HW and a sysroot with EGL/GLESv2/gbm/rga)" OFF)
if(MABUR_PLAYER_GPU)
  if(NOT MABUR_PLAYER_HW)
    message(FATAL_ERROR "MABUR_PLAYER_GPU=ON requires MABUR_PLAYER_HW=ON")
  endif()
  if(NOT CMAKE_SYSROOT)
    message(FATAL_ERROR
      "MABUR_PLAYER_GPU=ON requires a toolchain that sets CMAKE_SYSROOT "
      "(cmake/aarch64-buildroot.cmake); the EGL/GLES/gbm/rga search below "
      "disables CMake's find-root-path protections and would otherwise "
      "search raw host paths (e.g. under cmake/aarch64-musl.cmake, which "
      "never sets CMAKE_SYSROOT).")
  endif()
  # HINTS + NO_DEFAULT_PATH + NO_CMAKE_FIND_ROOT_PATH: cmake/aarch64-buildroot.cmake's
  # CMAKE_SYSROOT already lands "<sysroot>/usr" in CMAKE_SYSTEM_PREFIX_PATH,
  # and CMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY then prefixes CMAKE_SYSROOT
  # onto that AGAIN -- doubling it to "<sysroot>/<sysroot>/usr/include", which
  # never exists, so a plain find_path(EGL/egl.h) reports NOTFOUND even though
  # the header is right there. Pointing HINTS straight at CMAKE_SYSROOT and
  # disabling root-path prefixing for this call sidesteps the double-nesting.
  # This trades away find_path's own cross-safety net, so it is only safe
  # because CMAKE_SYSROOT is confirmed non-empty above -- without that guard,
  # an unset CMAKE_SYSROOT (e.g. cmake/aarch64-musl.cmake) would collapse the
  # HINTS to the literal host path "/usr/include" and search it unprotected.
  find_path(MABUR_EGL_INC EGL/egl.h HINTS "${CMAKE_SYSROOT}/usr/include" NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  find_path(MABUR_GBM_INC gbm.h HINTS "${CMAKE_SYSROOT}/usr/include" NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  find_path(MABUR_RGA_INC rga/im2d.h HINTS "${CMAKE_SYSROOT}/usr/include" NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  find_library(MABUR_EGL_LIB EGL HINTS "${CMAKE_SYSROOT}/usr/lib" NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  find_library(MABUR_GLES_LIB GLESv2 HINTS "${CMAKE_SYSROOT}/usr/lib" NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  find_library(MABUR_GBM_LIB gbm HINTS "${CMAKE_SYSROOT}/usr/lib" NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  find_library(MABUR_RGA_LIB rga HINTS "${CMAKE_SYSROOT}/usr/lib" NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  foreach(v MABUR_EGL_INC MABUR_GBM_INC MABUR_RGA_INC MABUR_EGL_LIB MABUR_GLES_LIB MABUR_GBM_LIB MABUR_RGA_LIB)
    if(NOT ${v})
      message(FATAL_ERROR "MABUR_PLAYER_GPU: ${v} not found in the sysroot (build mesa3d + librga in the Buildroot output; docs/colortrans.md)")
    endif()
  endforeach()
  target_sources(maburplay PRIVATE src/frame_colortrans.cpp)
  target_compile_definitions(maburplay PRIVATE MABUR_PLAYER_GPU=1)
  target_include_directories(maburplay PRIVATE ${MABUR_EGL_INC} ${MABUR_GBM_INC} ${MABUR_RGA_INC})
  target_link_libraries(maburplay PRIVATE ${MABUR_EGL_LIB} ${MABUR_GLES_LIB} ${MABUR_GBM_LIB} ${MABUR_RGA_LIB})
endif()
