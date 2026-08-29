add_library(mabur_drone_core STATIC
  src/config.cpp
  src/debug_http.cpp
  src/frame_source.cpp
  src/frame_pipeline.cpp
  src/radio_tx.cpp
  src/rc_agent.cpp
  src/msp_serial.cpp
  src/telemetry.cpp
  vendor/venc_frame_ring.c
  # venc_cfg.c is dependency-free C99 (errno/stdlib/string only), so it
  # compiles on host too — config.cpp's parse_venc() needs
  # venc_cfg_preset_known() at host-test time, not just on the ARM
  # cross-build. Excluded from venc_core's glob below to avoid compiling it
  # twice into the same link.
  venc/venc_cfg.c)
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
  # venc_cfg.c is already a source of mabur_drone_core (unconditionally,
  # for host builds) — don't compile it a second time into venc_core.
  list(REMOVE_ITEM VENC_CORE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/venc/venc_cfg.c)
  # NOTE: waybeam links these translation units as a flat .o list; here they
  # become an archive, and an archive member is pulled in only to satisfy an
  # UNDEFINED symbol. Any "registers itself / overrides a weak default"
  # pattern silently loses its object. venc_jpeg.c's weak backend stubs were
  # exactly that (star6e_jpeg.c.o never linked -> snapshot ENOSYS); they were
  # deleted rather than papered over with --whole-archive, so a missing
  # backend is now a link error. Keep new files free of that pattern.
  add_library(venc_core STATIC ${VENC_CORE_SRC})
  target_include_directories(venc_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/venc ${CMAKE_CURRENT_SOURCE_DIR}/vendor)
  target_compile_definitions(venc_core PRIVATE
    PLATFORM_STAR6E HAVE_BACKEND_STAR6E=1 _GNU_SOURCE)
  # -Os, overriding the Release -O3, to match the optimisation level waybeam
  # ships these exact translation units at. Not cosmetic: the VIF->VPE
  # "layout type 2, bindmode 4 not sync err" / "ISP channel [0] have NOT been
  # created" failure class on this SoC is documented upstream
  # (waybeam documentation/VIF_VPE_SYNC_ERROR_POSTMORTEM.md) as
  # binary-layout-dependent UB from MI struct handling, where "any binary
  # layout change -- new variables, different optimization -- can trigger the
  # latent bug". The known instance is fixed (i6e_vpe_chn/i6e_vpe_para
  # typedefs, which this build does use -- verified sizeof 196/100), but
  # running the SDK-facing code at an optimisation level nobody has flown it
  # at buys nothing. The encoder path is I/O-bound; waybeam does 1080p60 at
  # -Os on this chip.
  target_compile_options(venc_core PRIVATE -Os)
  target_link_libraries(venc_core PUBLIC dl pthread rt m)
  # maburd calls the venc_core.h facade directly on ARM. MABUR_HAVE_VENC
  # guards those call sites so the host build (which compiles none of
  # drone/venc) still builds and tests.
  target_link_libraries(mabur_drone_core PUBLIC venc_core)
  # Declared cycle, on purpose: venc_core's star6e_output.c calls
  # venc_frame_ring_{create,destroy,...}, which mabur_drone_core compiles
  # from drone/vendor. CMake repeats archives in a static-library cycle, so
  # the link does not depend on frame_source.cpp happening to pull that
  # object in first.
  target_link_libraries(venc_core PUBLIC mabur_drone_core)
  target_compile_definitions(mabur_drone_core PUBLIC MABUR_HAVE_VENC)
endif()
