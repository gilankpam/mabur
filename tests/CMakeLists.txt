find_program(Python3_EXECUTABLE python3)

# Where tests that must write a file put it: inside the build tree, so it
# goes with the build directory. See tests/scratch.h for why not /tmp.
set(MABUR_TEST_SCRATCH_DIR ${CMAKE_CURRENT_BINARY_DIR}/scratch)
file(MAKE_DIRECTORY ${MABUR_TEST_SCRATCH_DIR})

function(add_mtest name)
  cmake_parse_arguments(MT "" "" "LIBS" ${ARGN})
  add_executable(${name} ${MT_UNPARSED_ARGUMENTS})
  target_link_libraries(${name} PRIVATE mabur_common ${MT_LIBS})
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/third_party)
  target_compile_definitions(${name} PRIVATE
    MABUR_VECTOR_DIR="${CMAKE_CURRENT_SOURCE_DIR}/vectors"
    MABUR_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures"
    MABUR_BUNDLE_DIR="${PROJECT_SOURCE_DIR}/bundle"
    MABUR_TEST_SCRATCH_DIR="${MABUR_TEST_SCRATCH_DIR}")
  add_test(NAME ${name} COMMAND ${name})
endfunction()

add_mtest(test_smoke test_smoke.cpp)
add_mtest(test_crc16 test_crc16.cpp)
add_mtest(test_channel_set test_channel_set.cpp LIBS mabur_common)
add_mtest(test_channel_file test_channel_file.cpp LIBS mabur_common)
add_mtest(test_siphash test_siphash.cpp)
add_mtest(test_link_key test_link_key.cpp)
# CRC-16 microbench (tools/bench/crc_bench.cpp): table vs bit-serial, ns/B.
# Not a test; --target crc_bench.
add_executable(crc_bench ${PROJECT_SOURCE_DIR}/tools/bench/crc_bench.cpp)
target_link_libraries(crc_bench PRIVATE mabur_common)
add_mtest(test_gf256 test_gf256.cpp)
add_mtest(test_sw_wire test_sw_wire.cpp)
add_mtest(test_sw_encoder test_sw_encoder.cpp)
add_mtest(test_sw_ring test_sw_ring.cpp)
add_mtest(test_sw_encoder_async test_sw_encoder_async.cpp)
add_mtest(test_sw_decoder test_sw_decoder.cpp)
add_mtest(test_nack_tracker test_nack_tracker.cpp)
add_mtest(test_arrival_tracker test_arrival_tracker.cpp)
add_mtest(test_sbi test_sbi.cpp)
add_mtest(test_sbi_unpack test_sbi_unpack.cpp)
add_mtest(test_nal test_nal.cpp)
add_mtest(test_frag test_frag.cpp)
add_mtest(test_frame_wire test_frame_wire.cpp)
add_mtest(test_uep test_uep.cpp)
add_mtest(test_uep_sw test_uep_sw.cpp)
add_mtest(test_uep_decoder test_uep_decoder.cpp)
add_mtest(test_uep_frame test_uep_frame.cpp)
add_mtest(test_rc test_rc.cpp)
add_mtest(test_mav_lib test_mav_lib.cpp)
add_mtest(test_cal_mav test_cal_mav.cpp)
add_mtest(test_config_params test_config_params.cpp)
add_mtest(test_camera_mode_table test_camera_mode_table.cpp)
add_mtest(test_param_value test_param_value.cpp)
add_mtest(test_hevc_params test_hevc_params.cpp)
add_mtest(test_hevc_bits test_hevc_bits.cpp)
add_mtest(test_hevc_ps test_hevc_ps.cpp)
add_mtest(test_hevc_slice test_hevc_slice.cpp)
add_mtest(test_hevc_skip_slice test_hevc_skip_slice.cpp)
add_mtest(test_dvr_mux test_dvr_mux.cpp)
add_mtest(test_raw_dvr test_raw_dvr.cpp)
add_mtest(test_dvr_name test_dvr_name.cpp)
add_mtest(test_threaded_sink test_threaded_sink.cpp)
add_mtest(test_probe_wire test_probe_wire.cpp)
add_mtest(test_retx_ring test_retx_ring.cpp)
add_mtest(test_profile test_profile.cpp)
add_mtest(test_node test_node.cpp)
add_mtest(test_msp_dp test_msp_dp.cpp)
add_mtest(test_msp_source test_msp_source.cpp)
add_mtest(test_msp_status test_msp_status.cpp)
add_mtest(test_cal_wire test_cal_wire.cpp)
add_mtest(test_toml test_toml.cpp)
if(MABUR_BUILD_DRONE)
  add_mtest(test_config test_config.cpp LIBS mabur_drone_core)
  add_mtest(test_frame_source test_frame_source.cpp LIBS mabur_drone_core)
  add_mtest(test_frame_pipeline test_frame_pipeline.cpp LIBS mabur_drone_core)
  add_mtest(test_radio_tx test_radio_tx.cpp LIBS mabur_drone_core)
  add_mtest(test_tx_queue test_tx_queue.cpp LIBS mabur_drone_core)
  add_mtest(test_nack_bucket test_nack_bucket.cpp LIBS mabur_drone_core)
  add_mtest(test_tx_pool test_tx_pool.cpp LIBS mabur_drone_core)
  add_mtest(test_agent test_agent.cpp LIBS mabur_drone_core)
  add_mtest(test_vtx_recorder test_vtx_recorder.cpp LIBS mabur_drone_core)
  add_mtest(test_rec_frame test_rec_frame.cpp LIBS mabur_drone_core)
  add_mtest(test_tick_gate test_tick_gate.cpp LIBS mabur_drone_core)
  add_mtest(test_air_clock test_air_clock.cpp LIBS mabur_drone_core)
  add_mtest(test_ampdu_policy test_ampdu_policy.cpp LIBS mabur_drone_core)
  add_mtest(test_air_rate test_air_rate.cpp LIBS mabur_drone_core)
  add_mtest(test_peak_rate test_peak_rate.cpp LIBS mabur_drone_core)
  add_mtest(test_mi_ready test_mi_ready.cpp LIBS mabur_drone_core)
  add_mtest(sim_shed_lag sim_shed_lag.cpp LIBS mabur_drone_core)
  add_mtest(test_telemetry test_telemetry.cpp LIBS mabur_drone_core)
  add_mtest(test_power_plan test_power_plan.cpp LIBS mabur_drone_core)
  add_mtest(test_power_tables test_power_tables.cpp LIBS mabur_drone_core)
  add_mtest(test_cal_apply test_cal_apply.cpp LIBS mabur_drone_core)
  add_mtest(test_config_write test_config_write.cpp LIBS mabur_common)
  add_mtest(test_config_apply test_config_apply.cpp LIBS mabur_drone_core)
  add_mtest(test_mav_params test_mav_params.cpp LIBS mabur_drone_core)
  add_mtest(test_mav_component test_mav_component.cpp LIBS mabur_drone_core)
  add_mtest(test_cal_sweep test_cal_sweep.cpp LIBS mabur_drone_core)
  add_mtest(test_cal_link test_cal_link.cpp LIBS mabur_drone_core)
  add_mtest(test_live_power_table test_live_power_table.cpp LIBS mabur_drone_core)
  add_mtest(test_debug_http test_debug_http.cpp LIBS mabur_drone_core)
  add_mtest(test_probe_source test_probe_source.cpp LIBS mabur_drone_core)

  # Host coverage for the carried waybeam venc pure modules (Task B8).
  # These drone/venc/*.c files are ARM-only in venc_core (drone/
  # CMakeLists.txt only globs venc/*.c under CMAKE_CROSSCOMPILING), so each
  # test target compiles its own module source directly rather than
  # linking mabur_drone_core/venc_core. venc_cfg.c is the one module
  # already compiled into mabur_drone_core unconditionally (host+ARM) —
  # test_venc_cfg links that instead of recompiling it, to avoid defining
  # venc_cfg_defaults() twice in one link.
  set(MABUR_VENC_DIR ${PROJECT_SOURCE_DIR}/drone/venc)
  add_mtest(test_venc_idr_rate_limit test_venc_idr_rate_limit.cpp
    ${MABUR_VENC_DIR}/idr_rate_limit.c ${MABUR_VENC_DIR}/timing.c)
  target_include_directories(test_venc_idr_rate_limit PRIVATE ${MABUR_VENC_DIR})
  add_mtest(test_venc_h26x_util test_venc_h26x_util.cpp
    ${MABUR_VENC_DIR}/h26x_util.c)
  target_include_directories(test_venc_h26x_util PRIVATE ${MABUR_VENC_DIR})
  add_mtest(test_venc_h26x_param_sets test_venc_h26x_param_sets.cpp
    ${MABUR_VENC_DIR}/h26x_param_sets.c)
  target_include_directories(test_venc_h26x_param_sets PRIVATE ${MABUR_VENC_DIR})
  add_mtest(test_venc_cfg test_venc_cfg.cpp LIBS mabur_drone_core)
  add_mtest(test_camera_modes test_camera_modes.cpp ${MABUR_VENC_DIR}/camera_modes.c)
  target_include_directories(test_camera_modes PRIVATE ${MABUR_VENC_DIR})
  add_mtest(test_sensor_cal test_sensor_cal.cpp ${MABUR_VENC_DIR}/sensor_cal.c)
  target_include_directories(test_sensor_cal PRIVATE ${MABUR_VENC_DIR})
  add_mtest(test_sensor_steer test_sensor_steer.cpp ${MABUR_VENC_DIR}/sensor_steer.c)
  target_include_directories(test_sensor_steer PRIVATE ${MABUR_VENC_DIR}
    ${PROJECT_SOURCE_DIR}/common/include)
  # The real venc_core.c facade over a fake star6e runtime: its lifecycle
  # guards are what the rebuild failure ladder stands on.
  add_mtest(test_venc_core test_venc_core.cpp venc_core_fake.c
    ${MABUR_VENC_DIR}/venc_core.c)
  target_include_directories(test_venc_core PRIVATE ${MABUR_VENC_DIR}
    ${PROJECT_SOURCE_DIR}/drone/vendor)
  target_compile_definitions(test_venc_core PRIVATE
    PLATFORM_STAR6E HAVE_BACKEND_STAR6E=1 _GNU_SOURCE)
  target_link_libraries(test_venc_core PRIVATE pthread)

  add_test(NAME host_e2e COMMAND bash ${PROJECT_SOURCE_DIR}/tests/integration/run_host_e2e.sh)
  set_tests_properties(host_e2e PROPERTIES
    ENVIRONMENT "BUILD=${PROJECT_BINARY_DIR}")
endif()
# Host-only burst-loss simulator (spec 2026-07-15-per-layer-symbol-size).
# Not a benchmark of real time — fake clock — so safe as a ctest.
add_executable(burst_sim ${PROJECT_SOURCE_DIR}/tools/bench/burst_sim.cpp)
target_link_libraries(burst_sim PRIVATE mabur_common)
add_test(NAME burst_sim_selfcheck COMMAND burst_sim --self-check)

# Python tool tests (requires Python 3; found at the top of this file)
if(Python3_EXECUTABLE)
  add_test(NAME test_mavparam
    COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/test_mavparam.py)
  add_test(NAME test_vencburst
    COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/test_vencburst.py
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  add_test(NAME test_release
    COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/test_release.py)
endif()

if(MABUR_BUILD_LINKBENCH)
  add_mtest(test_linkbench_wire test_linkbench_wire.cpp LIBS linkbench_core)
  add_mtest(test_linkbench_pacer test_linkbench_pacer.cpp LIBS linkbench_core)
  add_mtest(test_linkbench_txpipe test_linkbench_txpipe.cpp LIBS linkbench_core)
  add_mtest(test_linkbench_rxpipe test_linkbench_rxpipe.cpp LIBS linkbench_core)
  add_mtest(test_linkbench_e2e test_linkbench_e2e.cpp LIBS linkbench_core)
endif()
