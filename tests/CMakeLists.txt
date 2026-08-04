function(add_mtest name)
  cmake_parse_arguments(MT "" "" "LIBS" ${ARGN})
  add_executable(${name} ${MT_UNPARSED_ARGUMENTS})
  target_link_libraries(${name} PRIVATE mabur_common ${MT_LIBS})
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/third_party)
  target_compile_definitions(${name} PRIVATE
    MABUR_VECTOR_DIR="${CMAKE_CURRENT_SOURCE_DIR}/vectors"
    MABUR_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
  add_test(NAME ${name} COMMAND ${name})
endfunction()
add_mtest(test_smoke test_smoke.cpp)
add_mtest(test_gf256 test_gf256.cpp)
add_mtest(test_sw_wire test_sw_wire.cpp)
add_mtest(test_sw_encoder test_sw_encoder.cpp)
add_mtest(test_sw_ring test_sw_ring.cpp)
add_mtest(test_sw_encoder_async test_sw_encoder_async.cpp)
add_mtest(test_sw_decoder test_sw_decoder.cpp)
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
add_mtest(test_profile test_profile.cpp)
add_mtest(test_node test_node.cpp)
add_mtest(test_msp_dp test_msp_dp.cpp)
add_mtest(test_msp_source test_msp_source.cpp)
add_mtest(test_sweep_wire test_sweep_wire.cpp)
target_include_directories(test_sweep_wire PRIVATE ${CMAKE_SOURCE_DIR}/bench/txagcbench)
if(MABUR_BUILD_DRONE)
  add_mtest(test_config test_config.cpp LIBS mabur_drone_core)
  target_compile_definitions(test_config PRIVATE
    MABUR_BUNDLE_DIR="${CMAKE_SOURCE_DIR}/bundle")
  add_mtest(test_waybeam_client test_waybeam_client.cpp LIBS mabur_drone_core)
  add_mtest(test_frame_source test_frame_source.cpp LIBS mabur_drone_core)
  add_mtest(test_frame_pipeline test_frame_pipeline.cpp LIBS mabur_drone_core)
  add_mtest(test_radio_tx test_radio_tx.cpp LIBS mabur_drone_core)
  add_mtest(test_tx_queue test_tx_queue.cpp LIBS mabur_drone_core)
  add_mtest(test_tx_pool test_tx_pool.cpp LIBS mabur_drone_core)
  add_mtest(test_agent test_agent.cpp LIBS mabur_drone_core)
  add_mtest(test_telemetry test_telemetry.cpp LIBS mabur_drone_core)
  add_mtest(test_power_plan test_power_plan.cpp LIBS mabur_drone_core)

  add_test(NAME host_e2e COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/run_host_e2e.sh)
  set_tests_properties(host_e2e PROPERTIES
    ENVIRONMENT "BUILD=${CMAKE_BINARY_DIR}")
endif()
if(MABUR_BUILD_GS)
  add_mtest(test_gs_config test_gs_config.cpp LIBS mabur_gs_core)
  target_compile_definitions(test_gs_config PRIVATE
    MABUR_GS_BUNDLE_DIR="${CMAKE_SOURCE_DIR}/gs/bundle")
  add_mtest(test_player_config test_player_config.cpp LIBS mabur_player_core)
  target_compile_definitions(test_player_config PRIVATE
    MABUR_PLAY_BUNDLE_DIR="${CMAKE_SOURCE_DIR}/gs/player/bundle")
  add_mtest(test_hevc_params test_hevc_params.cpp LIBS mabur_player_core)
  add_mtest(test_dvr_mux test_dvr_mux.cpp LIBS mabur_player_core)
  add_mtest(test_ring_client test_ring_client.cpp LIBS mabur_player_core pthread)
  add_mtest(test_osd_font test_osd_font.cpp LIBS mabur_player_core)
  add_mtest(test_gs_font test_gs_font.cpp LIBS mabur_player_core)
  target_compile_definitions(test_gs_font PRIVATE
    GEN_GSFONT="${CMAKE_SOURCE_DIR}/tools/msp/gen_gsfont.py")
  add_mtest(test_gs_draw test_gs_draw.cpp LIBS mabur_player_core)
  target_compile_definitions(test_gs_draw PRIVATE
    GEN_GSFONT="${CMAKE_SOURCE_DIR}/tools/msp/gen_gsfont.py")
  add_mtest(test_gs_snapshot test_gs_snapshot.cpp LIBS mabur_player_core)
  add_mtest(test_gs_source test_gs_source.cpp LIBS mabur_player_core)
  add_mtest(test_gs_overlay test_gs_overlay.cpp LIBS mabur_player_core)
  target_compile_definitions(test_gs_overlay PRIVATE
    GEN_GSFONT="${CMAKE_SOURCE_DIR}/tools/msp/gen_gsfont.py")
  add_mtest(test_gs_metrics test_gs_metrics.cpp LIBS mabur_player_core)
  add_mtest(test_osd_layout test_osd_layout.cpp LIBS mabur_player_core)
  add_mtest(test_osd_raster test_osd_raster.cpp LIBS mabur_player_core)
  add_mtest(test_zpos_plan test_zpos_plan.cpp LIBS mabur_player_core)
  add_mtest(test_in_formats test_in_formats.cpp LIBS mabur_player_core)
  add_mtest(test_osd_source test_osd_source.cpp LIBS mabur_player_core)
  add_mtest(test_osd_palette test_osd_palette.cpp LIBS mabur_player_core)
  target_compile_definitions(test_osd_palette PRIVATE
    MABUR_PLAY_BUNDLE_DIR="${CMAKE_SOURCE_DIR}/gs/player/bundle")
  # Wall-clock measurement of the OSD quantizer, which runs INLINE on
  # maburplay's 2 ms main loop. Deliberately NOT an add_test(): it is a
  # timing bench, so neither fast nor deterministic. Build and run by hand
  # (--target quantize_bench); see the file header.
  add_executable(quantize_bench ${CMAKE_SOURCE_DIR}/tools/bench/quantize_bench.cpp)
  target_link_libraries(quantize_bench PRIVATE mabur_common mabur_player_core)
  target_include_directories(quantize_bench PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
  target_compile_definitions(quantize_bench PRIVATE
    MABUR_PLAY_BUNDLE_DIR="${CMAKE_SOURCE_DIR}/gs/player/bundle")
  add_mtest(test_body_queue test_body_queue.cpp LIBS mabur_gs_core)
  add_mtest(test_aggregator test_aggregator.cpp LIBS mabur_gs_core)
  add_mtest(test_udp_sink test_udp_sink.cpp LIBS mabur_gs_core)
  add_mtest(test_msp_sink test_msp_sink.cpp LIBS mabur_gs_core)
  add_mtest(test_msp_e2e test_msp_e2e.cpp LIBS mabur_gs_core)
  add_mtest(test_frame_file_source test_frame_file_source.cpp LIBS mabur_gs_core)
  add_mtest(test_score test_score.cpp LIBS mabur_gs_core)
  add_mtest(test_rendezvous test_rendezvous.cpp LIBS mabur_gs_core)
  add_mtest(test_vrx_controller test_vrx_controller.cpp LIBS mabur_gs_core)
  add_mtest(test_ladder_controller test_ladder_controller.cpp LIBS mabur_gs_core)
  add_mtest(test_s1_loss test_s1_loss.cpp LIBS mabur_gs_core)
  add_mtest(test_tx_selector test_tx_selector.cpp LIBS mabur_gs_core)
  add_mtest(test_control_frame test_control_frame.cpp LIBS mabur_gs_radio)
  add_mtest(test_frame_stream test_frame_stream.cpp LIBS mabur_gs_core)
add_mtest(test_au_ring test_au_ring.cpp LIBS mabur_gs_core pthread)
  add_mtest(test_au_doorbell test_au_doorbell.cpp LIBS mabur_gs_core)
  if(MABUR_BUILD_DRONE)
    # frame_e2e_sustained_shed_no_gaps_no_stalls drives the real drone-side
    # FramePipeline into the GS-side decode/FrameStream path, the same way
    # test_frame_pipeline pulls in mabur_drone_core for drone/src + frame_pipeline.cpp.
    add_mtest(test_frame_e2e test_frame_e2e.cpp LIBS mabur_gs_core mabur_drone_core)
    # Lets the test source compile out FramePipeline usage when this branch
    # isn't taken (see the matching #ifdef in test_frame_e2e.cpp).
    target_compile_definitions(test_frame_e2e PRIVATE MABUR_TEST_HAVE_DRONE_CORE=1)
  else()
    add_mtest(test_frame_e2e test_frame_e2e.cpp LIBS mabur_gs_core)
  endif()
  add_mtest(test_stats_exporter test_stats_exporter.cpp LIBS mabur_gs_core)
  if(MABUR_BUILD_DRONE)
    add_test(NAME gs_e2e COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/run_gs_e2e.sh)
    set_tests_properties(gs_e2e PROPERTIES ENVIRONMENT "BUILD=${CMAKE_BINARY_DIR}")
  endif()
endif()
if(MABUR_BUILD_DRONE AND MABUR_BUILD_GS)
  add_test(NAME gs_au_e2e COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/run_gs_au_e2e.sh)
  set_tests_properties(gs_au_e2e PROPERTIES
    ENVIRONMENT "BUILD=${CMAKE_BINARY_DIR}")
  add_test(NAME player_e2e COMMAND bash ${CMAKE_SOURCE_DIR}/tests/integration/run_player_e2e.sh)
  set_tests_properties(player_e2e PROPERTIES
    ENVIRONMENT "BUILD=${CMAKE_BINARY_DIR}")
endif()
# Host-only burst-loss simulator (spec 2026-07-15-per-layer-symbol-size).
# Not a benchmark of real time — fake clock — so safe as a ctest.
add_executable(burst_sim ${CMAKE_SOURCE_DIR}/tools/bench/burst_sim.cpp)
target_link_libraries(burst_sim PRIVATE mabur_common)
add_test(NAME burst_sim_selfcheck COMMAND burst_sim --self-check)

# Post-flight analysis test (requires Python 3)
find_program(Python3_EXECUTABLE python3)
if(Python3_EXECUTABLE)
  add_test(NAME test_flightreport
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/test_flightreport.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  add_test(NAME test_gen_font
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/test_gen_font.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  add_test(NAME test_gen_gsfont
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tests/test_gen_gsfont.py)
endif()

if(MABUR_BUILD_LINKBENCH)
  add_mtest(test_linkbench_wire test_linkbench_wire.cpp LIBS linkbench_core)
  add_mtest(test_linkbench_pacer test_linkbench_pacer.cpp LIBS linkbench_core)
  add_mtest(test_linkbench_txpipe test_linkbench_txpipe.cpp LIBS linkbench_core)
  add_mtest(test_linkbench_rxpipe test_linkbench_rxpipe.cpp LIBS linkbench_core)
  add_mtest(test_linkbench_e2e test_linkbench_e2e.cpp LIBS linkbench_core)
endif()
