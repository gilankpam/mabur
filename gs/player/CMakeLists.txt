add_library(mabur_player_core STATIC
  src/player_config.cpp
  src/hevc_params.cpp)
target_include_directories(mabur_player_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
target_link_libraries(mabur_player_core PUBLIC mabur_gs_core)
