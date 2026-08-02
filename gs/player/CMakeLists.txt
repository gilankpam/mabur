add_library(mabur_player_core STATIC
  src/player_config.cpp
  src/hevc_params.cpp
  src/dvr_mux.cpp
  src/ring_client.cpp)
target_include_directories(mabur_player_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
target_link_libraries(mabur_player_core PUBLIC mabur_gs_core)

# maburplay: main.cpp + the null backend are always built (host and cross).
# Hardware backends (mpp_backend.cpp, drm_presenter.cpp) are appended here
# behind MABUR_PLAYER_HW starting Task 7.
add_executable(maburplay
  src/main.cpp
  src/null_backend.cpp)
target_link_libraries(maburplay PRIVATE mabur_player_core)
