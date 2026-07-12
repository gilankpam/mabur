add_library(mabur_gs_core STATIC
  src/config.cpp
  src/aggregator.cpp
  src/udp_sink.cpp
  src/frame_file_source.cpp
  src/energy.cpp
  src/op_table.cpp)
target_include_directories(mabur_gs_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
target_link_libraries(mabur_gs_core PUBLIC mabur_common)

add_executable(maburgs src/main.cpp)
target_include_directories(maburgs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(maburgs PRIVATE mabur_gs_core mabur_common pthread)
