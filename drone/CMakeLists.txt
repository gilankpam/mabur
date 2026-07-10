add_library(mabur_drone_core STATIC
  src/config.cpp
  src/waybeam_client.cpp
  src/ring_source.cpp
  src/radio_tx.cpp
  src/rc_agent.cpp
  vendor/venc_ring.c)
target_include_directories(mabur_drone_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/vendor
  PRIVATE ${CMAKE_SOURCE_DIR}/third_party)
target_link_libraries(mabur_drone_core PUBLIC mabur_common rt devourer)

add_executable(maburd src/main.cpp)
target_include_directories(maburd PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(maburd PRIVATE mabur_drone_core mabur_common devourer pthread)
