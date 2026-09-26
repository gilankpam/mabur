# THROWAWAY SPIKE (branch wasm-spike). Standalone: builds rxprobe natively or,
# under emcmake, as rxprobe.js/.wasm for the browser. See spike/wasm/README.md.
cmake_minimum_required(VERSION 3.16)
project(rxprobe CXX C)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(DEVOURER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../devourer" CACHE PATH "devourer checkout")
foreach(opt JAGUAR1 8814 JAGUAR2_8822B JAGUAR2_8821C JAGUAR3_8822C 8733B KESTREL_8852B KESTREL_8852C)
  set(DEVOURER_${opt} OFF CACHE BOOL "" FORCE)
endforeach()
set(DEVOURER_JAGUAR3_8822E ON CACHE BOOL "" FORCE)
set(DEVOURER_LOG_MAX_LEVEL WARN CACHE STRING "" FORCE)
if(EMSCRIPTEN)
  add_compile_options(-pthread)
endif()
add_subdirectory(${DEVOURER_DIR} ${CMAKE_BINARY_DIR}/devourer EXCLUDE_FROM_ALL)
add_executable(rxprobe rxprobe.cpp)
target_include_directories(rxprobe PRIVATE ${DEVOURER_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/../../common/include)
target_link_libraries(rxprobe PRIVATE devourer)
if(EMSCRIPTEN)
  target_link_options(rxprobe PRIVATE
    -pthread -sPROXY_TO_PTHREAD -sPTHREAD_POOL_SIZE=8 -sALLOW_MEMORY_GROWTH
    -sASYNCIFY -sMODULARIZE -sEXPORT_NAME=createRxProbe -sEXIT_RUNTIME
    -sENVIRONMENT=web,worker --bind)
endif()
