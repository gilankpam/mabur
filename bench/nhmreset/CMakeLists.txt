# nhm-reset-probe -- bench check: does a 0x1eb4[25] counter reset clear an
# in-flight ArmNhmBusy window? (header of probe_main.cpp)
if(TARGET devourer)
  add_executable(nhm-reset-probe probe_main.cpp)
  target_link_libraries(nhm-reset-probe PRIVATE devourer pthread)
endif()
