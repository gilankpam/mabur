add_library(mabur_common STATIC src/crc16.cpp src/gf256.cpp src/rs_encoder.cpp src/sbi.cpp src/nal.cpp src/frag.cpp src/uep_encoder.cpp src/rc_proto.cpp src/profile.cpp)
target_include_directories(mabur_common PUBLIC include)
