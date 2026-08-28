# Cross toolchain: OpenIPC Buildroot glibc ARMv7 hard-float, DYNAMIC.
# Required because the venc core dlopens the drone's glibc-built SigmaStar
# MI libraries — a static-musl binary cannot dlopen at all (fold-in Task B1).
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(OPENIPC_HOST_BIN "$ENV{OPENIPC_HOST_BIN}")
set(CMAKE_C_COMPILER   ${OPENIPC_HOST_BIN}/arm-openipc-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER ${OPENIPC_HOST_BIN}/arm-openipc-linux-gnueabihf-g++)
set(CMAKE_C_FLAGS_INIT   "-Os -mfpu=neon-vfpv4 -mfloat-abi=hard")
set(CMAKE_CXX_FLAGS_INIT "-Os -mfpu=neon-vfpv4 -mfloat-abi=hard")
set(CMAKE_FIND_ROOT_PATH "$ENV{MABUR_GLIBC_STAGING}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
