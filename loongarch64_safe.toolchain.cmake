set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR loongarch64)

# 绝对路径指向你现有的 64 位 rc1.4 工具链
set(TOOLCHAIN_DIR "/home/lin/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.4")

set(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}/bin/loongarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}/bin/loongarch64-linux-gnu-g++)

set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_DIR}/loongarch64-linux-gnu)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)