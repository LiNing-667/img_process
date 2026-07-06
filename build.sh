#!/bin/bash
mkdir -p build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../loongarch64_safe.toolchain.cmake ..
make -j$(nproc)