#!/bin/bash
set -e

# 1. 参数，默认 Release
BUILD_TYPE=${1:-Release}
if [[ "$BUILD_TYPE" != "Release" && "$BUILD_TYPE" != "Debug" ]]; then
    echo "Usage: $0 [Release|Debug]"
    exit 1
fi

# 2. 配置 & 编译
BUILD_DIR="build/linux-${BUILD_TYPE}"
cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$BUILD_DIR" --parallel "$(nproc)"
