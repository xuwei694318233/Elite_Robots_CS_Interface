#!/bin/bash
set -e

# 1. 读取参数，默认 Release
BUILD_TYPE=${1:-Release}
# 统一小写，拼路径
TYPE_LC=${BUILD_TYPE,,}
BUILD_DIR="build/linux/${TYPE_LC}"

if [[ "$BUILD_TYPE" != "Release" && "$BUILD_TYPE" != "Debug" ]]; then
    echo "Usage: $0 [Release|Debug]"
    exit 1
fi

# 2. 配置
cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

# 3. 编译
cmake --build "$BUILD_DIR" --parallel "$(nproc)"
