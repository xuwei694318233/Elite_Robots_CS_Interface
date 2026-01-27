#!/bin/bash
set -e

# 1. 参数解析
BUILD_TYPE=${1:-Release}
TEST_OPT=${2:-off}

if [[ "$BUILD_TYPE" != "Release" && "$BUILD_TYPE" != "Debug" ]]; then
    echo "Usage: $0 [Release|Debug] [test]"
    exit 1
fi

BUILD_TESTING="OFF"
if [[ "$TEST_OPT" == "test" ]]; then
    BUILD_TESTING="ON"
fi

echo "[INFO] Build Type: $BUILD_TYPE"
echo "[INFO] Build Testing: $BUILD_TESTING"

# 2. 配置 & 编译
BUILD_DIR="build/linux-${BUILD_TYPE}"
cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE" -DBUILD_TESTING="$BUILD_TESTING"
cmake --build "$BUILD_DIR" --parallel "$(nproc)"

if [[ "$BUILD_TESTING" == "ON" ]]; then
    echo "[INFO] Tests built. Run them using: ctest --test-dir $BUILD_DIR --output-on-failure"
fi
