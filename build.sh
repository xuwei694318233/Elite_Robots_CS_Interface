#!/bin/bash
set -e                 # 任何一条命令失败就立即退出
BUILD_DIR=build

# 1. 创建（若已存在则复用）并进入生成目录
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# 2. 生成构建系统
cmake .. -DCMAKE_BUILD_TYPE=Release

# 3. 编译
cmake --build . --parallel "$(nproc)"

