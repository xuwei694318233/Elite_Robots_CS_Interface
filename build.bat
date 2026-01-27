@echo off
setlocal

:: ---------------- 1. 解析参数 ----------------
:: 参数1: Build Type (Release/Debug)
set BUILD_TYPE=%~1
if "%BUILD_TYPE%"=="" set BUILD_TYPE=Release

if /i not "%BUILD_TYPE%"=="Release" if /i not "%BUILD_TYPE%"=="Debug" (
    echo Usage: %0 [Release^|Debug] [test]
    exit /b 1
)

:: 参数2: Test Option (test) - 默认为 OFF
set TEST_OPT=%~2
set BUILD_TESTING=OFF
if /i "%TEST_OPT%"=="test" set BUILD_TESTING=ON

echo [INFO] Build Type: %BUILD_TYPE%
echo [INFO] Build Testing: %BUILD_TESTING%

:: ---------------- 2. 配置 & 编译 ----------------
set BUILD_DIR=build\win-%BUILD_TYPE%
cmake -B %BUILD_DIR% -A x64 -DBUILD_TESTING=%BUILD_TESTING%
cmake --build %BUILD_DIR% --config %BUILD_TYPE% --parallel

if /i "%BUILD_TESTING%"=="ON" (
    echo [INFO] Tests built. Run them using: ctest --test-dir %BUILD_DIR% -C %BUILD_TYPE% --output-on-failure
)
