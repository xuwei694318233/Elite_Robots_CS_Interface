@echo off
setlocal

:: --------------- 1. 读取参数 -----------------
set VS_VER=%1
if "%VS_VER%"=="" set VS_VER=2022
set BUILD_TYPE=%2
if "%BUILD_TYPE%"=="" set BUILD_TYPE=Release

:: 参数校验
if /i not "%VS_VER%"=="2019" if /i not "%VS_VER%"=="2022" (
    echo Usage: %0 [2019^|2022] [Release^|Debug]
    exit /b 1
)
if /i not "%BUILD_TYPE%"=="Release" if /i not "%BUILD_TYPE%"=="Debug" (
    echo Usage: %0 [2019^|2022] [Release^|Debug]
    exit /b 1
)

:: 根据版本设 generator 名 与 vcvars 路径
if /i "%VS_VER%"=="2019" (
    set GENERATOR=Visual Studio 16 2019
    set VCVARS_VER=2019
) else (
    set GENERATOR=Visual Studio 17 2022
    set VCVARS_VER=2022
)

:: --------------- 2. 加载 VS 环境 ---------------
for %%e in (Community Professional Enterprise) do (
    set VCVARS=C:\Program Files\Microsoft Visual Studio\!VCVARS_VER!\%%e\VC\Auxiliary\Build\vcvars64.bat
    if exist "!VCVARS!" call "!VCVARS!" & goto :found
)
echo 未找到 VS!VCVARS_VER! 环境，请手动运行对应 vcvars64.bat 再执行脚本
exit /b 1
:found

:: --------------- 3. 配置 & 编译 ---------------
set BUILD_DIR=build\win-%VS_VER%\%BUILD_TYPE%
cmake -B %BUILD_DIR% -G "%GENERATOR%" -A x64
cmake --build %BUILD_DIR% --config %BUILD_TYPE% --parallel
