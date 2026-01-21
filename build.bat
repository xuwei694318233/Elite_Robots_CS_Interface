@echo off
setlocal

:: ---------------- 1. ≤Œ ˝£®ƒ¨»œ Release£© ----------------
set BUILD_TYPE=%~1
if "%BUILD_TYPE%"=="" set BUILD_TYPE=Release
if /i not "%BUILD_TYPE%"=="Release" if /i not "%BUILD_TYPE%"=="Debug" (
    echo Usage: %0 [Release^|Debug]
    exit /b 1
)

:: ---------------- 2. ≈‰÷√ & ±‡“Î ----------------
set BUILD_DIR=build\win-%BUILD_TYPE%
cmake -B %BUILD_DIR% -A x64
cmake --build %BUILD_DIR% --config %BUILD_TYPE% --parallel
