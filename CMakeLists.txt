cmake_minimum_required(VERSION 3.16)
project(EliteCSRobot LANGUAGES CXX)

# -------------------- 基础选项 --------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)   # 动态库需要 -fPIC

# -------------------- 平台 / 构建类型 --------------------
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(PLAT "win-x64")
    set(LIB_PREFIX "")
    set(LIB_EXT "dll")
    set(IMPLIB_EXT "lib")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(PLAT "linux-x64")
    set(LIB_PREFIX "lib")
    set(LIB_EXT "so")
else()
    message(FATAL_ERROR "Unsupported platform")
endif()

# 根据 Debug/Release 选第三方库目录
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EliteSDK/bin/${PLAT}/Debug")
else()
    set(SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EliteSDK/bin/${PLAT}/Release")
endif()

# -------------------- 导入第三方 SDK --------------------
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EliteSDK/include")

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    add_library(elite_sdk SHARED IMPORTED)
    set_target_properties(elite_sdk PROPERTIES
        IMPORTED_LOCATION "${SDK_LIB_DIR}/${LIB_PREFIX}elite-cs-series-sdk.${LIB_EXT}"
        IMPORTED_IMPLIB   "${SDK_LIB_DIR}/${LIB_PREFIX}elite-cs-series-sdk.${IMPLIB_EXT}")
else()
    add_library(elite_sdk SHARED IMPORTED)
    set_target_properties(elite_sdk PROPERTIES
        IMPORTED_LOCATION "${SDK_LIB_DIR}/${LIB_PREFIX}elite-cs-series-sdk.${LIB_EXT}")
endif()

# -------------------- Debug/Release 编译区分 --------------------
function(apply_build_config target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            $<$<CONFIG:Debug>:   /Od /Zi /MDd>
            $<$<CONFIG:Release>: /O2 /MD>)
        target_link_options(${target} PRIVATE
            $<$<CONFIG:Debug>:   /DEBUG>
            $<$<CONFIG:Release>: /INCREMENTAL:NO>)
    else()
        target_compile_options(${target} PRIVATE
            $<$<CONFIG:Debug>:   -g -O0 -DELITE_DEBUG>
            $<$<CONFIG:Release>: -O3 -DNDEBUG>)
        target_link_options(${target} PRIVATE
            $<$<CONFIG:Release>: -Wl,--strip-all>)
    endif()
endfunction()

# -------------------- 1. 自己的动态库（链接第三方 SDK） --------------------
add_library(EliteCSRobot SHARED source/EliteCSRobot.cpp)
target_include_directories(EliteCSRobot
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
# ★★★ 第三方库仅 PRIVATE 链接，对外隐藏
target_link_libraries(EliteCSRobot PRIVATE elite_sdk)
apply_build_config(EliteCSRobot)

# -------------------- 2. 可执行文件（仅链接自己的动态库） --------------------
add_executable(elite_test source/test.cpp)
# ★★★ 不再直接链接 elite_sdk
target_link_libraries(elite_test PRIVATE EliteCSRobot)
apply_build_config(elite_test)

# -------------------- Windows 自动拷运行时 DLL --------------------
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    add_custom_command(TARGET elite_test POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_RUNTIME_DLLS:elite_test>"
        "$<TARGET_FILE_DIR:elite_test>"
        COMMAND_EXPAND_LISTS)
endif()
