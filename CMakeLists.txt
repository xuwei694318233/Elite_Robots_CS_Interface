cmake_minimum_required(VERSION 3.16)
project(EliteCSRobot LANGUAGES CXX)

# ===== 全项目统一 UTF-8（仅 MSVC 需要） =====
if(MSVC)
    add_compile_options(/utf-8)          # 源码字符集 + 执行字符集均 UTF-8
endif()

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

# -------------------- 导入第三方 SDK（多配置友好） --------------------
# include_directories removed: dependencies prefer target_include_directories

# 1. 定义两套 IMPORTED 目标（Debug / Release）
set(SDK_LIB_DIR_DEBUG   "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EliteSDK/bin/${PLAT}/Debug")
set(SDK_LIB_DIR_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EliteSDK/bin/${PLAT}/Release")

# Debug 版
add_library(elite_sdk_debug SHARED IMPORTED)
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set_target_properties(elite_sdk_debug PROPERTIES
        IMPORTED_LOCATION "${SDK_LIB_DIR_DEBUG}/${LIB_PREFIX}elite-cs-series-sdk.${LIB_EXT}"
        IMPORTED_IMPLIB   "${SDK_LIB_DIR_DEBUG}/${LIB_PREFIX}elite-cs-series-sdk.${IMPLIB_EXT}")
else()
    set_target_properties(elite_sdk_debug PROPERTIES
        IMPORTED_LOCATION "${SDK_LIB_DIR_DEBUG}/${LIB_PREFIX}elite-cs-series-sdk.${LIB_EXT}")
endif()

# Release 版
add_library(elite_sdk_release SHARED IMPORTED)
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set_target_properties(elite_sdk_release PROPERTIES
        IMPORTED_LOCATION "${SDK_LIB_DIR_RELEASE}/${LIB_PREFIX}elite-cs-series-sdk.${LIB_EXT}"
        IMPORTED_IMPLIB   "${SDK_LIB_DIR_RELEASE}/${LIB_PREFIX}elite-cs-series-sdk.${IMPLIB_EXT}")
else()
    set_target_properties(elite_sdk_release PROPERTIES
        IMPORTED_LOCATION "${SDK_LIB_DIR_RELEASE}/${LIB_PREFIX}elite-cs-series-sdk.${LIB_EXT}")
endif()

# 2. 用接口库包装，让下游通过生成器表达式自动选 Debug/Release
add_library(elite_sdk INTERFACE)
target_link_libraries(elite_sdk INTERFACE
    $<$<CONFIG:Debug>:elite_sdk_debug>
    $<$<CONFIG:Release>:elite_sdk_release>)

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
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EliteSDK/include>
        $<INSTALL_INTERFACE:include>)
target_compile_definitions(EliteCSRobot PRIVATE ROBOT_EXPORT_LIBRARY)
target_link_libraries(EliteCSRobot PUBLIC elite_sdk)   # 自动切换 Debug/Release
apply_build_config(EliteCSRobot)

# -------------------- 2. 可执行文件（仅链接自己的动态库） --------------------
add_executable(elite_test source/test.cpp)
target_link_libraries(elite_test PRIVATE EliteCSRobot)
apply_build_config(elite_test)

# -------------------- Windows：复制整套 SDK bin 目录（含 pdb、附加 dll） --------------------
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    # 1. 最小运行时依赖
    add_custom_command(TARGET elite_test POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_RUNTIME_DLLS:elite_test>"
        "$<TARGET_FILE_DIR:elite_test>"
        COMMAND_EXPAND_LISTS)

    # 2. 整套 Debug/Release 文件夹
    add_custom_command(TARGET elite_test POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "$<IF:$<CONFIG:Debug>,${SDK_LIB_DIR_DEBUG},${SDK_LIB_DIR_RELEASE}>"
        "$<TARGET_FILE_DIR:elite_test>")
endif()

# -------------------- 测试 --------------------
option(BUILD_TESTING "Build the testing tree." ON)
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(test)
endif()
