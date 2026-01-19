cmake_minimum_required(VERSION 3.16)
project(EliteCSRobot LANGUAGES CXX)

# ---------------- 基础 ----------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 手动指定构建类型（命令行 -DCMAKE_BUILD_TYPE=Debug/Release 可覆盖）
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------- 1. 编译成动态库 EliteCSRobot ----------------
add_library(EliteCSRobot SHARED
    source/EliteCSRobot.cpp
)
target_include_directories(EliteCSRobot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EliteSDK/include
)

# 2. 链接第三方 EliteSDK 动态库
add_library(EliteSDK SHARED IMPORTED GLOBAL)
if(WIN32)
    set_target_properties(EliteSDK PROPERTIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EliteSDK/bin/elite-cs-series-sdk.dll
        IMPORTED_IMPLIB   ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EliteSDK/bin/elite-cs-series-sdk.lib
    )
else()
    set_target_properties(EliteSDK PROPERTIES
        IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/EliteSDK/bin/libelite-cs-series-sdk.so
    )
endif()
target_link_libraries(EliteCSRobot PRIVATE EliteSDK)

# 3. 编译可执行文件 test（含 main）
add_executable(test_exe source/test.cpp)
# 链接自己刚生成的动态库
target_link_libraries(test_exe PRIVATE EliteCSRobot)

# 4. Windows 自动把运行时 DLL 拷到可执行文件旁边
if(WIN32)
    add_custom_command(TARGET test_exe POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:test_exe>
            $<TARGET_FILE_DIR:test_exe>
        COMMAND_EXPAND_LISTS
    )
endif()
