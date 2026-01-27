find_package(GTest CONFIG REQUIRED)

# 复制资源文件到构建目录
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/resources)
configure_file(${CMAKE_SOURCE_DIR}/resources/external_control.script ${CMAKE_CURRENT_BINARY_DIR}/resources/external_control.script COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/resources/input_recipe.txt ${CMAKE_CURRENT_BINARY_DIR}/resources/input_recipe.txt COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/resources/output_recipe.txt ${CMAKE_CURRENT_BINARY_DIR}/resources/output_recipe.txt COPYONLY)

set(TEST_SOURCES
    ConnectionTest.cpp
    MotionTest.cpp
    StateTest.cpp
    PathTest.cpp
)

foreach(SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${SOURCE})
    
    target_link_libraries(${TEST_NAME} PRIVATE EliteCSRobot GTest::gtest GTest::gtest_main)
    target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)
    
    # Windows 平台 DLL 拷贝逻辑（复用根目录逻辑）
    if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        # 1. 拷贝运行时依赖 (EliteCSRobot.dll 等)
        add_custom_command(TARGET ${TEST_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_RUNTIME_DLLS:${TEST_NAME}>"
            "$<TARGET_FILE_DIR:${TEST_NAME}>"
            COMMAND_EXPAND_LISTS)

        # 2. 拷贝 SDK 依赖
        add_custom_command(TARGET ${TEST_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "$<IF:$<CONFIG:Debug>,${SDK_LIB_DIR_DEBUG},${SDK_LIB_DIR_RELEASE}>"
            "$<TARGET_FILE_DIR:${TEST_NAME}>")
    endif()

    # 注册测试
    include(GoogleTest)
    gtest_discover_tests(${TEST_NAME})
endforeach()
