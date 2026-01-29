# GTest 已由根目录 add_subdirectory(thirdparty/gtest) 生成，直接链接
# 资源文件复制（与平台无关）
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/resources)
configure_file(${CMAKE_SOURCE_DIR}/resources/external_control.script
               ${CMAKE_CURRENT_BINARY_DIR}/resources/external_control.script COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/resources/input_recipe.txt
               ${CMAKE_CURRENT_BINARY_DIR}/resources/input_recipe.txt COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/resources/output_recipe.txt
               ${CMAKE_CURRENT_BINARY_DIR}/resources/output_recipe.txt COPYONLY)

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

    # ========== 仅 Windows：拷贝运行时 DLL（不会冲突） ==========
    if(WIN32)
        add_custom_command(TARGET ${TEST_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_RUNTIME_DLLS:${TEST_NAME}>"
            "$<TARGET_FILE_DIR:${TEST_NAME}>"
            COMMAND_EXPAND_LISTS)
    endif()

    include(GoogleTest)
    gtest_discover_tests(${TEST_NAME})
endforeach()
