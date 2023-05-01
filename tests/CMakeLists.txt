if(ENABLE_TESTING)
    add_subdirectory(CustomClass)

    set(TEST_SOURCES "tests.cpp")

    foreach(SOURCE IN LISTS TEST_SOURCES)
        get_filename_component(TEST_NAME ${SOURCE} NAME_WLE)
        # message(STATUS "Creating test: " ${TEST_NAME} " from source: " ${SOURCE})
        
        add_executable(${TEST_NAME} ${SOURCE})
        target_link_libraries(${TEST_NAME} PUBLIC simpleGraphs customClass)
        # add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME} WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})

        # NOTE: code-coverage.cmake does not seem to work with test executables, so I had to edit 
        # test file paths in the tests/*.cpp files to navigate to test cases from the build directory
        # TODO: find a better way to do this 
        if(ENABLE_COVERAGE_REPORT)
            target_code_coverage(${TEST_NAME} AUTO ALL)
        endif()
    endforeach()
endif()
