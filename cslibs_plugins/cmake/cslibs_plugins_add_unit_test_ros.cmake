# ${PROJECT_NAME}_add_unit_test_gtest(<NAME_OF_THE_TEST>
#    [<LAUNCH_FILE> ...]
#    [<INCLUDE_DIRS> ...]
#    [<SOURCE_FILES> ...]
#    [<LINK_LIBRARIES> ...]
#    [<COMPILE_FLAGS> ...]
#)
function(${PROJECT_NAME}_add_unit_test_ros)
    cmake_parse_arguments(unit_test
        # list of names of the boolean arguments (only defined ones will be true)
        ""
        # list of names of mono-valued arguments
        "LAUNCH_FILE"
        # list of names of multi-valued arguments (output variables are lists)
        "INCLUDE_DIRS;SOURCE_FILES;LINK_LIBRARIES;COMPILE_OPTIONS"
        # arguments of the function to parse, here we take the all original ones
        ${ARGN}
    )

    find_package(rostest QUIET)
    find_package(roscpp QUIET)
    if(${rostest_FOUND})
        set(unit_test_NAME ${ARGV0})
        add_rostest_gtest(${unit_test_NAME}
                          ${unit_test_LAUNCH_FILE}
                          ${unit_test_SOURCE_FILES})

        target_include_directories(${unit_test_NAME}
            PRIVATE
                ${unit_test_INCLUDE_DIRS}
                ${rostest_INCLUDE_DIRS}
                ${roscpp_INCLUDE_DIRS}
        )
        target_link_libraries(${unit_test_NAME}
            #PRIVATE
                ${unit_test_LIBS}
                ${unit_test_LINK_LIBRARIES}
                ${rostest_LIBRARIES}
                ${roscpp_LIBRARIES}
                ${GTEST_LIBRARIES}
        )
        target_compile_options(${unit_test_NAME}
            PRIVATE
                ${unit_test_COMPILE_OPTIONS}
        )
    endif()
endfunction()