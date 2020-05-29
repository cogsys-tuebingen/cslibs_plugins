cmake_minimum_required(VERSION 2.8.3)
project(cslibs_plugins)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(cmake/cslibs_plugins_enable_c++17.cmake)
include(cmake/cslibs_plugins_add_unit_test_ros.cmake)

find_package(catkin REQUIRED)
find_package(TinyXML REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    CFG_EXTRAS
        cslibs_plugins_enable_c++17.cmake
    DEPENDS
        TinyXML
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
    INTERFACE
        include/
        ${catkin_INCLUDE_DIRS}
)

cslibs_plugins_add_unit_test_ros(test_launch_file_parser
    INCLUDE_DIRS
        include/
    LAUNCH_FILE
        test/launch_file_parser.test
    SOURCE_FILES
        test/launch_file_parser.cpp
    LINK_LIBRARIES
        ${catkin_LIBRARIES}
    COMPILE_OPTIONS
        ${TARGET_COMPILE_OPTIONS}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
