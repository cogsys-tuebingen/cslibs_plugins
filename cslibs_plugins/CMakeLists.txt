cmake_minimum_required(VERSION 2.8.3)
project(cslibs_plugins)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(cmake/cslibs_plugins_enable_c++11.cmake)
include(cmake/cslibs_plugins_extras.cmake)
include(cmake/cslibs_plugins_openmp.cmake)
include(cmake/cslibs_plugins_show_headers.cmake)
include(cmake/cslibs_plugins_add_unit_test_gtest.cmake)

find_package(catkin REQUIRED)
catkin_package(
    INCLUDE_DIRS   include
    CFG_EXTRAS     cslibs_plugins_extras.cmake
                   cslibs_plugins_enable_c++11.cmake
                   cslibs_plugins_openmp.cmake
                   cslibs_plugins_show_headers.cmake
                   cslibs_plugins_add_unit_test_gtest.cmake
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

cslibs_plugins_show_headers()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
