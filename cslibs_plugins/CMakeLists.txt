cmake_minimum_required(VERSION 2.8.3)
project(cslibs_plugins)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(cmake/cslibs_plugins_enable_c++17.cmake)

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

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
