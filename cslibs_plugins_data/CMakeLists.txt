cmake_minimum_required(VERSION 2.8.3)
project(cslibs_plugins_data)

find_package(catkin REQUIRED COMPONENTS
    cslibs_plugins
    cslibs_time
    cslibs_utility
    cslibs_math_ros
    cslibs_math_2d
    cslibs_math_3d
    pluginlib
    roscpp
    rostest
    sensor_msgs
    nav_msgs
    tf
)

catkin_package(
    INCLUDE_DIRS   include
    LIBRARIES      ${PROJECT_NAME}
    CATKIN_DEPENDS
        cslibs_plugins
        cslibs_time
        cslibs_utility
        cslibs_math_ros
        cslibs_math_2d
        cslibs_math_3d
        pluginlib
        roscpp
        rostest
        sensor_msgs
        nav_msgs tf
)
if(NOT ${CMAKE_BUILD_TYPE} STREQUAL Debug)
    set(TARGET_COMPILE_DEFINITIONS
        -Wall -WError -Ofast -ffastmath
    )
    # enabling -march=native my result in problems on some machines,
    # my try -mtune=native

    message(STATUS "[${PROJECT_NAME}]: Compiling with optimization!")
endif()

set(TARGET_INCLUDE_DIRS
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/laser_provider.cpp
    src/odometry_2d_provider.cpp
    src/odometry_2d_provider_tf.cpp
    src/pointcloud_3d_provider.cpp
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        ${TARGET_COMPILE_DEFINITIONS}
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${catkin_LIBRARIES}
)

cslibs_plugins_data_add_unit_test_ros(integration
    INCLUDE_DIRS
        ${TARGET_INCLUDE_DIRS}
    LAUNCH_FILE
        test/integration.test
    SOURCE_FILES
        test/integration.cpp
    LINK_LIBRARIES
        ${catkin_LIBRARIES}
    COMPILE_DEFINITIONS
        ${TARGET_COMPILE_DEFINITIONS}
)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
