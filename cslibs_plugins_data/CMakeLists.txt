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
    CATKIN_DEPENDS cslibs_plugins cslibs_time cslibs_utility cslibs_math_ros cslibs_math_2d cslibs_math_3d
                   pluginlib roscpp rostest sensor_msgs nav_msgs tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

cslibs_plugins_data_show_headers()

add_library(${PROJECT_NAME}
    src/laser_provider.cpp
    src/laser_pointcloud_2d_provider.cpp
    src/laser_polar_pointcloud_2d_provider.cpp
    src/odometry_2d_provider.cpp
    src/odometry_2d_provider_tf.cpp
    src/pointcloud_3d_provider.cpp
    )
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
