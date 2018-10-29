#include "laser_pointcloud_2d_provider.h"

#include <cslibs_plugins_data/types/pointcloud_2d.hpp>
#include <cslibs_plugins_data/types/laserscan_convert.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_plugins_data::LaserPointcloud2dProvider, cslibs_plugins_data::DataProvider)

namespace cslibs_plugins_data {
void LaserPointcloud2dProvider::callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    if (!time_offset_.isZero() && !time_of_last_measurement_.isZero())
        if (msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
            return;
}

void LaserPointcloud2dProvider::doSetup(ros::NodeHandle &nh)
{
    LaserProvider::doSetup(nh);

    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    const std::string poincloud_type    = nh.param<std::string>(param_name("pointcloud_type"), "2D");
    transform_                          = nh.param<bool>(param_name("transform"), true);
    transform_to_frame_                 = nh.param<std::string>(param_name("transform_to_frame"), "/base_link");

}
}
