#include "laser_polar_pointcloud_2d_provider.h"

#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_plugins_data/types/laserscan_convert.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_plugins_data::LaserPolarPointcloud2dProvider, cslibs_plugins_data::DataProvider)

namespace cslibs_plugins_data {
void LaserPolarPointcloud2dProvider::callback(const sensor_msgs::LaserScanConstPtr &msg)
{
//    if (!time_offset_.isZero() && !time_of_last_measurement_.isZero())
//        if (msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
//            return;

//    types::Laserscan::Ptr laserscan;
//    if (convert(msg, laserscan, enforce_stamp_))
//        data_received_(laserscan);

//    time_of_last_measurement_ = msg->header.stamp;
}
}
