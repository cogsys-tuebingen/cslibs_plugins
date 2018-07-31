#include "laser_provider_2d.h"

#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_plugins_data/types/laserscan_convert.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_plugins_data::LaserProvider2D, cslibs_plugins_data::DataProvider2D)

namespace cslibs_plugins_data {
LaserProvider2D::LaserProvider2D() :
    time_offset_(0.0)
{
}

void LaserProvider2D::callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    if (!time_offset_.isZero() && !time_of_last_measurement_.isZero())
        if (msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
            return;

    types::Laserscan::Ptr laserscan;
    if ((undistortion_ &&
         convertUndistorted(msg, tf_, undistortion_fixed_frame_, undistortion_tf_timeout_, laserscan)) ||
            convert(msg, laserscan, enforce_stamp_)) {
        data_received_(laserscan);
    }

    time_of_last_measurement_ = msg->header.stamp;
}

void LaserProvider2D::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    int queue_size              = nh.param<int>(param_name("queue_size"), 1);
    topic_                      = nh.param<std::string>(param_name("topic"), "/scan");
    source_                     = nh.subscribe(topic_, queue_size, &LaserProvider2D::callback, this);

    enforce_stamp_              = nh.param<bool>(param_name("enforce_stamp"), true);
    undistortion_               = nh.param<bool>(param_name("undistortion"), false);
    undistortion_fixed_frame_   = nh.param<std::string>(param_name("undistortion_fixed_frame"), "");
    undistortion_tf_timeout_    = ros::Duration(nh.param(param_name("undistotion_tf_timeout"), 0.1));

    double rate                 = nh.param<double>(param_name("rate"), 0.0);
    if (rate > 0.0) {
        time_offset_ = ros::Duration(1.0 / rate);
        ROS_INFO_STREAM(name_ << ": Throttling laserscan to rate of " << rate << "Hz!");
    }
}
}
