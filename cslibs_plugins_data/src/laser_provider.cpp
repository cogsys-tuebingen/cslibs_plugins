#include "laser_provider.h"

#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_plugins_data/types/laserscan_convert.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_plugins_data::LaserProvider, cslibs_plugins_data::DataProvider)

namespace cslibs_plugins_data {
LaserProvider::LaserProvider() :
    time_offset_(0.0)
{
}

void LaserProvider::callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    if (!time_offset_.isZero() && !time_of_last_measurement_.isZero())
        if (msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
            return;

    types::Laserscan::Ptr laserscan;
    if(transform_ ? convert(msg, tf_, transform_to_frame_, tf_timeout_, range_limits_, laserscan, enforce_stamp_) :
                    convert(msg, range_limits_, laserscan, enforce_stamp_))
        data_received_(laserscan);

    time_of_last_measurement_ = msg->header.stamp;
}

void LaserProvider::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    const int queue_size        = nh.param<int>(param_name("queue_size"), 1);

    topic_                      = nh.param<std::string>(param_name("topic"), "/scan");
    source_                     = nh.subscribe(topic_, queue_size, &LaserProvider::callback, this);

    enforce_stamp_              = nh.param<bool>(param_name("enforce_stamp"), true);

    transform_                  = nh.param<bool>(param_name("transform"), true);
    transform_to_frame_         = nh.param<std::string>(param_name("transform_to_frame"), "base_link");

    range_limits_               = {nh.param<double>(param_name("range_min"), std::numeric_limits<double>::max()), 
                                   nh.param<double>(param_name("range_max"), 0.0)};

    double rate                 = nh.param<double>(param_name("rate"), 0.0);
    if (rate > 0.0) {
        time_offset_ = ros::Duration(1.0 / rate);
        ROS_INFO_STREAM(name_ << ": Throttling laserscan to rate of " << rate << "Hz!");
    }
}
}
