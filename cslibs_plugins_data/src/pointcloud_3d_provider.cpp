#include "pointcloud_3d_provider.h"

#include <cslibs_plugins_data/types/pointcloud_3d.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_plugins_data::Pointcloud3dProvider, cslibs_plugins_data::DataProvider)

namespace cslibs_plugins_data {
Pointcloud3dProvider::Pointcloud3dProvider() :
    time_offset_(0.0)
{
}

void Pointcloud3dProvider::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!time_offset_.isZero() && !time_of_last_measurement_.isZero())
        if (msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
            return;

    types::Pointcloud3d::Ptr pointcloud(new types::Pointcloud3d(msg->header.frame_id,
                                                                cslibs_math_ros::sensor_msgs::conversion_3d::from(msg),
                                                                cslibs_time::Time(std::max(msg->header.stamp.toNSec(), ros::Time::now().toNSec()))));

    cslibs_math_ros::sensor_msgs::conversion_3d::from(msg, pointcloud->points());

    data_received_(pointcloud);

    time_of_last_measurement_ = msg->header.stamp;
}

void Pointcloud3dProvider::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    int queue_size  = nh.param<int>(param_name("queue_size"), 1);
    topic_          = nh.param<std::string>(param_name("topic"), "");
    source_         = nh.subscribe(topic_, queue_size, &Pointcloud3dProvider::callback, this);

    double rate     = nh.param<double>(param_name("rate"), 0.0);
    if (rate > 0.0) {
        time_offset_ = ros::Duration(1.0 / rate);
        ROS_INFO_STREAM(name_ << ": Throttling pointcloud to rate of " << rate << "Hz!");
    }
}
}
