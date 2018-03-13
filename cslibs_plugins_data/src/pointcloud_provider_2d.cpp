#include "pointcloud_provider_2d.h"

#include <cslibs_plugins_data/types/pointcloud.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_plugins_data::PointcloudProvider2D, cslibs_plugins_data::DataProvider2D)

namespace cslibs_plugins_data {
PointcloudProvider2D::PointcloudProvider2D() :
    time_offset_(0.0)
{
}

void PointcloudProvider2D::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!time_offset_.isZero() && !time_of_last_measurement_.isZero())
        if (msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
            return;

    types::Pointcloud::Ptr pointcloud(new types::Pointcloud(msg));
    data_received_(pointcloud);

    time_of_last_measurement_ = msg->header.stamp;
}

void PointcloudProvider2D::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    int queue_size  = nh.param<int>(param_name("queue_size"), 1);
    topic_          = nh.param<std::string>(param_name("topic"), "");
    source_         = nh.subscribe(topic_, queue_size, &PointcloudProvider2D::callback, this);

    double rate     = nh.param<double>(param_name("rate"), 0.0);
    if (rate > 0.0) {
        time_offset_ = ros::Duration(1.0 / rate);
        ROS_INFO_STREAM(name_ << ": Throttling pointcloud to rate of " << rate << "Hz!");
    }
}
}
