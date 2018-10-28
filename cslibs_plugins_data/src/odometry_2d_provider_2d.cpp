#include "odometry_2d_provider_2d.h"

#include <cslibs_plugins_data/types/odometry_2d.hpp>
#include <tf/tf.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_plugins_data::Odometry2DProvider2D, cslibs_plugins_data::DataProvider2D)

namespace cslibs_plugins_data {
void Odometry2DProvider2D::callback(const nav_msgs::OdometryConstPtr &msg)
{
    auto to_pose = [](const nav_msgs::OdometryConstPtr &msg) {
        return cslibs_math_2d::Pose2d(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      tf::getYaw(msg->pose.pose.orientation));
    };

    types::Odometry2D::Ptr odometry;
    if (last_msg_) {
        cslibs_time::TimeFrame time_frame(last_msg_->header.stamp.toNSec(),
                                          msg->header.stamp.toNSec());
        odometry.reset(new types::Odometry2D(msg->header.frame_id,
                                             time_frame,
                                             to_pose(last_msg_),
                                             to_pose(msg),
                                             cslibs_time::Time(std::max(msg->header.stamp.toNSec(),
                                             ros::Time::now().toNSec()))));
    } else {
        cslibs_time::TimeFrame time_frame(msg->header.stamp.toNSec(),
                                          msg->header.stamp.toNSec());
        odometry.reset(new types::Odometry2D(msg->header.frame_id,
                                             time_frame,
                                             to_pose(msg),
                                             to_pose(msg),
                                             cslibs_time::Time(std::max(msg->header.stamp.toNSec(),
                                                               ros::Time::now().toNSec()))));
    }

    data_received_(odometry);
    last_msg_ = msg;
}

void Odometry2DProvider2D::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    const int queue_size = nh.param<int>(param_name("queue_size"), 1);
    topic_ = nh.param<std::string>(param_name("topic"), "/odom");
    source_= nh.subscribe(topic_, queue_size, &Odometry2DProvider2D::callback, this);
}
}
