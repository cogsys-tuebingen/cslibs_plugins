#ifndef CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_H
#define CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cslibs_plugins_data/data_provider.hpp>
#include <cslibs_plugins_data/types/odometry_2d.hpp>
#include <tf/tf.h>

namespace cslibs_plugins_data {
template <typename T>
class Odometry2DProviderBase : public DataProvider
{
public:
    Odometry2DProviderBase() = default;
    virtual ~Odometry2DProviderBase() = default;

protected:
    ros::Subscriber source_;
    std::string     topic_;

    nav_msgs::Odometry::ConstPtr last_msg_;

    void callback(const nav_msgs::OdometryConstPtr &msg)
    {
        auto to_pose = [](const nav_msgs::OdometryConstPtr &msg) {
            return cslibs_math_2d::Pose2d<T>(msg->pose.pose.position.x,
                                             msg->pose.pose.position.y,
                                             tf::getYaw(msg->pose.pose.orientation));
        };

        typename types::Odometry2D<T>::Ptr odometry;
        if (last_msg_) {
            cslibs_time::TimeFrame time_frame(last_msg_->header.stamp.toNSec(),
                                              msg->header.stamp.toNSec());
            odometry.reset(new types::Odometry2D<T>(msg->header.frame_id,
                                                    time_frame,
                                                    to_pose(last_msg_),
                                                    to_pose(msg),
                                                    cslibs_time::Time(std::max(msg->header.stamp.toNSec(),
                                                                               ros::Time::now().toNSec()))));
        } else {
            cslibs_time::TimeFrame time_frame(msg->header.stamp.toNSec(),
                                              msg->header.stamp.toNSec());
            odometry.reset(new types::Odometry2D<T>(msg->header.frame_id,
                                                    time_frame,
                                                    to_pose(msg),
                                                    to_pose(msg),
                                                    cslibs_time::Time(std::max(msg->header.stamp.toNSec(),
                                                                               ros::Time::now().toNSec()))));
        }

        data_received_(odometry);
        last_msg_ = msg;
    }

    virtual void doSetup(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const int queue_size = nh.param<int>(param_name("queue_size"), 1);
        topic_ = nh.param<std::string>(param_name("topic"), "/odom");
        source_= nh.subscribe(topic_, queue_size, &Odometry2DProviderBase::callback, this);
    }
};

using Odometry2DProvider       = Odometry2DProviderBase<double>; // for backwards compatibility
using Odometry2DProviderDouble = Odometry2DProviderBase<double>;
using Odometry2DProviderFloat  = Odometry2DProviderBase<float>;
}

#endif // CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_2D_H
