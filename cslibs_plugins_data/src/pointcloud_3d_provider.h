#ifndef CSLIBS_PLUGINS_DATA_POINTCLOUD_PROVIDER_2D_H
#define CSLIBS_PLUGINS_DATA_POINTCLOUD_PROVIDER_2D_H

#include <sensor_msgs/PointCloud2.h>

#include <cslibs_plugins_data/data_provider.hpp>
#include <cslibs_plugins_data/types/pointcloud_3d.hpp>

namespace cslibs_plugins_data {
template <typename T>
class Pointcloud3dProviderBase : public DataProvider
{
public:
    Pointcloud3dProviderBase() :
        time_offset_(0.0)
    {
    }
    virtual ~Pointcloud3dProviderBase() = default;

protected:
    ros::Subscriber source_;                    /// the subscriber to be used
    std::string     topic_;                     /// topic to listen to

    ros::Duration   time_offset_;
    ros::Time       time_of_last_measurement_;

    void callback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        if (!time_offset_.isZero() && !time_of_last_measurement_.isZero())
            if (msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
                return;

        typename types::Pointcloud3d<T>::Ptr pointcloud(new types::Pointcloud3d<T>(msg->header.frame_id,
                                                                                   cslibs_math_ros::sensor_msgs::conversion_3d::from(msg),
                                                                                   cslibs_time::Time(std::max(msg->header.stamp.toNSec(), ros::Time::now().toNSec()))));

        cslibs_math_ros::sensor_msgs::conversion_3d::from(msg, pointcloud->points());
        data_received_(pointcloud);

        time_of_last_measurement_ = msg->header.stamp;
    }
    virtual void doSetup(ros::NodeHandle &nh) override
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
};

using Pointcloud3dProvider       = Pointcloud3dProviderBase<double>; // for backwards compatibility
using Pointcloud3dProviderDouble = Pointcloud3dProviderBase<double>;
using Pointcloud3dProviderFloat  = Pointcloud3dProviderBase<float>;
}

#endif // CSLIBS_PLUGINS_DATA_POINTCLOUD_PROVIDER_2D_H
