#ifndef CSLIBS_PLUGINS_DATA_LASER_PROVIDER_H
#define CSLIBS_PLUGINS_DATA_LASER_PROVIDER_H

#include <sensor_msgs/LaserScan.h>

#include <cslibs_plugins_data/data_provider.hpp>
#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_plugins_data/types/laserscan_convert.hpp>
#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_plugins_data {
template <typename T>
class LaserProvider : public DataProvider
{
public:
    using point_t = cslibs_math_2d::Point2d<T>;

    LaserProvider() :
        time_offset_(0.0)
    {
    }
    virtual ~LaserProvider() = default;

protected:
    ros::Subscriber         source_;                    /// the subscriber to be used
    std::string             topic_;                     /// topic to listen to
    bool                    enforce_stamp_;             /// Enforce that start_time = stamp = end_time

    ros::Duration           time_offset_;
    ros::Time               time_of_last_measurement_;

    bool                    transform_;
    std::string             transform_to_frame_;

    std::array<float, 2>    range_limits_;

    virtual void callback(const sensor_msgs::LaserScanConstPtr &msg)
    {
        if (!time_offset_.isZero() && !time_of_last_measurement_.isZero())
            if (msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
                return;

        typename types::Laserscan<T>::Ptr laserscan;
        if (transform_ ? convert(msg, tf_, transform_to_frame_, tf_timeout_, range_limits_, laserscan, enforce_stamp_) :
                        convert(msg, range_limits_, laserscan, enforce_stamp_))
            data_received_(laserscan);

        time_of_last_measurement_ = msg->header.stamp;
    }

    virtual void doSetup(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        const int queue_size        = nh.param<int>(param_name("queue_size"), 1);

        topic_                      = nh.param<std::string>(param_name("topic"), "/scan");
        source_                     = nh.subscribe(topic_, queue_size, &LaserProvider::callback, this);

        enforce_stamp_              = nh.param<bool>(param_name("enforce_stamp"), true);

        transform_                  = nh.param<bool>(param_name("transform"), true);
        transform_to_frame_         = nh.param<std::string>(param_name("transform_to_frame"), "base_link");

        range_limits_               = {nh.param<double>(param_name("range_min"), 0.0),
                                       nh.param<double>(param_name("range_max"), std::numeric_limits<double>::max())};

        double rate                 = nh.param<double>(param_name("rate"), 0.0);
        if (rate > 0.0) {
            time_offset_ = ros::Duration(1.0 / rate);
            ROS_INFO_STREAM(name_ << ": Throttling laserscan to rate of " << rate << "Hz!");
        }
    }
};

using LaserProviderDouble = LaserProvider<double>;
using LaserProviderFloat  = LaserProvider<float>;
}

#endif // CSLIBS_PLUGINS_DATA_LASER_PROVIDER_H
