#ifndef CSLIBS_PLUGINS_DATA_LASER_PROVIDER_H
#define CSLIBS_PLUGINS_DATA_LASER_PROVIDER_H

#include <sensor_msgs/LaserScan.h>

#include <cslibs_plugins_data/data_provider.hpp>
#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_plugins_data {

class LaserProvider : public DataProvider
{
public:
    using point_t = cslibs_math_2d::Point2d;
    using interval_t = std::array<double, 2>;

    LaserProvider();
    virtual ~LaserProvider() = default;

protected:
    ros::Subscriber source_;                    /// the subscriber to be used
    std::string     topic_;                     /// topic to listen to
    bool            enforce_stamp_;             /// Enforce that start_time = stamp = end_time

    double          range_min_;                 /// Set range min manually
    double          range_max_;                 /// Set range max manually

    ros::Duration   time_offset_;
    ros::Time       time_of_last_measurement_;

    bool            transform_;
    std::string     transform_to_frame_;

    virtual void callback(const sensor_msgs::LaserScanConstPtr &msg);
    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif // CSLIBS_PLUGINS_DATA_LASER_PROVIDER_H
