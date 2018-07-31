#ifndef CSLIBS_PLUGINS_DATA_LASER_PROVIDER_2D_H
#define CSLIBS_PLUGINS_DATA_LASER_PROVIDER_2D_H

#include <sensor_msgs/LaserScan.h>

#include <cslibs_plugins_data/data_provider_2d.hpp>
#include <cslibs_math_2d/linear/point.hpp>

namespace cslibs_plugins_data {
class LaserProvider2D : public DataProvider2D
{
public:
    using point_t = cslibs_math_2d::Point2d;
    using interval_t = std::array<double, 2>;

    LaserProvider2D();
    virtual ~LaserProvider2D() = default;

protected:
    ros::Subscriber source_;                    /// the subscriber to be used
    std::string     topic_;                     /// topic to listen to

    bool            enforce_stamp_;             /// Enforce that start_time = stamp = end_time
    bool            undistortion_;              /// check if undistortion shall be applied
    std::string     undistortion_fixed_frame_;  /// the fixed frame necessary for the undistortion
    ros::Duration   undistortion_tf_timeout_;   /// time out for the tf listener

    ros::Duration   time_offset_;
    ros::Time       time_of_last_measurement_;

    void callback(const sensor_msgs::LaserScanConstPtr &msg);
    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif // CSLIBS_PLUGINS_DATA_LASER_PROVIDER_2D_H
