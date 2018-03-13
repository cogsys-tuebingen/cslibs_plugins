#ifndef CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_2D_H
#define CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_2D_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cslibs_plugins_data/data_provider_2d.hpp>

namespace cslibs_plugins_data {
class Odometry2DProvider2D : public DataProvider2D
{
public:
    Odometry2DProvider2D() = default;
    virtual ~Odometry2DProvider2D() = default;

protected:
    ros::Subscriber source_;
    std::string     topic_;

    nav_msgs::Odometry::ConstPtr last_msg_;

    void callback(const nav_msgs::OdometryConstPtr &msg);
    virtual void doSetup(ros::NodeHandle &nh) override;

};
}

#endif // CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_2D_H
