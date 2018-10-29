#ifndef LASER_POINTCLOUD_2D_PROVIDER_H
#define LASER_POINTCLOUD_2D_PROVIDER_H

#include "laser_provider.h"

namespace cslibs_plugins_data {
class LaserPointcloud2dProvider : public LaserProvider
{
public:
    LaserPointcloud2dProvider() = default;
    virtual ~LaserPointcloud2dProvider() = default;

protected:
    virtual void callback(const sensor_msgs::LaserScanConstPtr &msg) override;
    virtual void doSetup(ros::NodeHandle &nh) override;

    bool            transform_;
    std::string     transform_to_frame_;

};
}

#endif // LASER_POINTCLOUD_2D_PROVIDER_H
