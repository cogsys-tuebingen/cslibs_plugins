#ifndef LASER_POLAR_POINTCLOUD_2D_PROVIDER_H
#define LASER_POLAR_POINTCLOUD_2D_PROVIDER_H

#include "laser_pointcloud_2d_provider.h"

namespace cslibs_plugins_data {
class LaserPolarPointcloud2dProvider : public LaserPointcloud2dProvider
{
public:
    LaserPolarPointcloud2dProvider() = default;
    virtual ~LaserPolarPointcloud2dProvider() = default;
protected:
     virtual void callback(const sensor_msgs::LaserScanConstPtr &msg) override;
};
}


#endif // LASER_POLAR_POINTCLOUD_2D_PROVIDER_H
