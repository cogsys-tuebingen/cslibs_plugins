#ifndef CSLIBS_PLUGINS_DATA_POINTCLOUD_PROVIDER_2D_H
#define CSLIBS_PLUGINS_DATA_POINTCLOUD_PROVIDER_2D_H

#include <sensor_msgs/PointCloud2.h>

#include <cslibs_plugins_data/data_provider.hpp>

namespace cslibs_plugins_data {
class Pointcloud3dProvider : public DataProvider
{
public:
    Pointcloud3dProvider();
    virtual ~Pointcloud3dProvider() = default;

protected:
    ros::Subscriber source_;                    /// the subscriber to be used
    std::string     topic_;                     /// topic to listen to

    ros::Duration   time_offset_;
    ros::Time       time_of_last_measurement_;

    void callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif // CSLIBS_PLUGINS_DATA_POINTCLOUD_PROVIDER_2D_H
