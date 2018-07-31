#ifndef CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_HPP
#define CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_HPP

#include <cslibs_plugins_data/data.hpp>

#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_3d.hpp>

namespace cslibs_plugins_data {
namespace types {
class Pointcloud : public Data
{
public:
    using Ptr     = std::shared_ptr<Pointcloud>;
    using cloud_t = cslibs_math_3d::Pointcloud3d;

    Pointcloud(const sensor_msgs::PointCloud2ConstPtr &msg) :
        Data(msg->header.frame_id,
             cslibs_math_ros::sensor_msgs::conversion_3d::from(msg),
             cslibs_time::Time(std::max(msg->header.stamp.toNSec(), ros::Time::now().toNSec())))
    {
        cslibs_math_ros::sensor_msgs::conversion_3d::from(msg, points_);
    }

    inline const typename cloud_t::Ptr& getPoints() const
    {
        return points_;
    }

private:
    typename cloud_t::Ptr points_;
};
}
}

#endif // CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_HPP
