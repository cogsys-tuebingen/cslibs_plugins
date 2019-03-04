#ifndef CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_2D_HPP
#define CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_2D_HPP

#include <cslibs_plugins_data/data.hpp>

#include <cslibs_math_2d/linear/pointcloud.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_2d.hpp>

namespace cslibs_plugins_data {
namespace types {
template <typename T>
class Pointcloud2d : public Data
{
public:
    using Ptr     = std::shared_ptr<Pointcloud2d<T>>;
    using cloud_t = cslibs_math_2d::Pointcloud2d<T>;

    Pointcloud2d(const std::string &frame_id) :
        Data(frame_id)
    {
    }

    Pointcloud2d(const std::string      &frame,
         const cslibs_time::TimeFrame   &time_frame,
         const cslibs_time::Time        &time_received) :
        Data(frame, time_frame, time_received)
    {
    }

    inline const typename cloud_t::ConstPtr points() const
    {
        return points_;
    }

    inline typename cloud_t::Ptr& points()
    {
        return points_;
    }

private:
    typename cloud_t::Ptr points_;
};
}
}

#endif // CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_2D_HPP
