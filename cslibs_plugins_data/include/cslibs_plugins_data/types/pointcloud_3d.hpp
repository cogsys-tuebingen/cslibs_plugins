#ifndef CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_3D_HPP
#define CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_3D_HPP

#include <cslibs_plugins_data/data.hpp>

#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_3d.hpp>

namespace cslibs_plugins_data {
namespace types {
template <typename T>
class Pointcloud3 : public Data
{
public:
    using Ptr     = std::shared_ptr<Pointcloud3<T>>;
    using cloud_t = cslibs_math_3d::Pointcloud3<T>;

    Pointcloud3(const std::string &frame_id) :
        Data(frame_id)
    {
    }

    Pointcloud3(const std::string      &frame,
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
using Pointcloud3d = Pointcloud3<double>;
using Pointcloud3f = Pointcloud3<float>;
}
}

#endif // CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_3D_HPP
