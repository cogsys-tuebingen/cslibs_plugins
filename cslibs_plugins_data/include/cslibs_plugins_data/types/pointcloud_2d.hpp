#ifndef CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_2D_HPP
#define CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_2D_HPP

#include <cslibs_plugins_data/data.hpp>

#include <cslibs_math_2d/linear/pointcloud.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_2d.hpp>

namespace cslibs_plugins_data {
namespace types {
template <typename T>
class Pointcloud2 : public Data
{
public:
    using Ptr     = std::shared_ptr<Pointcloud2<T>>;
    using cloud_t = cslibs_math_2d::Pointcloud2<T>;

    Pointcloud2(const std::string &frame_id) :
        Data(frame_id)
    {
    }

    Pointcloud2(const std::string      &frame,
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

using Pointcloud2d = Pointcloud2<double>;
using Pointcloud2f = Pointcloud2<float>;
}
}

#endif // CSLIBS_PLUGINS_DATA_TYPES_POINTCLOUD_2D_HPP
