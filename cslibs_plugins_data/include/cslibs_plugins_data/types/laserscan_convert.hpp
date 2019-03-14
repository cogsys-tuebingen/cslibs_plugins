#ifndef CSLIBS_PLUGINS_DATA_TYPES_LASERSCAN_CONVERT_HPP
#define CSLIBS_PLUGINS_DATA_TYPES_LASERSCAN_CONVERT_HPP

#include <sensor_msgs/LaserScan.h>

#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_math_ros/tf/tf_listener.hpp>

namespace cslibs_plugins_data {
namespace types {
template <typename T>
using interval_t = std::array<T, 2>;

template <typename T>
inline typename Laserscan2<T>::Ptr create(const sensor_msgs::LaserScanConstPtr &src,
                                         const std::string                     &frame_id,
                                         const interval_t<T>                   &linear_interval,
                                         const interval_t<T>                   &angular_interval,
                                         const bool                            enforce_stamp = false)
{
    const ros::Time start_stamp = src->header.stamp;
    if (enforce_stamp) {
        const  typename Laserscan2<T>::time_frame_t time_frame(start_stamp.toNSec(), start_stamp.toNSec());
        return typename Laserscan2<T>::Ptr (new Laserscan2<T>(frame_id,
                                                              time_frame,
                                                              linear_interval,
                                                              angular_interval,
                                                              cslibs_time::Time(std::max(start_stamp.toNSec(),
                                                                                       ros::Time::now().toNSec()))));
    }

    ros::Duration delta_stamp = ros::Duration(src->time_increment) * static_cast<double>(src->ranges.size());
    if (delta_stamp <= ros::Duration(0.0))
        delta_stamp = ros::Duration(src->scan_time);

    const uint64_t start_time = start_stamp.toNSec();
    const uint64_t end_time   = start_time + delta_stamp.toNSec();

    const  typename Laserscan2<T>::time_frame_t time_frame(start_time, end_time);
    return typename Laserscan2<T>::Ptr (new Laserscan2<T>(frame_id,
                                                          time_frame,
                                                          linear_interval,
                                                          angular_interval,
                                                          cslibs_time::Time(std::max(end_time,
                                                                                     ros::Time::now().toNSec()))));
}

template <typename T>
inline bool convert(const sensor_msgs::LaserScanConstPtr &src,
                    const interval_t<T>                  &range_limits,
                    typename Laserscan2<T>::Ptr          &dst,
                    const bool                            enforce_stamp)
{
    const auto src_linear_min  = std::max(static_cast<T>(src->range_min), range_limits[0]);
    const auto src_linear_max  = std::min(static_cast<T>(src->range_max), range_limits[1]);
    const auto src_angular_min = static_cast<T>(src->angle_min);
    const auto src_angular_max = static_cast<T>(src->angle_max);
    const auto &src_ranges         = src->ranges;
    const auto src_angle_increment = src->angle_increment;

    if (src_ranges.size() == 0ul)
        return false;

    const interval_t<T> dst_linear_interval  = { src_linear_min,  src_linear_max };
    const interval_t<T> dst_angular_interval = { src_angular_min, src_angular_max };
    dst = create<T>(src, src->header.frame_id, dst_linear_interval, dst_angular_interval, enforce_stamp);

    auto in_linear_interval = [&dst_linear_interval](const T range) {
        return range > dst_linear_interval[0] && range < dst_linear_interval[1];
    };
    auto in_angular_interval = [&dst_angular_interval](const T angle) {
        return angle >= dst_angular_interval[0] && angle <= dst_angular_interval[1];
    };

    auto angle = src_angular_min;
    for (const auto range : src_ranges) {
        if(in_linear_interval(range) && in_angular_interval(angle))
            dst->insert(static_cast<T>(angle), static_cast<T>(range));
        else
            dst->insertInvalid();

        angle += src_angle_increment;
    }
    return true;
}

template <typename T>
inline bool convert(const sensor_msgs::LaserScanConstPtr  &src,
                    cslibs_math_ros::tf::TFProvider::Ptr  &tf_listener,
                    const std::string                     &tf_target_frame,
                    const ros::Duration                   &tf_timeout,
                    const interval_t<T>                   &range_limits,
                    typename Laserscan2<T>::Ptr            &dst,
                    const bool                             enforce_stamp)
{
    const auto src_linear_min  = std::max(static_cast<T>(src->range_min), range_limits[0]);
    const auto src_linear_max  = std::min(static_cast<T>(src->range_max), range_limits[1]);
    const auto src_angular_min = static_cast<T>(src->angle_min);
    const auto src_angular_max = static_cast<T>(src->angle_max);
    const auto &src_ranges         = src->ranges;
    const auto src_angle_increment = src->angle_increment;

    if (src_ranges.size() == 0ul)
        return false;

    const interval_t<T> dst_linear_interval  = { src_linear_min,  src_linear_max };
    const interval_t<T> dst_angular_interval = { src_angular_min, src_angular_max };
    dst = create(src, tf_target_frame, dst_linear_interval, dst_angular_interval, enforce_stamp);

    auto in_linear_interval = [&dst_linear_interval](const T range) {
        return range > dst_linear_interval[0] && range < dst_linear_interval[1];
    };
    auto in_angular_interval = [&dst_angular_interval](const T angle) {
        return angle >= dst_angular_interval[0] && angle <= dst_angular_interval[1];
    };

    cslibs_math_3d::Transform3<T> t_T_l;
    if(tf_listener->lookupTransform(tf_target_frame, src->header.frame_id, src->header.stamp, t_T_l, tf_timeout)) {
        const cslibs_math_2d::Point2<T> start_point(t_T_l.tx(), t_T_l.ty());
        auto angle = src_angular_min;
        for (const auto range : src_ranges) {
            if(in_linear_interval(range) && in_angular_interval(angle)) {
                const cslibs_math_3d::Point3<T> p =
                        t_T_l * cslibs_math_3d::Point3<T>(std::cos(static_cast<T>(angle)) * static_cast<T>(range),
                                                          std::sin(static_cast<T>(angle)) * static_cast<T>(range),
                                                          T());
                const cslibs_math_2d::Point2<T> end_point(p(0), p(1));

                const T transformed_angle = cslibs_math_2d::angle(end_point - start_point);
                const T range = cslibs_math::linear::distance(start_point, end_point);

                dst->insert(transformed_angle, range, end_point, start_point);
            } else {
                dst->insertInvalid();
            }

            angle += src_angle_increment;
        }
        return true;
    }
    return false;
}

template <typename T>
inline bool convertUndistorted(const sensor_msgs::LaserScanConstPtr  &src,
                               cslibs_math_ros::tf::TFProvider::Ptr  &tf_listener,
                               const std::string                     &fixed_frame,
                               const ros::Duration                   &tf_timeout,
                               typename Laserscan2<T>::Ptr            &dst)
{
    const auto src_linear_min  = static_cast<T>(src->range_min);
    const auto src_linear_max  = static_cast<T>(src->range_max);
    const auto src_angular_min = static_cast<T>(src->angle_min);
    const auto src_angular_max = static_cast<T>(src->angle_max);
    const auto &src_ranges         = src->ranges;
    const auto src_angle_increment = src->angle_increment;

    if (src_ranges.size() == 0ul)
        return false;

    const interval_t<T> dst_linear_interval  = { src_linear_min,  src_linear_max };
    const interval_t<T> dst_angular_interval = { src_angular_min, src_angular_max };
    dst = create(src, src->header.frame_id, dst_linear_interval, dst_angular_interval);

    auto in_linear_interval = [&dst_linear_interval](const T range) {
        return range > dst_linear_interval[0] && range < dst_linear_interval[1];
    };
    auto in_angular_interval = [&dst_angular_interval](const T angle) {
        return angle >= dst_angular_interval[0] && angle <= dst_angular_interval[1];
    };

    const ros::Time start_stamp = src->header.stamp;
    ros::Duration   delta_stamp = ros::Duration(src->time_increment);
    if (delta_stamp <= ros::Duration(0.0))
        delta_stamp = ros::Duration(static_cast<double>(src->scan_time) / static_cast<double>(src_ranges.size()));

    const ros::Time end_stamp = start_stamp + delta_stamp * src_ranges.size();


    cslibs_math_2d::Transform2<T> cs_start_T_end;
    if (!tf_listener->lookupTransform(fixed_frame, dst->frame(), end_stamp, cs_start_T_end, tf_timeout))
        return false;
    tf::Transform start_T_end = cslibs_math_ros::tf::conversion_2d::from(cs_start_T_end);

    if (!tf_listener->waitForTransform(fixed_frame, dst->frame(), start_stamp, tf_timeout))
        return false;

    tf::Transform end_T_start = start_T_end.inverse();

    auto angle = src_angular_min;
    auto stamp = start_stamp;
    for (const auto range : src_ranges) {
        if (in_linear_interval(range) && in_angular_interval(angle)) {
            cslibs_math_2d::Transform2<T> cs_start_T_stamp;
            tf_listener->lookupTransform(fixed_frame, dst->frame(), stamp, cs_start_T_stamp);
            tf::Transform end_T_stamp = end_T_start * cslibs_math_ros::tf::conversion_2d::from(cs_start_T_stamp);
            tf::Point pt = end_T_stamp * tf::Point(std::cos(angle) * range, std::sin(angle) * range, 0.0);
            dst->insert(cslibs_math_2d::Point2<T>(pt.x(), pt.y()));
        } else
            dst->insertInvalid();

        angle += src_angle_increment;
        stamp += delta_stamp;
    }
    return true;
}
}
}

#endif // CSLIBS_PLUGINS_DATA_TYPES_LASERSCAN_CONVERT_HPP
