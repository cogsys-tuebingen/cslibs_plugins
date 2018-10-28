#ifndef CSLIBS_PLUGINS_DATA_TYPES_LASERSCAN_CONVERT_HPP
#define CSLIBS_PLUGINS_DATA_TYPES_LASERSCAN_CONVERT_HPP

#include <sensor_msgs/LaserScan.h>

#include <cslibs_plugins_data/types/laserscan.hpp>
#include <cslibs_math_ros/tf/tf_listener.hpp>

namespace cslibs_plugins_data {
namespace types {

using interval_t = std::array<double, 2>;

inline Laserscan::Ptr create(const sensor_msgs::LaserScanConstPtr &src,
                             const interval_t                     &linear_interval,
                             const interval_t                     &angular_interval,
                             const bool                            enforce_stamp = false)
{
    const ros::Time start_stamp = src->header.stamp;
    if (enforce_stamp) {
      const  Laserscan::time_frame_t time_frame(start_stamp.toNSec(), start_stamp.toNSec());
      return Laserscan::Ptr (new Laserscan(src->header.frame_id,
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

    const  Laserscan::time_frame_t time_frame(start_time, end_time);
    return Laserscan::Ptr (new Laserscan(src->header.frame_id,
                                         time_frame,
                                         linear_interval,
                                         angular_interval,
                                         cslibs_time::Time(std::max(end_time,
                                                                    ros::Time::now().toNSec()))));
}


inline bool convert(const sensor_msgs::LaserScanConstPtr &src,
                    Laserscan::Ptr                       &dst,
                    const bool                            enforce_stamp)
{
    const auto src_linear_min  = static_cast<double>(src->range_min);
    const auto src_linear_max  = static_cast<double>(src->range_max);
    const auto src_angular_min = static_cast<double>(src->angle_min);
    const auto src_angular_max = static_cast<double>(src->angle_max);
    const auto &src_ranges         = src->ranges;
    const auto src_angle_increment = src->angle_increment;

    if (src_ranges.size() == 0ul)
        return false;

    const interval_t dst_linear_interval  = { src_linear_min,  src_linear_max };
    const interval_t dst_angular_interval = { src_angular_min, src_angular_max };
    dst = create(src, dst_linear_interval, dst_angular_interval, enforce_stamp);

    auto in_linear_interval = [&dst_linear_interval](const double range) {
        return range >= dst_linear_interval[0] && range <= dst_linear_interval[1];
    };
    auto in_angular_interval = [&dst_angular_interval](const double angle) {
        return angle >= dst_angular_interval[0] && angle <= dst_angular_interval[1];
    };

    auto angle = src_angular_min;
    for (const auto range : src_ranges) {
        if(in_linear_interval(range) && in_angular_interval(angle))
            dst->insert(angle, range);
        else
            dst->insertInvalid();

        angle += src_angle_increment;
    }
    return true;
}

inline bool convertUndistorted(const sensor_msgs::LaserScanConstPtr     &src,
                               cslibs_math_ros::tf::TFProvider::Ptr     &tf_listener,
                               const std::string                        &fixed_frame,
                               const ros::Duration                      &tf_timeout,
                               Laserscan::Ptr                           &dst)
{
    const auto src_linear_min  = static_cast<double>(src->range_min);
    const auto src_linear_max  = static_cast<double>(src->range_max);
    const auto src_angular_min = static_cast<double>(src->angle_min);
    const auto src_angular_max = static_cast<double>(src->angle_max);
    const auto &src_ranges         = src->ranges;
    const auto src_angle_increment = src->angle_increment;

    if (src_ranges.size() == 0ul)
        return false;

    const interval_t dst_linear_interval  = { src_linear_min,  src_linear_max };
    const interval_t dst_angular_interval = { src_angular_min, src_angular_max };
    dst = create(src, dst_linear_interval, dst_angular_interval);

    auto in_linear_interval = [&dst_linear_interval](const double range) {
        return range >= dst_linear_interval[0] && range <= dst_linear_interval[1];
    };
    auto in_angular_interval = [&dst_angular_interval](const double angle) {
        return angle >= dst_angular_interval[0] && angle <= dst_angular_interval[1];
    };

    const ros::Time start_stamp = src->header.stamp;
    ros::Duration   delta_stamp = ros::Duration(src->time_increment);
    if (delta_stamp <= ros::Duration(0.0))
        delta_stamp = ros::Duration(src->scan_time / static_cast<float>(src_ranges.size()));

    const ros::Time end_stamp = start_stamp + delta_stamp * src_ranges.size();


    cslibs_math_2d::Transform2d cs_start_T_end;
    if (!tf_listener->lookupTransform(fixed_frame, dst->getFrame(), end_stamp, cs_start_T_end, tf_timeout))
        return false;
    tf::Transform start_T_end = cslibs_math_ros::tf::conversion_2d::from(cs_start_T_end);

    if (!tf_listener->waitForTransform(fixed_frame, dst->getFrame(), start_stamp, tf_timeout))
        return false;

    tf::Transform end_T_start = start_T_end.inverse();

    auto angle = src_angular_min;
    auto stamp = start_stamp;
    for (const auto range : src_ranges) {
        if (in_linear_interval(range) && in_angular_interval(angle)) {
            cslibs_math_2d::Transform2d cs_start_T_stamp;
            tf_listener->lookupTransform(fixed_frame, dst->getFrame(), stamp, cs_start_T_stamp);
            tf::Transform end_T_stamp = end_T_start * cslibs_math_ros::tf::conversion_2d::from(cs_start_T_stamp);
            tf::Point pt = end_T_stamp * tf::Point(std::cos(angle) * range, std::sin(angle) * range, 0.0);
            dst->insert(cslibs_math_2d::Point2d(pt.x(), pt.y()));
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
