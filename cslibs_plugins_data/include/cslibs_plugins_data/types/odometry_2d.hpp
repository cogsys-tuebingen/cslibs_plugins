#ifndef CSLIBS_PLUGINS_DATA_TYPES_ODOMETRY_2D_HPP
#define CSLIBS_PLUGINS_DATA_TYPES_ODOMETRY_2D_HPP

#include <cslibs_plugins_data/data.hpp>

#include <cslibs_math_2d/linear/pose.hpp>
#include <cslibs_math_2d/linear/transform.hpp>

namespace cslibs_plugins_data {
namespace types {
class Odometry2D : public Data {
public:
    using Ptr          = std::shared_ptr<Odometry2D>;
    using ConstPtr     = std::shared_ptr<const Odometry2D>;
    using time_frame_t = cslibs_time::TimeFrame;
    using time_t       = cslibs_time::Time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Odometry2D>;

    Odometry2D(const std::string &frame) :
      Data(frame),
      start_pose_(cslibs_math_2d::Transform2d::identity()),
      end_pose_(cslibs_math_2d::Transform2d::identity()),
      delta_linear_(0.0),
      delta_angular_(0.0),
      forward_(true)
    {
    }

    Odometry2D(const std::string  &frame,
               const time_frame_t &time_frame,
               const time_t       &time_received) :
      Data(frame, time_frame, time_received),
      start_pose_(cslibs_math_2d::Transform2d::identity()),
      end_pose_(cslibs_math_2d::Transform2d::identity()),
      delta_linear_(0.0),
      delta_angular_(0.0),
      forward_(true)
    {
    }

    Odometry2D(const std::string            &frame,
               const time_frame_t           &time_frame,
               const cslibs_math_2d::Pose2d &start,
               const cslibs_math_2d::Pose2d &end,
               const time_t                 &time_received) :
      Data(frame, time_frame, time_received),
      start_pose_(start),
      end_pose_(end),
      delta_lin_abs_(end.translation() - start.translation()),
      delta_linear_(delta_lin_abs_.length()),
      delta_angular_(cslibs_math::common::angle::difference(end.yaw(), start.yaw())),
      forward_((start.inverse() * end).tx() >= 0.0)
    {
    }

    inline double getDeltaAngularAbs() const
    {
        return std::atan2(delta_lin_abs_(1),
                          delta_lin_abs_(0));
    }

    const inline cslibs_math_2d::Pose2d& getStartPose() const
    {
        return start_pose_;
    }

    const inline cslibs_math_2d::Pose2d& getEndPose() const
    {
        return end_pose_;
    }

    const inline cslibs_math_2d::Vector2d& getDelta() const
    {
        return delta_lin_abs_;
    }

    inline double getDeltaLinear() const
    {
        return delta_linear_;
    }

    inline double getDeltaAngular() const
    {
        return delta_angular_;
    }

    inline bool forward() const
    {
        return forward_;
    }

    inline Odometry2D::ConstPtr cutFront(const time_t &split_time) const
    {
        Odometry2D::ConstPtr a;
        Odometry2D::ConstPtr b;
        split(split_time, a, b);
        return b;
    }

    inline bool split(const time_t         &split_time,
                      Odometry2D::ConstPtr &a,
                      Odometry2D::ConstPtr &b) const
    {
        auto do_not_split = []() {
            return false;
        };

        auto do_split = [&split_time, &a, &b, this] () {
            const double ratio = (split_time - time_frame_.start).nanoseconds() / time_frame_.duration().nanoseconds();

            cslibs_math_2d::Pose2d split_pose = start_pose_.interpolate(end_pose_, ratio);
            a.reset(new Odometry2D(frame_, time_frame_t(time_frame_.start, split_time), start_pose_, split_pose, time_received_));
            b.reset(new Odometry2D(frame_, time_frame_t(split_time, time_frame_.end), split_pose, end_pose_, time_received_ + (split_time - time_frame_.start)));

            return true;
        };

        return time_frame_.within(split_time) ? do_split() : do_not_split();
    }

private:
    cslibs_math_2d::Pose2d    start_pose_;
    cslibs_math_2d::Pose2d    end_pose_;
    cslibs_math_2d::Vector2d  delta_lin_abs_;
    double                    delta_linear_;
    double                    delta_angular_;
    bool                      forward_;

}__attribute__ ((aligned (16)));
}

inline std::ostream & operator << (std::ostream &out, const cslibs_plugins_data::types::Odometry2D &odom)
{
    out << "[Odometry2D]: linear  " << odom.getDeltaLinear()  << "\n";
    out << "              angular " << odom.getDeltaAngular() << "\n";
    out << "              delta   " << odom.getDelta()(0) << " " << odom.getDelta()(1);
    return out;
}
}

#endif // CSLIBS_PLUGINS_DATA_TYPES_ODOMETRY_2D_HPP
