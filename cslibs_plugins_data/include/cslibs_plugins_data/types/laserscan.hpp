#ifndef CSLIBS_PLUGINS_DATA_TYPES_LASERSCAN_HPP
#define CSLIBS_PLUGINS_DATA_TYPES_LASERSCAN_HPP

#include <cslibs_plugins_data/data.hpp>

#include <cslibs_math_2d/linear/point.hpp>
#include <cslibs_time/time_frame.hpp>

#include <limits>
#include <vector>

namespace cslibs_plugins_data {
namespace types {
class Laserscan : public Data
{
public:
    using point_t       = cslibs_math_2d::Point2d;
    using time_frame_t  = cslibs_time::TimeFrame;
    using interval_t    = std::array<double, 2>;

    struct Ray {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using allocator_t = Eigen::aligned_allocator<Ray>;

        const double  angle;
        const double  range;
        const point_t point;

        inline Ray(const double angle,
                   const double range) :
            angle(angle),
            range(range),
            point(point_t(std::cos(angle) * range,
                          std::sin(angle) * range))
        {
        }

        inline Ray(const point_t &pt) :
                   angle(cslibs_math_2d::angle(pt)),
                   range(pt.length()),
                   point(pt)
        {
        }

        inline Ray() :
            angle(0.0),
            range(0.0),
            point(point_t())
        {
        }

        inline Ray(const Ray &other) :
            angle(other.angle),
            range(other.range),
            point(other.point)
        {
        }

        inline Ray(Ray &&other) :
            angle(other.angle),
            range(other.range),
            point(std::move(other.point))
        {
        }

        inline bool valid() const
        {
            return std::isnormal(range);
        }

    };

    using Ptr              = std::shared_ptr<Laserscan>;
    using rays_t           = std::vector<Ray, Ray::allocator_t>;
    using const_iterator_t = rays_t::const_iterator;

    Laserscan(const std::string          &frame,
                const time_frame_t       &time_frame,
                const cslibs_time::Time  &time_received) :
        Data(frame, time_frame, time_received),
        linear_interval_{0.0, std::numeric_limits<double>::max()},
        angular_interval_{-M_PI, M_PI}
    {
    }

    Laserscan(const std::string          &frame,
                const time_frame_t       &time_frame,
                const interval_t         &linear_interval,
                const interval_t         &angular_interval,
                const cslibs_time::Time  &time_received) :
        Data(frame, time_frame, time_received),
        linear_interval_(linear_interval),
        angular_interval_(angular_interval)
    {
    }

    inline void setLinearInterval(const double min,
                                  const double max)
    {
        linear_interval_[0] = min;
        linear_interval_[1] = max;
    }

    inline void setLinearInterval(const interval_t &interval)
    {
        linear_interval_ = interval;
    }

    inline void setAngularInterval(const double min,
                                   const double max)
    {
        angular_interval_[0] = min;
        angular_interval_[1] = max;
    }

    inline void setAngularInterval(const interval_t &interval)
    {
        angular_interval_ = interval;
    }

    inline double getLinearMin() const
    {
        return linear_interval_[0];
    }

    inline double getLinearMax() const
    {
        return linear_interval_[1];
    }

    inline double getAngularMin() const
    {
        return angular_interval_[0];
    }

    inline double getAngularMax() const
    {
        return angular_interval_[1];
    }

    inline void insert(const double angle,
                       const double range)
    {
        rays_.emplace_back(Ray(angle, range));
    }

    inline void insert(const point_t &pt)
    {
        rays_.emplace_back(Ray(pt));
    }

    inline void insertInvalid()
    {
        rays_.emplace_back(Ray());
    }

    inline const_iterator_t begin() const
    {
        return rays_.begin();
    }

    inline const_iterator_t end() const
    {
        return rays_.end();
    }

    inline const rays_t& getRays() const
    {
        return rays_;
    }

private:
    rays_t     rays_;         /// only valid rays shall be contained here
    interval_t linear_interval_;
    interval_t angular_interval_;
}__attribute__ ((aligned (16)));
}
}

#endif // CSLIBS_PLUGINS_DATA_TYPES_LASERSCAN_HPP
