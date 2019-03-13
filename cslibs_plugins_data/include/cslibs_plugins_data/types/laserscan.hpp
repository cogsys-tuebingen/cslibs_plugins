#ifndef CSLIBS_PLUGINS_DATA_TYPES_LASERSCAN_HPP
#define CSLIBS_PLUGINS_DATA_TYPES_LASERSCAN_HPP

#include <cslibs_plugins_data/data.hpp>

#include <cslibs_math_2d/linear/point.hpp>
#include <cslibs_time/time_frame.hpp>

#include <limits>
#include <vector>

namespace cslibs_plugins_data {
namespace types {
template <typename T>
class EIGEN_ALIGN16 Laserscan2 : public Data
{
public:    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t   = Eigen::aligned_allocator<Laserscan2<T>>;

    using point_t       = cslibs_math_2d::Point2<T>;
    using time_frame_t  = cslibs_time::TimeFrame;
    using interval_t    = std::array<T, 2>;

    /**
     * @brief The Ray struct represents a scan ray with start point, end point, angle and range.
     */
    struct EIGEN_ALIGN16 Ray {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using allocator_t = Eigen::aligned_allocator<Ray>;

        const T       angle;
        const T       range;
        const point_t end_point;
        const point_t start_point;

        /**
         * @brief Ray constructor.
         * @param angle         - ray angle
         * @param range         - ray range
         * @param start_point   - the point the ray originates from
         */
        inline Ray(const T       angle,
                   const T       range,
                   const point_t start_point = point_t()) :
            angle(angle),
            range(range),
            end_point(point_t(std::cos(angle) * range,
                              std::sin(angle) * range)),
            start_point(start_point)
        {
        }

        /**
         * @brief Ray constructor.
         * @param end_point     - the point the ray is ending at
         * @param start_point   - the point the ray originates from
         */
        inline Ray(const point_t &end_point,
                   const point_t &start_point = point_t()) :
                   angle(cslibs_math_2d::angle(end_point - start_point)),
                   range((end_point  - start_point).length()),
                   end_point(end_point),
                   start_point(start_point)
        {
        }

        /**
         * @brief Ray
         * @param angle         - ray angle
         * @param range         - ray range
         * @param end_point     - the point the ray is ending at
         * @param start_point   - the point the ray originates from
         */
        inline Ray(const T       angle,
                   const T       range,
                   const point_t &end_point,
                   const point_t &start_point) :
                   angle(angle),
                   range(range),
                   end_point(end_point),
                   start_point(start_point)
        {
        }

        /**
         * @brief Ray empty constructor resulting in invalid ray.
         */
        inline Ray() :
            angle(0.0),
            range(0.0),
            end_point(point_t()),
            start_point(point_t())
        {
        }

        /**
         * @brief Ray copy constructor.
         * @param other         - ray to make a copy from
         */
        inline Ray(const Ray &other) :
            angle(other.angle),
            range(other.range),
            end_point(other.end_point),
            start_point(other.start_point)
        {
        }

        /**
         * @brief Ray move constructor.
         * @param other         - the ray to move
         */
        inline Ray(Ray &&other) :
            angle(other.angle),
            range(other.range),
            end_point(std::move(other.end_point)),
            start_point(std::move(other.start_point))
        {
        }

        /**
         * @brief Check wehter a ray can be considered valid.
         * @return
         */
        inline bool valid() const
        {
            return std::isnormal(range) && range > 0.0;
        }

    };

    using Ptr              = std::shared_ptr<Laserscan2<T>>;
    using ConstPtr         = std::shared_ptr<const Laserscan2<T>>;
    using rays_t           = std::vector<Ray, typename Ray::allocator_t>;
    using const_iterator_t = typename rays_t::const_iterator;

    Laserscan2(const std::string          &frame,
              const time_frame_t       &time_frame,
              const cslibs_time::Time  &time_received) :
        Data(frame, time_frame, time_received),
        linear_interval_{0.0, std::numeric_limits<T>::max()},
        angular_interval_{-M_PI, M_PI}
    {
    }

    Laserscan2(const std::string        &frame,
              const time_frame_t       &time_frame,
              const interval_t         &linear_interval,
              const interval_t         &angular_interval,
              const cslibs_time::Time  &time_received) :
        Data(frame, time_frame, time_received),
        linear_interval_(linear_interval),
        angular_interval_(angular_interval)
    {
    }

    inline void setLinearInterval(const T min,
                                  const T max)
    {
        linear_interval_[0] = min;
        linear_interval_[1] = max;
    }

    inline void setLinearInterval(const interval_t &interval)
    {
        linear_interval_ = interval;
    }

    inline void setAngularInterval(const T min,
                                   const T max)
    {
        angular_interval_[0] = min;
        angular_interval_[1] = max;
    }

    inline void setAngularInterval(const interval_t &interval)
    {
        angular_interval_ = interval;
    }

    inline T getLinearMin() const
    {
        return linear_interval_[0];
    }

    inline T getLinearMax() const
    {
        return linear_interval_[1];
    }

    inline T getAngularMin() const
    {
        return angular_interval_[0];
    }

    inline T getAngularMax() const
    {
        return angular_interval_[1];
    }

    inline void insert(const T       angle,
                       const T       range,
                       const point_t &start_point = point_t())
    {
        rays_.emplace_back(Ray(angle, range, start_point));
    }

    inline void insert(const point_t &end_point,
                       const point_t &start_point = point_t())
    {
        rays_.emplace_back(Ray(end_point, start_point));
    }

    inline void insert(const T       angle,
                       const T       range,
                       const point_t &end_point,
                       const point_t &start_point = point_t())
    {
        rays_.emplace_back(Ray(angle, range, end_point, start_point));
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
};

using Laserscan2d = Laserscan2<double>;
using Laserscan2f = Laserscan2<float>;
}
}

#endif // CSLIBS_PLUGINS_DATA_TYPES_LASERSCAN_HPP
