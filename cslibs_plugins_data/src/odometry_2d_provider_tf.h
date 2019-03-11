#ifndef CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_TF_H
#define CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_TF_H

#include <ros/ros.h>
#include <thread>
#include <atomic>

#include <cslibs_math_ros/tf/tf_listener.hpp>
#include <cslibs_plugins_data/data_provider.hpp>
#include <cslibs_plugins_data/types/odometry_2d.hpp>
#include <tf/tf.h>

namespace cslibs_plugins_data {
template <typename T>
class EIGEN_ALIGN16 Odometry2DProviderTFBase : public DataProvider
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using stamped_t     = cslibs_time::Stamped<cslibs_math_2d::Transform2d<T>>;

    Odometry2DProviderTFBase() :
        o_T_b1_(cslibs_math_2d::Transform2d<T>(), cslibs_time::Time(ros::Time::now().toNSec()).time()),
        initialized_(false),
        rate_(60.0),
        running_(false),
        stop_(false)
    {
    }

    virtual ~Odometry2DProviderTFBase()
    {
        if( running_) {
            stop_ = true;
            if (worker_thread_.joinable())
                worker_thread_.join();
        }
    }

protected:
    std::string      odom_frame_;
    std::string      base_frame_;

    stamped_t        o_T_b1_;
    bool             initialized_;
    ros::Rate        rate_;
    std::atomic_bool running_;
    std::atomic_bool stop_;
    std::thread      worker_thread_;

    void loop()
    {
        running_ = true;
        while (!stop_) {
            const ros::Time now = ros::Time::now();
            stamped_t o_T_b2(cslibs_math_2d::Transform2d<T>(), cslibs_time::Time(now.toNSec()).time());
            if (tf_->lookupTransform(odom_frame_, base_frame_, now, o_T_b2, tf_timeout_)) {
                if (initialized_) {
                    cslibs_time::TimeFrame time_frame(o_T_b1_.stamp(), o_T_b2.stamp());
                    typename types::Odometry2D<T>::Ptr odometry(new types::Odometry2D<T>(odom_frame_,
                                                                                         time_frame,
                                                                                         o_T_b1_.data(),
                                                                                         o_T_b2.data(),
                                                                                         cslibs_time::Time(ros::Time::now().toNSec())));
                    data_received_(odometry);
                } else
                    initialized_ = true;
                o_T_b1_ = o_T_b2;
            }
            rate_.sleep();
        }
        running_ = false;
    }

    virtual inline void doSetup(ros::NodeHandle &nh) override
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};

        odom_frame_ = nh.param<std::string>(param_name("odom_frame"), "/odom");
        base_frame_ = nh.param<std::string>(param_name("base_frame"), "/base_link");
        rate_       = ros::Rate(nh.param<double>(param_name("rate"), 70.0));

        if (!running_) {
            /// fire up the thread
            worker_thread_ = std::thread([this](){ loop();} );
        }
    }
};

using Odometry2DProviderTF   = Odometry2DProviderTFBase<double>; // for backwards compatibility
using Odometry2DProviderTF_d = Odometry2DProviderTFBase<double>;
using Odometry2DProviderTF_f = Odometry2DProviderTFBase<float>;
}

#endif // CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_TF_H
