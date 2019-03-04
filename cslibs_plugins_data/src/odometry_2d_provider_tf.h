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
class EIGEN_ALIGN16 Odometry2DProviderTF : public DataProvider
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using stamped_t     = cslibs_time::Stamped<cslibs_math_2d::Transform2d<T>>;
    using tf_provider_t = cslibs_math_ros::tf::TFProvider<T>;

    Odometry2DProviderTF() :
        o_T_b1_(cslibs_math_2d::Transform2d<T>(), cslibs_time::Time(ros::Time::now().toNSec()).time()),
        initialized_(false),
        rate_(60.0),
        running_(false),
        stop_(false)
    {
    }

    virtual ~Odometry2DProviderTF()
    {
        if( running_) {
            stop_ = true;
            if (worker_thread_.joinable())
                worker_thread_.join();
        }
    }

    inline void doSetup(const typename tf_provider_t::Ptr &tf,
                        ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){ return name_ + "/" + name; };

        tf_         = tf;
        tf_timeout_ = ros::Duration(nh.param<double>(param_name("tf_timeout"), 0.1));
        doSetup(nh);
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

        if (!tf_)
            tf_.reset(new tf_provider_t);
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

    typename tf_provider_t::Ptr tf_;
    ros::Duration               tf_timeout_;

    void loop()
    {
        running_ = true;
        while (!stop_) {
            const ros::Time now = ros::Time::now();
            stamped_t o_T_b2(cslibs_math_2d::Transform2d(), cslibs_time::Time(now.toNSec()).time());
            if (tf_.lookupTransform(odom_frame_, base_frame_, now, o_T_b2, tf_timeout_)) {
                if (initialized_) {
                    cslibs_time::TimeFrame time_frame(o_T_b1_.stamp(), o_T_b2.stamp());
                    types::Odometry2D<T>::Ptr odometry(new types::Odometry2D<T>(odom_frame_,
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
};

using Odometry2DProviderTFDouble = Odometry2DProviderTF<double>;
using Odometry2DProviderTFFloat  = Odometry2DProviderTF<float>;
}

#endif // CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_TF_H
