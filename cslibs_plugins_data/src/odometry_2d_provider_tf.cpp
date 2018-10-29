#include "odometry_2d_provider_tf.h"

#include <cslibs_plugins_data/types/odometry_2d.hpp>
#include <tf/tf.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(cslibs_plugins_data::Odometry2DProviderTF, cslibs_plugins_data::DataProvider)

namespace cslibs_plugins_data {
Odometry2DProviderTF::Odometry2DProviderTF() :
    o_T_b1_(cslibs_math_2d::Transform2d(), cslibs_time::Time(ros::Time::now().toNSec()).time()),
    initialized_(false),
    rate_(60.0),
    running_(false),
    stop_(false)
{
}

Odometry2DProviderTF::~Odometry2DProviderTF()
{
    if( running_) {
        stop_ = true;
        if (worker_thread_.joinable())
            worker_thread_.join();
    }
}

void Odometry2DProviderTF::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    odom_frame_ = nh.param<std::string>(param_name("odom_frame"), "/odom");
    base_frame_ = nh.param<std::string>(param_name("base_frame"), "/base_link");
    rate_       = ros::Rate(nh.param<double>(param_name("rate"), 70.0));
    timeout_    = ros::Duration(nh.param<double>(param_name("timeout"), 0.1));

    if (!running_) {
        /// fire up the thread
        worker_thread_ = std::thread([this](){ loop();} );
    }
}

void Odometry2DProviderTF::loop()
{
    running_ = true;
    while (!stop_) {
        const ros::Time now = ros::Time::now();
        stamped_t o_T_b2(cslibs_math_2d::Transform2d(), cslibs_time::Time(now.toNSec()).time());
        if (tf_.lookupTransform(odom_frame_, base_frame_, now, o_T_b2, timeout_)) {
            if (initialized_) {
                cslibs_time::TimeFrame time_frame(o_T_b1_.stamp(), o_T_b2.stamp());
                types::Odometry2D::Ptr odometry(new types::Odometry2D(odom_frame_,
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
}
