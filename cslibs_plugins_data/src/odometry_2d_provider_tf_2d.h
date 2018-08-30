#ifndef CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_TF_2D_H
#define CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_TF_2D_H

#include <ros/ros.h>
#include <thread>
#include <atomic>

#include <cslibs_plugins_data/data_provider_2d.hpp>
#include <cslibs_math_ros/tf/tf_listener_2d.hpp>

namespace cslibs_plugins_data {
class Odometry2DProviderTF2D : public DataProvider2D
{
public:
    using stamped_t = cslibs_math::utility::Stamped<cslibs_math_2d::Transform2d>;

    Odometry2DProviderTF2D();
    virtual ~Odometry2DProviderTF2D();

protected:
    cslibs_math_ros::tf::TFListener2d     tf_;
    std::string                           odom_frame_;
    std::string                           base_frame_;

    stamped_t                             o_T_b1_;
    bool                                  initialized_;
    ros::Rate                             rate_;
    ros::Duration                         timeout_;
    std::atomic_bool                      running_;
    std::atomic_bool                      stop_;
    std::thread                           worker_thread_;

    virtual void doSetup(ros::NodeHandle &nh) override;
    void loop();
};
}

#endif // CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_TF_2D_H
