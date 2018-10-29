#ifndef CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_TF_H
#define CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_TF_H

#include <ros/ros.h>
#include <thread>
#include <atomic>

#include <cslibs_plugins_data/data_provider.hpp>
#include <cslibs_math_ros/tf/tf_listener.hpp>

namespace cslibs_plugins_data {
class EIGEN_ALIGN16 Odometry2DProviderTF : public DataProvider
{
public:
    using stamped_t = cslibs_time::Stamped<cslibs_math_2d::Transform2d>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Odometry2DProviderTF();
    virtual ~Odometry2DProviderTF();

protected:
    cslibs_math_ros::tf::TFListener       tf_;
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

#endif // CSLIBS_PLUGINS_DATA_ODOMETRY_2D_PROVIDER_TF_H
