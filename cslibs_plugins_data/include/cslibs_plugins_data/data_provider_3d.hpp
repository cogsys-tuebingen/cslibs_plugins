#ifndef CSLIBS_PLUGINS_DATA_PROVIDER_3D_HPP
#define CSLIBS_PLUGINS_DATA_PROVIDER_3D_HPP

#include <cslibs_plugins_data/data_provider.hpp>
#include <cslibs_math_ros/tf/tf_listener_3d.hpp>

namespace cslibs_plugins_data {
class DataProvider3D : public DataProvider
{
public:
    using Ptr = std::shared_ptr<DataProvider3D>;

    inline const static std::string Type()
    {
        return "cslibs_plugins_data::DataProvider3D";
    }

    inline void setup(const cslibs_math_ros::tf::TFListener3d::Ptr &tf,
                      ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){ return name_ + "/" + name; };

        tf_         = tf;
        tf_timeout_ = ros::Duration(nh.param<double>(param_name("tf_timeout"), 0.1));
        doSetup(nh);
    }

protected:
    cslibs_math_ros::tf::TFListener3d::Ptr tf_;
    ros::Duration                          tf_timeout_;

    virtual void doSetup(ros::NodeHandle &nh) = 0;
};
}

#endif // CSLIBS_PLUGINS_DATA_PROVIDER_3D_HPP
