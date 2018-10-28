#ifndef CSLIBS_PLUGINS_DATA_PROVIDER_2D_HPP
#define CSLIBS_PLUGINS_DATA_PROVIDER_2D_HPP

#include <cslibs_plugins_data/data_provider.hpp>
#include <cslibs_math_ros/tf/tf_provider.hpp>
#include <ros/node_handle.h>

namespace cslibs_plugins_data {
class DataProvider2D : public DataProvider
{
public:
    using Ptr = std::shared_ptr<DataProvider2D>;

    inline const static std::string Type()
    {
        return "cslibs_plugins_data::DataProvider2D";
    }

    inline void setup(const cslibs_math_ros::tf::TFProvider::Ptr &tf,
                      ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){ return name_ + "/" + name; };

        tf_         = tf;
        tf_timeout_ = ros::Duration(nh.param<double>(param_name("tf_timeout"), 0.1));
        doSetup(nh);
    }

protected:
    cslibs_math_ros::tf::TFProvider::Ptr   tf_;
    ros::Duration                          tf_timeout_;

    virtual void doSetup(ros::NodeHandle &nh) = 0;
};
}

#endif // CSLIBS_PLUGINS_DATA_PROVIDER_2D_HPP
