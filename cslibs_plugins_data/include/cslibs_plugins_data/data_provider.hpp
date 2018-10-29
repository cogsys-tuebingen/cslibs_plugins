#ifndef CSLIBS_PLUGINS_DATA_PROVIDER_HPP
#define CSLIBS_PLUGINS_DATA_PROVIDER_HPP

#include <memory>
#include <functional>

#include <cslibs_plugins/plugin.hpp>
#include <cslibs_plugins_data/data.hpp>

#include <cslibs_utility/signals/signals.hpp>
#include <cslibs_utility/common/delegate.hpp>

#include <cslibs_math_ros/tf/tf_provider.hpp>
#include <ros/node_handle.h>

namespace cslibs_plugins_data {
class DataProvider : public cslibs_plugins::Plugin
{
public:
    using Ptr          = std::shared_ptr<DataProvider>;
    using callback_t   = cslibs_utility::common::delegate<void(const Data::ConstPtr&)>;
    using signal_t     = cslibs_utility::signals::Signal<callback_t>;
    using connection_t = signal_t::Connection;

    DataProvider() = default;
    virtual ~DataProvider() = default;

    inline const static std::string Type()
    {
        return "cslibs_plugins_data::DataProvider";
    }

    inline void setup(const cslibs_math_ros::tf::TFProvider::Ptr &tf,
                      ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){ return name_ + "/" + name; };

        tf_         = tf;
        tf_timeout_ = ros::Duration(nh.param<double>(param_name("tf_timeout"), 0.1));
        doSetup(nh);
    }

     /**
     * @brief Connect to data provider. Callback will be executed as
     *        long as the connection object is alive.
     * @param callback - function to call
     * @return
     */
    connection_t::Ptr connect(const callback_t &callback)
    {
        return data_received_.connect(callback);
    }

    /**
     * @brief Enable data to be pushed through.
     */
    void enable()
    {
        data_received_.enable();
    }

    /**
     * @brief Disable data to be pushed through.
     */
    void disable()
    {
        data_received_.disable();
    }

protected:
    signal_t                                data_received_;

    cslibs_math_ros::tf::TFProvider::Ptr    tf_;
    ros::Duration                           tf_timeout_;

    virtual void doSetup(ros::NodeHandle &nh) = 0;

};
}

#endif // CSLIBS_PLUGINS_DATA_PROVIDER_HPP
