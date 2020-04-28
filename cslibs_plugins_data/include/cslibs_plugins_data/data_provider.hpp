#ifndef CSLIBS_PLUGINS_DATA_PROVIDER_HPP
#define CSLIBS_PLUGINS_DATA_PROVIDER_HPP

#include <ros/node_handle.h>

#include <cslibs_math_ros/tf/tf_provider.hpp>
#include <cslibs_plugins/plugin.hpp>
#include <cslibs_plugins_data/data.hpp>
#include <cslibs_utility/common/delegate.hpp>
#include <cslibs_utility/signals/signals.hpp>
#include <functional>
#include <memory>

namespace cslibs_plugins_data {
class DataProvider : public cslibs_plugins::Plugin {
 public:
  using Ptr = std::shared_ptr<DataProvider>;

  using callback_t =
      cslibs_utility::common::delegate<void(const Data::ConstPtr &)>;
  using signal_t = cslibs_utility::signals::Signal<callback_t>;
  using connection_t = signal_t::Connection;
  using tf_provider_t = cslibs_math_ros::tf::TFProvider;

  /**
   * @brief the default constructor
   */
  inline DataProvider() = default;
  /**
   * @brief the default destructor
   */
  virtual ~DataProvider() = default;

  /**
   * @brief Returns the type as string for the pluginlib.
   * @return the type as string
   */
  inline static std::string const Type() {
    return "cslibs_plugins_data::DataProvider";
  }

  /**
   * @brief Set up the data provider by passing a tf provider and ROS node handle.
   * @param tf      the tf provider
   * @param nh      the ros node handle
   */
  inline void setup(const typename tf_provider_t::Ptr &tf,
                    ros::NodeHandle &nh) {
    auto param_name = [this](const std::string &name) {
      return name_ + "/" + name;
    };

    tf_ = tf;
    tf_timeout_ =
        ros::Duration(nh.param<double>(param_name("tf_timeout"), 0.1));
    doSetup(nh);
  }

  /**
   * @brief Connect to data provider. Callback will be executed as
   *        long as the connection object is alive.
   * @param callback    function to call
   * @return connection for a given callback
   */
  connection_t::Ptr connect(const callback_t &callback) {
    return data_received_.connect(callback);
  }

  /**
   * @brief Enable data to be pushed through.
   */
  void enable() { data_received_.enable(); }

  /**
   * @brief Disable data to be pushed through.
   */
  void disable() { data_received_.disable(); }

  /**
   * @brief Test if publisher has a certain type.
   */
  template <typename T>
  bool isType() const {
    const T *t = dynamic_cast<const T *>(this);
    return t != nullptr;
  }

  /**
   * @brief Helper method for casting providers.
   */
  template <typename T>
  T const &as() const {
    return dynamic_cast<const T &>(*this);
  }

  /**
   * @brief Helper method for casting providers.
   */
  template <typename T>
  T &as() {
    return dynamic_cast<T &>(*this);
  }

 protected:
  signal_t data_received_;

  typename tf_provider_t::Ptr tf_;
  ros::Duration tf_timeout_;

  virtual void doSetup(ros::NodeHandle &nh) = 0;
};
}  // namespace cslibs_plugins_data

#endif  // CSLIBS_PLUGINS_DATA_PROVIDER_HPP
