#ifndef CSLIBS_PLUGINS_PLUGIN_LOADER_V2_HPP
#define CSLIBS_PLUGINS_PLUGIN_LOADER_V2_HPP

#include <ros/node_handle.h>

#include <cslibs_plugins/launch_file_parser.hpp>
#include <cslibs_plugins/plugin_manager.hpp>
#include <cslibs_plugins/terminal_color.hpp>
#include <map>
#include <memory>
#include <regex>

namespace cslibs_plugins {
class PluginLoaderV2 {
 public:
  inline explicit PluginLoaderV2(const std::string &package_name)
      : package_name_{package_name},
        launch_file_parser_{new LaunchfileParser{nh_private_}},
        nh_private_{"~"} {}

  template <typename plugin_t, typename... arguments_t>
  inline bool load(std::map<std::string, typename plugin_t::Ptr> &plugins,
                   const arguments_t &... arguments) {
    plugins.clear();

    // get plugin manager instance and the id counter
    auto *plugin_manager = getInstance<plugin_t>();
    auto &id = getId<plugin_t>();

    // get all plugins for this type
    const auto base_class_name = plugin_t::Type();
    LaunchfileParser::found_plugin_set_t found_plugins_for_type;
    launch_file_parser_->getNamesForBaseClass<plugin_t>(found_plugins_for_type);

    // create all plugins in list
    for (const auto &plugin_entry : found_plugins_for_type) {
      const auto &name = plugin_entry.name;
      const auto &class_name = plugin_entry.class_name;

      auto constructor = plugin_manager->getConstructor(class_name);
      if (constructor) {
        auto *p = constructor();
        p->setName(name);
        p->setId(++id);
        p->setup(arguments...);
        plugins[name].reset(p);
      } else {
        printError(name, class_name, base_class_name);
      }
    }

    return plugins.size() > 0;
  }

  template <typename plugin_t, typename... arguments_t>
  inline void load(typename plugin_t::Ptr &plugin,
                   const arguments_t &... arguments) {
    plugin.reset();

    auto *plugin_manager = getInstance<plugin_t>();
    auto &id = getId<plugin_t>();

    // get all plugins for this type
    const auto base_class_name = plugin_t::Type();
    LaunchfileParser::found_plugin_set_t found_plugins_for_type;
    launch_file_parser_->getNamesForBaseClass<plugin_t>(found_plugins_for_type);

    // get the last plugin entry
    if (found_plugins_for_type.empty()) {
      return;
    }

    const auto &plugin_entry = *(found_plugins_for_type.rbegin());
    const auto &name = plugin_entry.name;
    const auto &class_name = plugin_entry.class_name;
    auto constructor = plugin_manager->getConstructor(class_name);
    if (constructor) {
      auto *p = constructor();
      p->setName(name);
      p->setId(++id);
      p->setup(arguments...);
      plugin.reset(p);
    } else {
      printError(name, class_name, base_class_name);
    }
  }

 private:
  struct PluginManager {
    using Ptr = std::unique_ptr<PluginManager>;
  };

  template <typename plugin_t>
  struct PluginManagerInstance : public PluginManager {
    std::unique_ptr<cslibs_plugins::PluginManager<plugin_t>> instance_{
        new cslibs_plugins::PluginManager<plugin_t>};
  };

  std::string package_name_;
  std::unique_ptr<LaunchfileParser> launch_file_parser_;
  std::map<std::string, PluginManager::Ptr> plugin_managers_;
  std::map<std::string, std::size_t> plugin_ids_;

  /**
   * @brief Returns plugin manager instance and creates it, if necessary.
   * @ return pointer to the instance wrapper
   */
  template <typename plugin_t>
  cslibs_plugins::PluginManager<plugin_t> *getInstance() {
    PluginManagerInstance<plugin_t> *instance{nullptr};
    const auto type_id_name = typeid(plugin_t).name();
    if (plugin_managers_.find(type_id_name) == plugin_managers_.end()) {
      instance = new PluginManagerInstance<plugin_t>;
      plugin_managers_[type_id_name].reset(instance);
    } else {
      auto &instance_entry = plugin_managers_[type_id_name];
      instance = std::dynamic_pointer_cast<PluginManagerInstance<plugin_t>>(
          instance_entry.get());
    }
    return instance->instance_.get();
  }

  /**
   * @brief Returns the stored id counter for a plugin type by reference.
   * @reference to the id counter
   */
  template <typename plugin_t>
  std::size_t &getId() {
    const auto type_id_name = typeid(plugin_t).name();
    if (plugin_ids_.find(type_id_name) == plugin_ids_.end()) {
      plugin_ids_[type_id_name] = 0;
    }
    return plugin_ids_[type_id_name];
  }

  /**
   * @brief Prints an error message for plugins that could not be created.
   */
  void printError(const std::string &name, const std::string &class_name,
                  const std ::string &base_class_name) {
    std::cerr << "[PluginFactory]: Could not create plugin '"
              << io::color::bold(io::color::yellow(name)) << "' with type \n  "
              << io::color::bold(io::color::green(class_name)) << " -> "
              << io::color::bold(io::color::blue(base_class_name)) << ". \n  "
              << "Empty constructor received!." << std::endl;
  }

  ros::NodeHandle nh_private_;
};
}  // namespace cslibs_plugins

#endif  // CSLIBS_PLUGINS_PLUGIN_LOADER_V2_HPP
