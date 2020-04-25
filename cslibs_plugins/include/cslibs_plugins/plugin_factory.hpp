#ifndef CSLIBS_PLUGINS_PLUGIN_FACTORY_HPP
#define CSLIBS_PLUGINS_PLUGIN_FACTORY_HPP

#include <cslibs_plugins/plugin_manager.hpp>
#include <string>

namespace cslibs_plugins {
template <typename plugin_t, typename... setup_args_t>
class PluginFactory {
 public:
  inline PluginFactory(const std::string &package_name)
      : plugin_manager{plugin_t::Type(), package_name} {
    plugin_manager.load();
  }

  inline typename plugin_t::Ptr create(const std::string &class_name,
                                       const std::string &plugin_name,
                                       const setup_args_t &... arguments) {
    if (auto constructor = plugin_manager.getConstructor(class_name)) {
      typename plugin_t::Ptr plugin = constructor();
      plugin->setName(plugin_name);
      plugin->setId(++plugin_id_);
      plugin->setup(arguments...);
      return plugin;
    }
    return nullptr;
  }

  inline static const std::string Type() { return plugin_t::Type(); }

 protected:
  PluginManager<plugin_t> plugin_manager;
  std::size_t plugin_id_{0};
};
}  // namespace cslibs_plugins

#endif  // CSLIBS_PLUGINS_PLUGIN_FACTORY_HPP
