#ifndef CSLIBS_PLUGINS_LAUNCH_FILE_PARSER_HPP
#define CSLIBS_PLUGINS_LAUNCH_FILE_PARSER_HPP

#include <ros/node_handle.h>

#include <cslibs_plugins/terminal_color.hpp>
#include <map>
#include <regex>
#include <set>

namespace cslibs_plugins {
class LaunchfileParser {
 public:
  inline explicit LaunchfileParser(ros::NodeHandle &nh_private)
      : nh_private_{nh_private} {
    parseLaunchFile();
  }

  struct FoundPlugin {
    inline explicit FoundPlugin(const std::string &_class_name,
                                const std::string &_name)
        : class_name{_class_name}, name{_name} {}

    std::string class_name;
    std::string name;

    struct less {
      inline bool operator()(const FoundPlugin &a, const FoundPlugin &b) const {
        return a.name < b.name && a.class_name < b.class_name;
      }
    };
  };

  using found_plugin_set_t = std::set<FoundPlugin, FoundPlugin::less>;

  template <typename plugin_t>
  inline void getNamesForBaseClass(found_plugin_set_t &found_plugins) const {
    const std::string base_class_name = plugin_t::Type();

    if (plugins_.find(base_class_name) == plugins_.end()) {
      return;
    }

    const auto &base_class_entry = plugins_.at(base_class_name);
    for (const auto &class_entry : base_class_entry) {
      const auto &class_name = class_entry.first;
      const auto &names = class_entry.second;
      for (const auto &name : names) {
        found_plugins.emplace(class_name, name);
      }
    }
  }

 private:
  struct LaunchEntry {
    std::string class_name;
    std::string base_class_name;
  };

  ros::NodeHandle& nh_private_;
  std::map<std::string, std::map<std::string, std::set<std::string>>> plugins_;

  inline void parseLaunchFile() {
    const auto ns = nh_private_.getNamespace();

    /// first parse the parameters
    std::vector<std::string> params;
    nh_private_.getParamNames(params);

    const std::string class_regex_str = "(" + ns + "/)(.*)(/class)";
    const std::string base_class_regex_str = "(" + ns + "/)(.*)(/base_class)";

    std::regex class_regex(class_regex_str);
    std::regex base_class_regex(base_class_regex_str);
    std::cmatch match;
    std::map<std::string, LaunchEntry> buffer;

    for (const auto &p : params) {
      if (std::regex_match(p.c_str(), match, class_regex)) {
        const auto name = match[2];
        nh_private_.getParam(p, buffer[name].class_name);
      }
      if (std::regex_match(p.c_str(), match, base_class_regex)) {
        const auto name = match[2];
        nh_private_.getParam(p, buffer[name].base_class_name);
      }
    }

    for (const auto &entry : buffer) {
      const auto &base_class_name = entry.second.base_class_name;
      const auto &class_name = entry.second.class_name;
      const auto &name = entry.first;
      plugins_[base_class_name][class_name].emplace(name);
    }
  }
};
}  // namespace cslibs_plugins

#endif  // CSLIBS_PLUGINS_LAUNCH_FILE_PARSER_HPP
