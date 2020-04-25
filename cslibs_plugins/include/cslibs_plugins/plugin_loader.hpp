#ifndef CSLIBS_PLUGINS_PLUGIN_LOADER_HPP
#define CSLIBS_PLUGINS_PLUGIN_LOADER_HPP

#include <cslibs_plugins/plugin_factory.hpp>
#include <cslibs_plugins/terminal_color.hpp>

#include <map>
#include <ros/node_handle.h>
#if __GNUC__ > 5
# include <regex>
#else
# include <boost/regex.hpp>
#endif

namespace cslibs_plugins {
class PluginLoader
{
public:
    inline PluginLoader(const std::string &package_name,
                        ros::NodeHandle &nh_private) :
        package_name_(package_name),
        nh_private_(nh_private)
    {
        parseLaunchFile();
    }

    template<typename plugin_t, typename ... arguments_t>
    inline bool load(std::map<std::string, typename plugin_t::Ptr> &plugins,
                     const arguments_t&... arguments)
    {
        plugins.clear();

        /// all in the launch file entered plugins have been retrieved now
        /// now we load the ones related to this ProviderManager
        static PluginFactory<plugin_t, arguments_t...> factory{package_name_};
        for (const auto &entry : plugins_found_) {
            const auto &name = entry.first;
            const auto &base_class_name = entry.second.base_class_name;
            const auto &class_name = entry.second.class_name;

            if (base_class_name == plugin_t::Type()) {
                plugins[name] = factory.create(class_name, name, arguments...);
                if (!plugins[name]) {
                    std::cerr << "[PluginFactory]: Could not create plugin '"
                              << io::color::bold(io::color::yellow(name)) << "' with type \n  "
                              << io::color::bold(io::color::green(class_name)) <<" -> "
                              << io::color::bold(io::color::blue(base_class_name)) << ". \n  "
                              << "Empty constructor received!." << std::endl;
                    plugins.erase(name);
                }
            }
        }

        return plugins.size() > 0;
    }

    template<typename plugin_t, typename ... arguments_t>
    inline void load(typename plugin_t::Ptr &plugin,
                     const arguments_t&... arguments)
    {
        static PluginFactory<plugin_t, arguments_t ...> factory(package_name_);
        for (const auto &entry : plugins_found_) {
            const auto &name = entry.first;
            const auto &base_class_name = entry.second.base_class_name;
            const auto &class_name = entry.second.class_name;

            if (base_class_name == plugin_t::Type())
                plugin = factory.create(class_name, name, arguments...);
        }
    }

    inline std::set<std::string> getFoundNames() const
    {
        std::set<std::string> found;
        for (const auto &entry : plugins_found_)
            found.insert(entry.first);

        return found;
    }

private:
    struct LaunchEntry {
        std::string class_name;
        std::string base_class_name;
    };

    std::string     package_name_;
    ros::NodeHandle nh_private_;
    std::map<std::string, LaunchEntry> plugins_found_;

    inline void parseLaunchFile()
    {
        const auto ns = nh_private_.getNamespace();

        /// first parse the parameters
        std::vector<std::string> params;
        nh_private_.getParamNames(params);

        const std::string class_regex_str = "(" + ns + "/)(.*)(/class)";
        const std::string base_class_regex_str = "(" + ns + "/)(.*)(/base_class)";

#if __GNUC__ > 5
        std::regex class_regex(class_regex_str);
        std::regex base_class_regex(base_class_regex_str);
        std::cmatch match;
        for (const auto &p : params) {
            if (std::regex_match(p.c_str(), match, class_regex))
                nh_private_.getParam(p, plugins_found_[match[2]].class_name);

            if (std::regex_match(p.c_str(), match, base_class_regex))
                nh_private_.getParam(p, plugins_found_[match[2]].base_class_name);
        }
#else
        boost::regex class_regex(class_regex_str);
        boost::regex base_class_regex(base_class_regex_str);
        boost::cmatch match;
        for (const auto &p : params) {
            if (boost::regex_match(p.c_str(), match, class_regex))
                nh_private_.getParam(p, plugins_found_[match[2]].class_name);

            if (boost::regex_match(p.c_str(), match, base_class_regex))
                nh_private_.getParam(p, plugins_found_[match[2]].base_class_name);
        }
#endif
    }
};
}

#endif // CSLIBS_PLUGINS_PLUGIN_LOADER_HPP
