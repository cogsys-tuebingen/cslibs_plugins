#ifndef CSLIBS_PLUGINS_PLUGIN_MANAGER_HPP
#define CSLIBS_PLUGINS_PLUGIN_MANAGER_HPP

/// SYSTEM
#include <tinyxml.h>

#include <class_loader/class_loader.hpp>
#include <cslibs_utility/common/delegate.hpp>
#include <functional>
#include <mutex>
#include <pluginlib/class_loader.hpp>
#include <set>
#include <typeindex>

namespace cslibs_plugins {
/**
 * @brief Class that parses xml files provided by packges, searching for a
 * specific tag. A specific type of plugin is expected.
 */
template <class M>
class PluginManagerImp {
 public:
  virtual ~PluginManagerImp() = default;

 protected:
  template <class>
  friend class PluginManager;
  using PluginConstructorM =
      cslibs_utility::common::delegate<std::shared_ptr<M>()>;
  using Constructors = std::map<std::string, PluginConstructorM>;
  /**
   * @brief PluginManager constructor.
   * @param base_class_type   full name of the base class including the
   * namespace
   * @param package_name      name of the package for which the plugin was
   * exported
   */
  inline explicit PluginManagerImp(const std::string& base_class_type,
                                   const std::string& package_name)
      : base_class_type_(base_class_type),
        loader_(package_name, base_class_type) {}

  inline PluginManagerImp(const PluginManagerImp& rhs) = default;
  inline PluginManagerImp& operator=(const PluginManagerImp& rhs) = default;

  inline void load() {
    auto xml_files = loader_.getPluginXmlPaths();
    for (const auto& manifest : xml_files) {
      processManifest(manifest);
    }
    plugins_loaded_ = true;
  }

  inline bool processManifest(const std::string& xml_file) {
    TiXmlDocument document;
    document.LoadFile(xml_file);
    const auto config = document.RootElement();
    if (config == nullptr) return false;

    if (config->ValueStr() != "library") return false;

    auto library = config;
    while (library != nullptr) {
      const std::string library_name{library->Attribute("path")};
      if (library_name.size() == 0) continue;

      loadLibrary(library_name, library);
      library = library->NextSiblingElement("library");
    }

    return true;
  }

  inline std::string loadLibrary(const std::string& library_name,
                                 TiXmlElement* library) {
    const auto library_path = library_name + ".so";

    bool load = false;
    {
      auto class_element = library->FirstChildElement("class");
      while (class_element) {
        std::string base_class_type{
            class_element->Attribute("base_class_type")};
        if (base_class_type == base_class_type_) {
          load = true;
          break;
        }
        class_element = class_element->NextSiblingElement("class");
      }
    }

    if (!load) {
      return "";
    }

    std::shared_ptr<class_loader::ClassLoader> loader(
        new class_loader::ClassLoader(library_path));

    auto class_element = library->FirstChildElement("class");
    while (class_element) {
      loadClass(library_name, class_element, loader.get());
      class_element = class_element->NextSiblingElement("class");
    }
    loaders_[library_name] = std::move(loader);

    return library_path;
  }

  inline void loadClass(const std::string& library_name,
                        TiXmlElement* class_element,
                        class_loader::ClassLoader* loader) {
    const std::string base_class_type{
        class_element->Attribute("base_class_type")};
    const std::string derived_class{class_element->Attribute("type")};

    const std::string lookup_name{(class_element->Attribute("name") != nullptr)
                                      ? class_element->Attribute("name")
                                      : derived_class};

    if (base_class_type == base_class_type_) {
      const auto description = readString(class_element, "description");
      const auto icon = readString(class_element, "icon");
      const auto tags = readString(class_element, "tags");

      available_classes.emplace(lookup_name, [loader, lookup_name]() {
        return std::shared_ptr<M>{
            loader->createUnmanagedInstance<M>(lookup_name)};
      });
    }
  }

  inline std::string readString(TiXmlElement* class_element,
                                const std::string& name) {
    auto description = class_element->FirstChildElement(name);
    std::string description_str{
        (description && description->GetText()) ? description->GetText() : ""};

    return description_str;
  }

 protected:
  bool plugins_loaded_{false};
  std::string base_class_type_;

  pluginlib::ClassLoader<M> loader_;
  std::map<std::string, std::shared_ptr<class_loader::ClassLoader>> loaders_;

  Constructors available_classes;
};

class PluginManagerLocker {
 public:
  inline static std::mutex& getMutex() {
    static PluginManagerLocker instance;
    return instance.mutex;
  }

 private:
  PluginManagerLocker() = default;
  std::mutex mutex;
};

template <class M>
class PluginManager {
 protected:
  using Parent = PluginManagerImp<M>;

 public:
  using Constructor = typename Parent::PluginConstructorM;
  using Constructors = typename Parent::Constructors;

  /**
   * @brief PluginManager constructor.
   * @param base_class_type   full name of the base class including the
   * namespace
   * @param package_name      name of the package for which the plugin was
   * exported
   */
  inline explicit PluginManager(const std::string& base_class_type,
                                const std::string& package_name) {
    std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
    if (i_count == 0) {
      ++i_count;
      instance = new Parent(base_class_type, package_name);
    }
  }

  inline virtual ~PluginManager() {
    std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
    --i_count;
    if (i_count == 0) delete instance;
  }

  inline bool pluginsLoaded() const {
    std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
    return instance->plugins_loaded_;
  }

  inline void load() {
    std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
    instance->load();
  }

  inline Constructor getConstructor(const std::string& name) {
    std::unique_lock<std::mutex> lock(PluginManagerLocker::getMutex());
    auto pos = instance->available_classes.find(name);
    if (pos != instance->available_classes.end())
      return pos->second;
    else
      return {};
  }

 protected:
  static int i_count;
  static Parent* instance;
};

template <class M>
int PluginManager<M>::i_count = 0;
template <class M>
typename PluginManager<M>::Parent* PluginManager<M>::instance(nullptr);

}  // namespace cslibs_plugins

#endif  // CSLIBS_PLUGINS_PLUGIN_MANAGER_HPP
