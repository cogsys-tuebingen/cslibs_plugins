#ifndef CSLIBS_PLUGINS_PLUGIN_HPP
#define CSLIBS_PLUGINS_PLUGIN_HPP

namespace cslibs_plugins {
class Plugin
{
public:
    virtual inline ~Plugin() = default;

    inline std::string getName() const
    {
        return name_;
    }

    inline void setName(const std::string &name)
    {
        name_ = name;
    }

    inline std::size_t getId() const
    {
        return id_;
    }

    inline void setId(const std::size_t id)
    {
        id_ = id;
    }

protected:
    std::size_t id_;
    std::string name_;
};
}

#endif // CSLIBS_PLUGINS_PLUGIN_HPP
