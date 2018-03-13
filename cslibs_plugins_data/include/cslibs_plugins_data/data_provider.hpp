#ifndef CSLIBS_PLUGINS_DATA_PROVIDER_HPP
#define CSLIBS_PLUGINS_DATA_PROVIDER_HPP

#include <memory>
#include <functional>

#include <cslibs_plugins_data/data.hpp>
#include <cslibs_utility/signals/signals.hpp>
#include <cslibs_utility/common/delegate.hpp>

namespace cslibs_plugins_data {
class DataProvider {
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

    inline std::string getName() const
    {
        return name_;
    }

    void setName(const std::string &name)
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
    std::size_t       id_;
    std::string       name_;
    signal_t          data_received_;
};
}

#endif // CSLIBS_PLUGINS_DATA_PROVIDER_HPP
