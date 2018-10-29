# CS - Library: Plugins
This library contains basic functions to load plugins and implementations of plugins providing data. The code is open-source ([BSD License](LICENSE)) and has been tested under Ubuntu 16.04 with ROS Kinetic. Please note that this project is part of ongoing research and that there will be changes in the future.

## Structure
This project is divided up into the following subpackages:

* [cslibs\_plugins](cslibs_plugins/):<br>
    This package contains utilities to load plugins.

* [cslibs\_plugins\_data](cslibs_plugins_data/):<br>
    This package contains plugins providing data. Currently implemented data types are 2D Laserscans, 3D Pointclouds and Odometry data.

## Usage

### Dependencies

The [cslibs\_plugins](cslibs_plugins/) package does not have any internal dependencies.
<br>
The [cslibs\_plugins\_data](cslibs_plugins_data/) package depends on the following packages of our research group:

* [cslibs\_math](https://github.com/cogsys-tuebingen/cslibs_math)
* [cslibs\_time](https://github.com/cogsys-tuebingen/cslibs_time)
* [cslibs\_utility](https://github.com/cogsys-tuebingen/cslibs_utility)

### Implementing Plugins
To implement a plugin, the easiest way is to inherit from ``cslibs_plugins::Plugin``. Additionally, the following needs to be implemented:

*   ```inline const static std::string Type()```<br>
    This function should return a unique identifier, e.g. class name including namespace of the plugin.

*  ```inline void setup(...)```<br>
    This function is called by the ``cslibs_plugins::PluginFactory`` and sets up the plugin instance. This function may be implemented with any set of arguments which have to be passed to the ``cslibs_plugins::PluginLoader`` when the plugin shall be loaded.

* ```using Ptr = std::shared_ptr<plugin_t>;```<br>
    This typedef also needs to be overridden, where ``plugin_t`` is the newly implemented plugin type.

### Loading Plugins
To load plugins, use an instance of ``cslibs_plugins::PluginLoader``, whose constructor needs the name of the package the plugin that shall be loaded is located in, and a ``ros::NodeHandle``:

    cslibs_plugins::PluginLoader loader("package_name", nh);

Then, call the ``load`` function, which is templated regarding plugin type and the arguments of the ``setup`` function that is required to create the plugin. A single instance ``plugin`` of type ``plugin_t`` is loaded by:

    loader.load<plugin_t, arg1_t, arg2_t, ...>(plugin, arg1, arg2, ...);

Multiple plugins of same type can be loaded concurrently by loading the plugins into a ``std::map<std::string, typename plugin_t::Ptr> plugin_map`` of type ``plugin_t``:

    loader.load<plugin_t, arg1_t, arg2_t, ...>(plugin_map, arg1, arg2, ...);

### Examples
An exemplary abstract plugin definition can be found in [cslibs\_plugins\_data](cslibs_plugins_data/include/cslibs_plugins_data/data_provider.hpp).<br>
The plugins themselves can be found in the [src](cslibs_plugins_data/src/) folder.<br>
Loading and usage of plugins of this type can be found in [muse\_mcl\_2d](https://github.com/cogsys-tuebingen/muse_mcl_2d/blob/c718bdf4ef9a2c308ce53549c18531664da4b818/muse_mcl_2d/src/node/muse_mcl_2d_node.cpp#L150).

## Contributing
[Contribution guidelines for this project](CONTRIBUTING.md)