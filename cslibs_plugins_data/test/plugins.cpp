#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cslibs_math_ros/tf/tf_listener.hpp>
#include <cslibs_plugins/plugin_loader.hpp>
#include <cslibs_plugins/plugin_loader_v2.hpp>
#include <cslibs_plugins_data/data_provider.hpp>

using data_provider_t = cslibs_plugins_data::DataProvider;
using tf_listener_t = cslibs_math_ros::tf::TFListener;

TEST(Test_cslibs_plugins_data, testLoadProviders) {
  ros::NodeHandle nh{"~"};

  const std::string package_name = "cslibs_plugins_data";

  cslibs_plugins::PluginManager<data_provider_t> manager(
      data_provider_t::Type(), package_name);
  manager.load();
  EXPECT_TRUE(manager.pluginsLoaded());

  std::vector<std::string> class_names = {
      "cslibs_plugins_data::LaserProvider",
      "cslibs_plugins_data::LaserProvider_d",
      "cslibs_plugins_data::LaserProvider_f",
      "cslibs_plugins_data::Pointcloud3dProvider",
      "cslibs_plugins_data::Pointcloud3dProvider_d",
      "cslibs_plugins_data::Pointcloud3dProvider_f",
      "cslibs_plugins_data::Odometry2DProvider",
      "cslibs_plugins_data::Odometry2DProvider_d",
      "cslibs_plugins_data::Odometry2DProvider_f",
      "cslibs_plugins_data::Odometry2DProviderTF",
      "cslibs_plugins_data::Odometry2DProviderTF_d",
      "cslibs_plugins_data::Odometry2DProviderTF_f"};

  for (const auto &class_name : class_names) {
    auto constructor = manager.getConstructor(class_name);
    data_provider_t::Ptr plugin;
    if (constructor) {
      plugin = constructor();
    }
    EXPECT_TRUE(static_cast<bool>(constructor));
    EXPECT_TRUE(plugin.get() != nullptr);
  }
}

TEST(Test_cslibs_plugins_data, testParseLaunchFile) {
  ros::NodeHandle nh{"~"};
  cslibs_plugins::LaunchfileParser parser(nh);
  cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
  parser.getNamesForBaseClass<cslibs_plugins_data::DataProvider>(plugins);

  cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
  expected_plugins.emplace("cslibs_plugins_data::LaserProvider", "laser");
  EXPECT_EQ(1ul, expected_plugins.size());
  expected_plugins.emplace("cslibs_plugins_data::LaserProvider_d", "laser_d");
  EXPECT_EQ(2ul, expected_plugins.size());
  expected_plugins.emplace("cslibs_plugins_data::LaserProvider_f", "laser_f");
  EXPECT_EQ(3ul, expected_plugins.size());
  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProvider",
                           "odometry");
  EXPECT_EQ(4ul, expected_plugins.size());
  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProvider_d",
                           "odometry_d");
  EXPECT_EQ(5ul, expected_plugins.size());
  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProvider_f",
                           "odometry_f");
  EXPECT_EQ(6ul, expected_plugins.size());
  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProviderTF",
                           "odometry_tf");
  EXPECT_EQ(7ul, expected_plugins.size());
  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProviderTF_d",
                           "odometry_tf_d");
  EXPECT_EQ(8ul, expected_plugins.size());
  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProviderTF_f",
                           "odometry_tf_f");
  EXPECT_EQ(9ul, expected_plugins.size());
  expected_plugins.emplace("cslibs_plugins_data::Pointcloud3dProvider",
                           "pointcloud");
  EXPECT_EQ(10ul, expected_plugins.size());
  expected_plugins.emplace("cslibs_plugins_data::Pointcloud3dProvider_d",
                           "pointcloud_d");
  EXPECT_EQ(11ul, expected_plugins.size());
  expected_plugins.emplace("cslibs_plugins_data::Pointcloud3dProvider_f",
                           "pointcloud_f");
  EXPECT_EQ(12ul, expected_plugins.size());

  EXPECT_EQ(expected_plugins.size(), plugins.size());
  for (auto plugin : plugins) {
    EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
  }
}

TEST(Test_cslibs_plugins_data, testPluginLoaderV2) {
  ros::NodeHandle nh{"~"};
  cslibs_math_ros::tf::TFProvider::Ptr tf_{new cslibs_math_ros::tf::TFListener};
  cslibs_plugins::PluginLoaderV2 loader("cslibs_plugins_data", nh);

  cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
  const auto &parser = loader.getLaunchFileParser();
  parser->getNamesForBaseClass<cslibs_plugins_data::DataProvider>(plugins);

  cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
  expected_plugins.emplace("cslibs_plugins_data::LaserProvider", "laser");
  expected_plugins.emplace("cslibs_plugins_data::LaserProvider_d", "laser_d");
  expected_plugins.emplace("cslibs_plugins_data::LaserProvider_f", "laser_f");

  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProvider",
                           "odometry");
  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProvider_d",
                           "odometry_d");
  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProvider_f",
                           "odometry_f");

  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProviderTF",
                           "odometry_tf");
  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProviderTF_d",
                           "odometry_tf_d");
  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProviderTF_f",
                           "odometry_tf_f");

  expected_plugins.emplace("cslibs_plugins_data::Pointcloud3dProvider",
                           "pointcloud");
  expected_plugins.emplace("cslibs_plugins_data::Pointcloud3dProvider_d",
                           "pointcloud_d");
  expected_plugins.emplace("cslibs_plugins_data::Pointcloud3dProvider_f",
                           "pointcloud_f");


  for (auto plugin : plugins) {
    EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
  }

  std::map<std::string, cslibs_plugins_data::DataProvider::Ptr> loaded_plugins;
  loader.load<cslibs_plugins_data::DataProvider, decltype(tf_), decltype(nh)&>(loaded_plugins, tf_, nh);
  EXPECT_EQ(12ul, loaded_plugins.size());
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_load_plugins");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
