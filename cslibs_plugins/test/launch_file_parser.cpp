#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cslibs_plugins/launch_file_parser.hpp>

struct UpdateModel2D {
  static std::string Type() { return "muse_mcl_2d::UpdateModel2D"; }
};

struct PredictionModel2D {
  static std::string Type() { return "muse_mcl_2d::PredictionModel2D"; }
};

struct DataProvider {
  static std::string Type() { return "cslibs_plugins_data::DataProvider"; }
};

struct MapProvider2D {
  static std::string Type() { return "muse_mcl_2d::MapProvider2D"; }
};

struct Resampling2D {
  static std::string Type() { return "muse_mcl_2d::Resampling2D"; }
};

struct Uniform2D {
  static std::string Type() { return "muse_mcl_2d::UniformSampling2D"; }
};

struct Normal2D {
  static std::string Type() { return "muse_mcl_2d::NormalSampling2D"; }
};

struct Density2D {
  static std::string Type() { return "muse_mcl_2d::SampleDensity2D"; }
};

struct Scheduler2D {
  static std::string Type() { return "muse_mcl_2d::Scheduler2D"; }
};


TEST(Test_cslibs_plugins_data, testLoadProviders) {
  ros::NodeHandle nh{"~"};
  cslibs_plugins::LaunchfileParser parser(nh);
  {
    cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
    expected_plugins.emplace("cslibs_plugins_data::LaserProvider",
                             "front_laser");
    expected_plugins.emplace("cslibs_plugins_data::LaserProvider",
                             "rear_laser");
    expected_plugins.emplace("cslibs_plugins_data::Odometry2DProvider",
                             "odometry");

    cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
    parser.getNamesForBaseClass<DataProvider>(plugins);
    EXPECT_EQ(expected_plugins.size(), plugins.size());
    for (auto plugin : plugins) {
      EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
    }
  }
  {
    cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
    expected_plugins.emplace("muse_mcl_2d_gridmaps::BeamModelAMCL",
                             "beam_model_front");
    expected_plugins.emplace("muse_mcl_2d_gridmaps::BeamModelAMCL",
                             "beam_model_rear");

    cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
    parser.getNamesForBaseClass<UpdateModel2D>(plugins);
    EXPECT_EQ(expected_plugins.size(), plugins.size());
    for (auto plugin : plugins) {
      EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
    }
  }
  {
    cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
    expected_plugins.emplace("muse_mcl_2d_gridmaps::BinaryGridmapProvider",
                             "rawseeds");

    cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
    parser.getNamesForBaseClass<MapProvider2D>(plugins);
    EXPECT_EQ(expected_plugins.size(), plugins.size());
    for (auto plugin : plugins) {
      EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
    }
  }
  {
    cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
    expected_plugins.emplace("muse_mcl_2d_odometry::DifferentialDrive",
                             "differential_drive_model");
    cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
    parser.getNamesForBaseClass<PredictionModel2D>(plugins);
    EXPECT_EQ(expected_plugins.size(), plugins.size());
    for (auto plugin : plugins) {
      EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
    }
  }
  {
    cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
    expected_plugins.emplace("muse_mcl_2d::KLD2D", "resampling");
    cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
    parser.getNamesForBaseClass<Resampling2D>(plugins);
    EXPECT_EQ(expected_plugins.size(), plugins.size());
    for (auto plugin : plugins) {
      EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
    }
  }
  {
    cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
    expected_plugins.emplace("muse_mcl_2d::UniformAllMaps2D",
                             "uniform_pose_generation");
    cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
    parser.getNamesForBaseClass<Uniform2D>(plugins);
    EXPECT_EQ(expected_plugins.size(), plugins.size());
    for (auto plugin : plugins) {
      EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
    }
  }
  {
    cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
    expected_plugins.emplace("muse_mcl_2d::Normal2D", "normal_pose_generation");
    cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
    parser.getNamesForBaseClass<Normal2D>(plugins);
    EXPECT_EQ(expected_plugins.size(), plugins.size());
    for (auto plugin : plugins) {
      EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
    }
  }
  {
    cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
    expected_plugins.emplace("muse_mcl_2d::SimpleSampleDensity2D", "density");
    cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
    parser.getNamesForBaseClass<Density2D>(plugins);
    EXPECT_EQ(expected_plugins.size(), plugins.size());
    for (auto plugin : plugins) {
      EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
    }
  }
  {
    cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
    expected_plugins.emplace("muse_mcl_2d::CFS", "scheduler");
    cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
    parser.getNamesForBaseClass<Scheduler2D>(plugins);
    EXPECT_EQ(expected_plugins.size(), plugins.size());
    for (auto plugin : plugins) {
      EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
    }
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_launch_file_parser");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
