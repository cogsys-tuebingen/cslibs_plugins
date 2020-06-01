#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cslibs_plugins/launch_file_parser.hpp>

struct UpdateModel2D {
  static std::string Type() { return "muse_mcl_2d::UpdateModel2D"; }
};

TEST(Test_cslibs_plugins_data, testLoadProviders) {
  ros::NodeHandle nh{"~"};
  cslibs_plugins::LaunchfileParser parser(nh);
  cslibs_plugins::LaunchfileParser::found_plugin_set_t plugins;
  parser.getNamesForBaseClass<UpdateModel2D>(plugins);

  cslibs_plugins::LaunchfileParser::found_plugin_set_t expected_plugins;
  expected_plugins.emplace("cslibs_plugins_data::LaserProvider", "front_laser");
  expected_plugins.emplace("cslibs_plugins_data::LaserProvider", "rear_laser");
  expected_plugins.emplace("muse_mcl_2d_gridmaps::BeamModelAMCL", "beam_model_front");
  expected_plugins.emplace("muse_mcl_2d_gridmaps::BeamModelAMCL", "beam_model_rear");
  expected_plugins.emplace("muse_mcl_2d_gridmaps::BinaryGridmapProvider", "rawseeds");
  expected_plugins.emplace("muse_mcl_2d_odometry::DifferentialDrive", "differential_drive_model");
  expected_plugins.emplace("cslibs_plugins_data::Odometry2DProvider", "odometry");
  expected_plugins.emplace("muse_mcl_2d::KLD2D", "resampling");
  expected_plugins.emplace("muse_mcl_2d::UniformAllMaps2D", "uniform_pose_generation");
  expected_plugins.emplace("muse_mcl_2d::Normal2D", "normal_pose_generation");
  expected_plugins.emplace("muse_mcl_2d::SimpleSampleDensity2D", "density");
  expected_plugins.emplace("muse_mcl_2d::CFS", "scheduler");

  for (auto plugin : plugins) {
    EXPECT_TRUE(expected_plugins.find(plugin) != expected_plugins.end());
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_launch_file_parser");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
