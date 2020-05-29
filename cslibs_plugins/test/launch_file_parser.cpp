#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cslibs_plugins/launch_file_parser.hpp>

struct UpdateModel2D {
  static std::string Type() {return "muse_mcl_2d::UpdateModel2D"; }
};


TEST(Test_cslibs_plugins_data, testLoadProviders) {
  ros::NodeHandle nh{"~"};
  cslibs_plugins::LaunchfileParser parser(nh);
  std::vector<cslibs_plugins::LaunchfileParser::FoundPlugin> plugins;
  parser.getNamesForBaseClass<UpdateModel2D>(plugins);

  for(auto plugin : plugins) {
    std::cerr << plugin.name << " " << plugin.class_name << std::endl;
  }

}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "test_launch_file_parser");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
