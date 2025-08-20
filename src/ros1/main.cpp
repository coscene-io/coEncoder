// Copyright 2024 coScene
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <ros1/coencoder.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "coencoder");
  std::string config_file;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--config-file" && i + 1 < argc) {
      config_file = argv[i + 1];
      break;
    }
  }
  ROS_INFO("config_path: %s", config_file.c_str());

  try {
    CoEncoder node(config_file);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception & e) {
    ROS_ERROR("Exception in main: %s", e.what());
  }
  return 0;
}
