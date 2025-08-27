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

#include <memory>
#include <string>
#include "ros2/coencoder.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string config_file;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--config-file" && i + 1 < argc) {
      config_file = argv[i + 1];
      break;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("MAIN"), "config_path: %s", config_file.c_str());

  try {
    auto node = std::make_shared<CoEncoder>(config_file);
    RCLCPP_INFO(rclcpp::get_logger("MAIN"), "CoEncoder MultiThreadedExecutor SPIN!");

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MAIN"), "Exception in main: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
