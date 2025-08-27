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


#ifndef UTILS__CONFIG_HPP_
#define UTILS__CONFIG_HPP_
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include "json.hpp"

#include "utils/logger.hpp"

struct TopicParam
{
  int64_t bitrate;
  std::string input_topic;
  std::string output_topic;
  std::string encoder_name;

  TopicParam(
    const int64_t bitrate, const std::string & input_topic,
    const std::string & output_topic, const std::string & encoder_name)
  {
    this->bitrate = bitrate;
    this->input_topic = input_topic;
    this->output_topic = output_topic;
    this->encoder_name = encoder_name;
  }

  bool operator==(const TopicParam & other) const
  {
    return bitrate == other.bitrate &&
           input_topic == other.input_topic &&
           output_topic == other.output_topic &&
           encoder_name == other.encoder_name;
  }

  bool operator<(const TopicParam & other) const
  {
    if (bitrate != other.bitrate) {
      return bitrate < other.bitrate;
    }
    if (input_topic != other.input_topic) {
      return input_topic < other.input_topic;
    }
    if (output_topic != other.output_topic) {
      return output_topic < other.output_topic;
    }
    return encoder_name < other.encoder_name;
  }
};

class Config
{
public:
  Config()
  {
    current_config_["enable_by_default"] = true;
    current_config_["log_directory"] = "/tmp/coencoder/log/";
    current_config_["log_level"] = "Debug";
    current_config_["topics_param"] = nlohmann::json::array();
  }

  ~Config() = default;

  void load_config(const std::string & config_file)
  {
    if (access(config_file.c_str(), F_OK) == -1) {
      save_config(config_file);
      COLOG_WARN("Config file does not exist");
      return;
    }
    try {
      std::ifstream config(config_file);
      if (!config.is_open()) {
        COLOG_WARN("Failed to open config file");
        return;
      }

      nlohmann::json config_json;
      config >> config_json;
      config.close();

      update_config(config_json);
    } catch (const nlohmann::json::parse_error & e) {
      COLOG_ERROR("Failed to parse config file %s: %s", config_file.c_str(), e.what());
    } catch (const std::exception & e) {
      COLOG_ERROR("Failed to load config file %s: %s", config_file.c_str(), e.what());
    }
  }

  bool update_config(const nlohmann::json & config_json)
  {
    if (!check_config(config_json)) {
      COLOG_ERROR("can't update config, invalid config!");
      throw std::runtime_error("Invalid config");
    }
    if (config_json == current_config_) {
      return false;
    }
    current_config_ = config_json;
    parse_config();
    return true;
  }

  void save_config(const std::string & path) const
  {
    std::ofstream config_file(path);
    if (!config_file.is_open()) {
      COLOG_WARN("failed to open config file while saving config");
      return;
    }

    config_file << current_config_.dump(2);
    config_file.close();
    COLOG_INFO("successfully saved config to file [%s]", path.c_str());
  }

  std::string print_config() const
  {
    return current_config_.dump(2);
  }

  bool enable_by_default_{true};
  std::set<TopicParam> topics_param{};
  std::string log_directory_{"/tmp/coencoder/log/"};
  std::string log_level_{"Debug"};

private:
  static bool check_config(const nlohmann::json & json_obj)
  {
    if (!json_obj.contains("topics_param") || !json_obj["topics_param"].is_array()) {
      COLOG_ERROR("topics_param not found!");
      std::cerr << "topics_param not found!" << std::endl;
      return false;
    }

    const nlohmann::json params = json_obj["topics_param"];
    for (const auto & param : params) {
      if (!param.contains("input") || !param["input"].is_string() ||
        !param.contains("output") || !param["output"].is_string() ||
        !param.contains("bitrate") || !param["bitrate"].is_number())
      {
        return false;
      }
    }
    return true;
  }

  void parse_config()
  {
    if (current_config_.contains("enable_by_default")) {
      enable_by_default_ = current_config_["enable_by_default"].get<bool>();
    }
    if (current_config_.contains("log_directory")) {
      log_directory_ = current_config_["log_directory"].get<std::string>();
    }
    if (current_config_.contains("log_level")) {
      log_level_ = current_config_["log_level"].get<std::string>();
    }

    topics_param.clear();
    for (const auto & param : current_config_["topics_param"]) {
      const std::string encoder_name = param.contains("encoder_name") ?
        param["encoder_name"].get<std::string>() : "libx264";
      topics_param.emplace(
        std::move(
          TopicParam(
            param["bitrate"].get<int64_t>(),
            param["input"].get<std::string>(),
            param["output"].get<std::string>(),
            encoder_name
          )
        )
      );
    }
  }

  nlohmann::json current_config_;
};

#endif  // UTILS__CONFIG_HPP_
