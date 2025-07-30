// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

// clang-format off
#include <cstdint>

#include "qrb_ros_audio_service/audio_service.hpp"
#include "qrb_audio_manager/audio_manager.hpp"
// clang-format on

using namespace std::placeholders;

using qrb::audio_manager::audio_manager_cmd_name;
using qrb::audio_manager::AudioManager;
using qrb::audio_manager::AudioManagerCommand;

#define AUDIO_SERVER_NAME_NODE_NAME "audio_service"
#define AUDIO_SERVER_NAME "audio_server"

namespace qrb_ros
{
namespace audio_service
{

AudioServer::AudioServer(const rclcpp::NodeOptions & options)
  : Node(AUDIO_SERVER_NAME_NODE_NAME, options)
{
  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  server_ = this->create_service<AudioService>(AUDIO_SERVER_NAME,
      std::bind(&AudioServer::service_callback, this, _1, _2, _3), rmw_qos_profile_services_default,
      callback_group_);

  AudioManager::get_instance();

  rclcpp::on_shutdown(std::bind(&AudioServer::shutdown_callback, this));
}

void AudioServer::service_callback(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<AudioService::Request> request,
    std::shared_ptr<AudioService::Response> response)
{
  (void)request_header;

  RCLCPP_INFO(this->get_logger(), "received service call");

  auto ret = false;

  auto channels = request->audio_info.channels;
  auto sample_rate = request->audio_info.sample_rate;
  auto sample_format = request->audio_info.sample_format;
  auto bitrate = request->audio_info.bitrate;
  auto coding_format = request->audio_info.coding_format;

  auto command = request->command;
  auto type = request->type;
  auto source = request->source;
  auto volume = request->volume;
  auto repeat = request->repeat;
  auto stream_handle_req = request->stream_handle;
  auto pub_pcm = request->pub_pcm;
  auto topic_name = request->topic_name;
  auto mute = request->mute;

  std::string play_mode = "normal";
  uint32_t stream_handle_by_create = 0;
  std::vector<std::string> buildin_sound_name = {};
  auto buildin_sound_count = 0;

  auto am = AudioManager::get_instance();

  switch (static_cast<int>(audio_manager_cmd_name[command])) {
    case static_cast<int>(AudioManagerCommand::PLAY):
      play_mode = "one-touch";
      type = "playback";
    case static_cast<int>(AudioManagerCommand::CREATE):
      if ((type == "playback") || (play_mode == "one-touch")) {
        RCLCPP_INFO(this->get_logger(),
            "source %s, coding_format %s, volume %d, play_mode %s, repate %d topic_name %s",
            source.c_str(), coding_format.c_str(), volume, play_mode.c_str(), repeat,
            topic_name.c_str());
        if (!source.empty() && !topic_name.empty()) {
          RCLCPP_ERROR(this->get_logger(), "either source or topic_name can exist, but not both");
          break;
        }
        if (!source.empty() || !topic_name.empty()) {
          try {
            stream_handle_by_create = am->create_playback_stream(source, sample_rate, channels,
                sample_format, coding_format, volume, play_mode, repeat, topic_name);
            ret = true;
          } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
          }
        }
      } else if (type == "record") {
        RCLCPP_INFO(this->get_logger(),
            "source %s, channels %d, sample_rate %d, sample_format %d, "
            "coding_format %s, pub_pcm %s, topic_name %s",
            source.c_str(), channels, sample_rate, sample_format, coding_format.c_str(),
            (pub_pcm ? "true" : "false"), topic_name.c_str());
        if (!((source.empty() && !pub_pcm) || (0 == channels) || (0 == sample_rate) ||
                (0 == sample_format))) {
          try {
            stream_handle_by_create = am->create_record_stream(
                sample_rate, channels, sample_format, coding_format, source, pub_pcm, topic_name);
            ret = true;
          } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "%s", e.what());
          }
        }
      }
      response->stream_handle = stream_handle_by_create;
      break;
    case static_cast<int>(AudioManagerCommand::START):
      ret = am->start_stream(stream_handle_req);
      break;
    case static_cast<int>(AudioManagerCommand::MUTE):
      ret = am->mute_stream(stream_handle_req, mute);
      break;
    case static_cast<int>(AudioManagerCommand::STOP):
      ret = am->stop_stream(stream_handle_req);
      break;
    case static_cast<int>(AudioManagerCommand::RELEASE):
      ret = am->release_stream(stream_handle_req);
      break;
    case static_cast<int>(AudioManagerCommand::GETBUILDINSOUND):
      for (const auto & pair : am->get_buildin_sounds())
        buildin_sound_name.push_back(pair.first);
      if ((buildin_sound_count = buildin_sound_name.size()) > 0) {
        response->buildin_sound_name.resize(buildin_sound_count);
        for (std::size_t i = 0; i < buildin_sound_name.size(); ++i)
          response->buildin_sound_name[i] = buildin_sound_name[i];
        ret = true;
      }
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "unsupported command: %s", command.c_str());
      break;
  }

  response->success = ret;
}

void AudioServer::shutdown_callback()
{
  AudioManager::get_instance()->clean();
}

}  // namespace audio_service
}  // namespace qrb_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::audio_service::AudioServer)