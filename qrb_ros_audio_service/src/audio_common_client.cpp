// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <cinttypes>

#include "qrb_ros_audio_service/audio_common_client.hpp"

using namespace std::placeholders;

using qrb::audio_manager::audio_stream_cmd_name;
using qrb::audio_manager::Stream;
using qrb::audio_manager::StreamType;

#define AUDIO_COMMON_CLIENT_NODE_NAME "audio_commmon_client"
#define PLAYBACK_SERVER_NAME "ros_audio_playback"
#define RECORD_SERVER_NAME "ros_audio_capture"

namespace qrb_ros
{
namespace audio_service
{

AudioCommonClient * AudioCommonClient::instance_ = nullptr;

AudioCommonClient::AudioCommonClient(const rclcpp::NodeOptions & options)
  : Node(AUDIO_COMMON_CLIENT_NODE_NAME, options)
{
  playback_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions playback_sub_options;
  playback_sub_options.callback_group = playback_callback_group_;

  record_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions record_sub_options;
  record_sub_options.callback_group = record_callback_group_;

  playback_client_ = rclcpp_action::create_client<AudioCommonAction>(
      this, PLAYBACK_SERVER_NAME, playback_callback_group_);

  record_client_ = rclcpp_action::create_client<AudioCommonAction>(
      this, RECORD_SERVER_NAME, record_callback_group_);

  send_goal_options_.goal_response_callback =
      std::bind(&AudioCommonClient::goal_response_callback, this, _1);
  send_goal_options_.feedback_callback =
      std::bind(&AudioCommonClient::feedback_callback, this, _1, _2);
  send_goal_options_.result_callback = std::bind(&AudioCommonClient::result_callback, this, _1);

  this->declare_parameter<bool>("use_async", true);
  use_async_ = this->get_parameter("use_async").as_bool();
  RCLCPP_INFO(this->get_logger(), "use_async %s", use_async_ ? "true" : "false");

  instance_ = this;
  Stream::register_callback(DOMAIN_ID_AUDIO_COMMON, stream_cb, use_async_);
}

void AudioCommonClient::goal_response_callback(
    const GoalHandleAudioCommonAction::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "goal accepted by server, waiting for result");
  }
}

void AudioCommonClient::feedback_callback(GoalHandleAudioCommonAction::SharedPtr,
    const std::shared_ptr<const AudioCommonAction::Feedback> feedback)
{
  RCLCPP_INFO_STREAM(
      this->get_logger(), "received feedback, timestamp %.2f" << feedback->timestamp);
}

void AudioCommonClient::result_callback(const GoalHandleAudioCommonAction::WrappedResult & result)
{
  auto audio_comm_handle = result.result->stream_handle;
  auto stream_handle = result.result->audio_service_stream_handle;
  auto command = result.result->command;
  auto ret = false;

  char result_str[100];
  snprintf(result_str, sizeof(result_str), "audio_comm_handle 0x%" PRIx64 ", command %s",
      audio_comm_handle, command.c_str());

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      ret = true;
      RCLCPP_INFO(this->get_logger(), "async result: goal was successed, %s", result_str);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "async result: goal was aborted, %s", result_str);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "async result: goal was canceled, %s", result_str);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "async result: unknown result code, %s", result_str);
      return;
  }

  qrb::audio_manager::AudioManager::on_task_completed(
      DOMAIN_ID_AUDIO_COMMON, stream_handle, audio_comm_handle, command, ret);
}

rclcpp_action::ClientGoalHandle<AudioCommonAction>::WrappedResult AudioCommonClient::send_and_wait(
    rclcpp_action::Client<AudioCommonAction>::SharedPtr client,
    std::shared_ptr<qrb_ros_audio_common_msgs::action::AudioCommon_Goal_<std::allocator<void>>>
        goal_msg)
{
  char result_str[100];
  snprintf(result_str, sizeof(result_str), "audio_comm_handle 0x%" PRIx64 ", command %s",
      goal_msg->stream_handle, goal_msg->command.c_str());

  auto goal_handle_future = client->async_send_goal(*goal_msg);
  if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    throw std::runtime_error(std::string("sync result: failed to send goal, ") + result_str);

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle)
    throw std::runtime_error(
        std::string("sync result: goal was rejected by server, ") + result_str);

  auto result_future = client->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    throw std::runtime_error(std::string("sync result: failed to get result, ") + result_str);

  auto result = result_future.get();
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "sync result: goal was successed, %s", result_str);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      throw std::runtime_error(std::string("sync result: goal was aborted, ") + result_str);
    case rclcpp_action::ResultCode::CANCELED:
      throw std::runtime_error(std::string("sync result: goal was canceled, ") + result_str);
    default:
      throw std::runtime_error(std::string("sync result: unknown result code, ") + result_str);
  }

  return result;
}

bool AudioCommonClient::stream_cb(const void * const payload,
    StreamCommand cmd,
    uint32_t & audio_comm_handle)
{
  rclcpp_action::Client<AudioCommonAction>::SharedPtr client = nullptr;
  rclcpp_action::ClientGoalHandle<qrb_ros_audio_common_msgs::action::AudioCommon>::WrappedResult
      result;
  auto cmd_str = audio_stream_cmd_name[cmd];
  const StreamConfigs * configs = static_cast<const StreamConfigs *>(payload);

  auto goal_msg = std::make_shared<AudioCommonAction::Goal>();
  goal_msg->command = cmd_str;
  goal_msg->file_path = configs->source;

  client = (configs->type == StreamType::PLAYBACK) ?
               AudioCommonClient::instance_->playback_client_ :
               AudioCommonClient::instance_->record_client_;

  RCLCPP_INFO(AudioCommonClient::instance_->get_logger(),
      "receive cmd %s, audio_comm_handle 0x%" PRIx64 "", cmd_str.c_str(), audio_comm_handle);

  if (!client->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_ERROR(AudioCommonClient::instance_->get_logger(), "action server not available...");
    return false;
  }

  switch (cmd) {
    case qrb::audio_manager::StreamCommand::OPEN:
      if (configs->type == StreamType::PLAYBACK) {
        goal_msg->audio_info.coding_format = configs->coding_format;
        goal_msg->repeat = configs->repeat;
        goal_msg->volume = configs->volume;
      } else {
        goal_msg->audio_info.sample_rate = configs->sample_rate;
        goal_msg->audio_info.channels = configs->channels;
        goal_msg->audio_info.sample_format = configs->sample_format;
        goal_msg->audio_info.coding_format = configs->coding_format;
        goal_msg->pub_pcm = configs->pub_pcm;
      }
      goal_msg->audio_service_stream_handle = configs->stream_handle;
      break;
    case qrb::audio_manager::StreamCommand::START:
    case qrb::audio_manager::StreamCommand::MUTE:
      goal_msg->mute = configs->mute;
    case qrb::audio_manager::StreamCommand::STOP:
    case qrb::audio_manager::StreamCommand::CLOSE:
      goal_msg->stream_handle = audio_comm_handle;
      break;
    default:
      RCLCPP_ERROR(AudioCommonClient::instance_->get_logger(), "unknown cmd %s", cmd_str.c_str());
      return false;
  }

  try {
    if ((AudioCommonClient::instance_->use_async_) ||
        (cmd == qrb::audio_manager::StreamCommand::START))
      client->async_send_goal(*goal_msg, AudioCommonClient::instance_->send_goal_options_);
    else {
      auto result = AudioCommonClient::instance_->send_and_wait(client, goal_msg);
      if (qrb::audio_manager::StreamCommand::OPEN == cmd)
        audio_comm_handle = result.result->stream_handle;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(AudioCommonClient::instance_->get_logger(), "caught exception: %s", e.what());
    return false;
  }

  return true;
}

}  // namespace audio_service
}  // namespace qrb_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::audio_service::AudioCommonClient)