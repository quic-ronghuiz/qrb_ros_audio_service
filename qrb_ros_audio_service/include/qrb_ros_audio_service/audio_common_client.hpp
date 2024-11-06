// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_DEMOS_ACTION__ACTION_CLIENT_HPP_
#define QRB_ROS_DEMOS_ACTION__ACTION_CLIENT_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "qrb_audio_manager/audio_manager.hpp"
#include "qrb_ros_audio_common_msgs/action/audio_common.hpp"

using AudioCommonAction = qrb_ros_audio_common_msgs::action::AudioCommon;
using GoalHandleAudioCommonAction = rclcpp_action::ClientGoalHandle<AudioCommonAction>;

using qrb::audio_manager::StreamCommand;
using qrb::audio_manager::StreamConfigs;

namespace qrb_ros
{
namespace audio_service
{

class AudioCommonClient : public rclcpp::Node
{
public:
  explicit AudioCommonClient(const rclcpp::NodeOptions & options);

private:
  static AudioCommonClient * instance_;
  rclcpp_action::Client<AudioCommonAction>::SharedPtr playback_client_;
  rclcpp_action::Client<AudioCommonAction>::SharedPtr record_client_;
  rclcpp_action::Client<AudioCommonAction>::SendGoalOptions send_goal_options_;
  rclcpp::CallbackGroup::SharedPtr playback_callback_group_{ nullptr };
  rclcpp::CallbackGroup::SharedPtr record_callback_group_{ nullptr };
  bool use_async_{ true };
  rclcpp_action::ClientGoalHandle<AudioCommonAction>::WrappedResult send_and_wait(
      rclcpp_action::Client<AudioCommonAction>::SharedPtr client,
      std::shared_ptr<qrb_ros_audio_common_msgs::action::AudioCommon_Goal_<std::allocator<void>>>
          msg);
  void goal_response_callback(const GoalHandleAudioCommonAction::SharedPtr & goal_handle);
  void feedback_callback(GoalHandleAudioCommonAction::SharedPtr,
      const std::shared_ptr<const AudioCommonAction::Feedback> feedback);
  void result_callback(const GoalHandleAudioCommonAction::WrappedResult & result);
  static bool stream_cb(const void * const payload,
      StreamCommand cmd,
      uint32_t & audio_comm_handle);
};

}  // namespace audio_service
}  // namespace qrb_ros

#endif  // QRB_ROS_DEMOS_ACTION__ACTION_CLIENT_HPP_