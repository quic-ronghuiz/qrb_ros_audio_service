// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_AUDIO_COMMON__AUDIO_COMMON_HPP_
#define QRB_ROS_AUDIO_COMMON__AUDIO_COMMON_HPP_

#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <unistd.h>

#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "qrb_audio_common_lib/audio_common.hpp"
#include "qrb_audio_common_lib/audio_common_wrapper.hpp"
#include "qrb_ros_audio_common_msgs/action/audio_common.hpp"
#include "qrb_ros_audio_common_msgs/msg/audio_data.hpp"
#include "qrb_ros_audio_common_msgs/msg/audio_info.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace qrb::audio_common_lib;

namespace qrb_ros
{
namespace audio_common
{

using Stream_Action = qrb_ros_audio_common_msgs::action::AudioCommon;
using GoalHandleStream = rclcpp_action::ServerGoalHandle<Stream_Action>;

struct Stream_Event
{
  intptr_t stream_handle;
  StreamEvent event;
  Stream_Event_Data s_event_data;
};

class AudioCommonNode : public rclcpp::Node
{
public:
  AudioCommonNode(const rclcpp::NodeOptions & options);

private:
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Stream_Action::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleStream> goal_handle);
  void virtual stream_monitor(const std::shared_ptr<GoalHandleStream> goal_handle,
      intptr_t stream_handle);
  void execute(const std::shared_ptr<GoalHandleStream> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleStream> goal_handle);

  audio_stream_info virtual parse_stream_info(const std::shared_ptr<GoalHandleStream> goal_handle);
  void stream_event_cb(StreamEvent s_event, Stream_Event_Data s_event_data, void * stream_handle);
  std::queue<Stream_Event> get_stream_event(void * current_stream_handle);
  void on_audio_data(const qrb_ros_audio_common_msgs::msg::AudioData::SharedPtr msg);
  void create_topic();
  void delete_topic();

  rclcpp_action::Server<Stream_Action>::SharedPtr stream_action_ = nullptr;

  stream_type stream_type_ = StreamPlayback;
  intptr_t pcm_stream_handle_ = 0;
  std::mutex stream_monitor_mtx_;
  std::condition_variable stream_monitor_cv_;

  std::mutex event_mtx_;
  std::condition_variable event_cv_;
  std::queue<Stream_Event> stream_event_queue_;
  string topic_name_ = "AudioData";
  string action_name_ = "ros_audio";
  bool need_topic_ = false;
  StreamEvent current_stream_state_ = StreamEvent::StreamStoped;

  rclcpp::Subscription<qrb_ros_audio_common_msgs::msg::AudioData>::SharedPtr audio_data_sub_ =
      nullptr;
  rclcpp::Publisher<qrb_ros_audio_common_msgs::msg::AudioData>::SharedPtr audio_data_pub_ = nullptr;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace audio_common
}  // namespace qrb_ros

#endif
