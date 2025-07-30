// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_audio_common/audio_common.hpp"

#include <rclcpp/rclcpp.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

#define DEFAULT_VOLUME 50

using namespace qrb::audio_common_lib;

namespace qrb_ros
{
namespace audio_common
{

using Stream_Action = qrb_ros_audio_common_msgs::action::AudioCommon;
using GoalHandleStream = rclcpp_action::ServerGoalHandle<Stream_Action>;

void AudioCommonNode::stream_event_cb(StreamEvent s_event,
    Stream_Event_Data s_event_data,
    void * stream_handle)
{
  Stream_Event event;
  event.event = s_event;
  event.stream_handle = (intptr_t)stream_handle;
  event.s_event_data = s_event_data;

  stream_event_queue_.push(event);
  event_cv_.notify_all();
}

std::queue<Stream_Event> AudioCommonNode::get_stream_event(void * current_stream_handle)
{
  std::unique_lock<std::mutex> lock(event_mtx_);
  std::queue<Stream_Event> matchedEvents;
  std::queue<Stream_Event> temp;

  intptr_t current_handle = (intptr_t)current_stream_handle;

  event_cv_.wait(lock);

  while (!stream_event_queue_.empty()) {
    Stream_Event event = stream_event_queue_.front();
    stream_event_queue_.pop();
    if (event.stream_handle == current_handle) {
      matchedEvents.push(event);
    } else {
      temp.push(event);
    }
  }

  // push back not match events
  std::swap(stream_event_queue_, temp);

  lock.unlock();

  return matchedEvents;
}

AudioCommonNode::AudioCommonNode(const rclcpp::NodeOptions & options)
  : Node("audio_base_node", options)
{
  using namespace std::placeholders;

  this->declare_parameter<std::string>("Stream_type", "default_stream");
  this->declare_parameter<std::string>("action_name", "default_action");
  this->declare_parameter<std::string>("topic_name", "default_topic");

  auto parameter_stream_type = this->get_parameter("Stream_type").as_string();
  auto parameter_action_name = this->get_parameter("action_name").as_string();
  auto parameter_topic_name = this->get_parameter("topic_name").as_string();

  if (parameter_stream_type == "playback")
    stream_type_ = StreamPlayback;
  else if (parameter_stream_type == "capture")
    stream_type_ = StreamCapture;

  if (!parameter_topic_name.empty())
    topic_name_ = parameter_topic_name;
  if (!parameter_action_name.empty())
    action_name_ = parameter_action_name;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "stream type: %d\n", stream_type_);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "action name: %s\n", action_name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pcm topic name: %s\n", topic_name_.c_str());

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  stream_action_ = rclcpp_action::create_server<Stream_Action>(this, action_name_.c_str(),
      std::bind(&AudioCommonNode::handle_goal, this, _1, _2),
      std::bind(&AudioCommonNode::handle_cancel, this, _1),
      std::bind(&AudioCommonNode::handle_accepted, this, _1),
      rcl_action_server_get_default_options(), callback_group_);
}

rclcpp_action::GoalResponse AudioCommonNode::handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Stream_Action::Goal> goal)
{
  RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"), "Received goal request with order %s\n", goal->command.c_str());
  (void)uuid;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AudioCommonNode::handle_cancel(
    const std::shared_ptr<GoalHandleStream> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  std::unique_lock<std::mutex> lock(stream_monitor_mtx_);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to cancel %s", goal->command.c_str());
  if (goal->command != "start") {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "only start can cencel");
    return rclcpp_action::CancelResponse::REJECT;
  }

  audio_stream_stop(goal->stream_handle);

  if (current_stream_state_ == StreamEvent::StreamStart)
    stream_monitor_cv_.wait(lock);
  delete_topic();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "handle cancel %s complete", goal->command.c_str());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AudioCommonNode::execute(const std::shared_ptr<GoalHandleStream> goal_handle)
{
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<Stream_Action::Result>();
  int ret = 0;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal command = %s, stream_handle = %ld\n",
      goal->command.c_str(), goal->stream_handle);

  if (goal->command == "open") {
    audio_stream_info stream_info = parse_stream_info(goal_handle);
    auto callback = std::bind(&AudioCommonNode::stream_event_cb, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3);

    result->stream_handle = audio_stream_open(stream_info, callback);
    if (!result->stream_handle) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Stream Open failed\n");
      ret = -EIO;
    } else {
      result->audio_service_stream_handle = goal->audio_service_stream_handle;
      if (need_topic_)
        pcm_stream_handle_ = result->stream_handle;
    }
  } else if (goal->stream_handle == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "stream_handle is need for command %s\n",
        goal->command.c_str());
    ret = -EIO;
  } else if (goal->command == "start") {
    ret = audio_stream_start(goal->stream_handle);
    if (!ret) {
      result->stream_handle = goal->stream_handle;
      std::thread stream_monitor_thread = std::thread(
          &AudioCommonNode::stream_monitor, this, std::ref(goal_handle), result->stream_handle);
      create_topic();
      stream_monitor_thread.join();
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "goal (%s) canceled\n", goal->command.c_str());
        goal_handle->canceled(result);
        return;
      }
      if (current_stream_state_ == StreamEvent::StreamAbort)
        ret = -EIO;
    }
  } else if (goal->command == "stop") {
    result->stream_handle = goal->stream_handle;
    ret = audio_stream_stop(goal->stream_handle);
    delete_topic();
  } else if (goal->command == "close") {
    result->stream_handle = goal->stream_handle;
    ret = audio_stream_close(goal->stream_handle);
    pcm_stream_handle_ = 0;
    need_topic_ = false;
  } else if (goal->command == "mute") {
    result->stream_handle = goal->stream_handle;
    ret = audio_stream_mute(goal->stream_handle, goal->mute);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid command %s\n", goal->command.c_str());
    ret = -EINVAL;
  }

  result->command = goal->command;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "goal execute exit with ret(%d)", ret);
  if (ret) {
    goal_handle->abort(result);
  } else if (!ret) {
    goal_handle->succeed(result);
  }
}

void AudioCommonNode::handle_accepted(const std::shared_ptr<GoalHandleStream> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "handle_accepted\n");
  using namespace std::placeholders;
  std::thread{ std::bind(&AudioCommonNode::execute, this, _1), goal_handle }.detach();
}

audio_stream_info AudioCommonNode::parse_stream_info(
    const std::shared_ptr<GoalHandleStream> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  audio_stream_info stream_info;

  if (goal->audio_info.sample_rate)
    stream_info.rate = goal->audio_info.sample_rate;

  if (goal->audio_info.channels)
    stream_info.channels = goal->audio_info.channels;

  if (goal->audio_info.sample_format)
    stream_info.format = goal->audio_info.sample_format;

  if (goal->volume == 0)
    stream_info.volume = DEFAULT_VOLUME;
  else
    stream_info.volume = goal->volume;

  if (goal->need_timestamp)
    stream_info.need_timestamp = goal->need_timestamp;

  if (goal->repeat)
    stream_info.repeat = goal->repeat;

  if (!(goal->file_path.empty()))
    stream_info.file_path = goal->file_path;

  if (goal->pub_pcm || (!goal->topic_name.empty())) {
    stream_info.pcm_mode = true;
    need_topic_ = true;
    if (!goal->topic_name.empty())
      topic_name_ = goal->topic_name;
    else
      topic_name_ = this->get_parameter("topic_name").as_string();
  }

  stream_info.type = stream_type_;
  return stream_info;
}

void AudioCommonNode::on_audio_data(const qrb_ros_audio_common_msgs::msg::AudioData::SharedPtr msg)
{
  if (msg->data.size() > 0)
    audio_stream_write(pcm_stream_handle_, msg->data.size(), &msg->data[0]);
}

void AudioCommonNode::stream_monitor(const std::shared_ptr<GoalHandleStream> goal_handle,
    intptr_t stream_handle)
{
  auto feedback = std::make_shared<Stream_Action::Feedback>();

  for (;;) {
    std::queue<Stream_Event> events = get_stream_event((void *)stream_handle);
    while (!events.empty()) {
      Stream_Event event = events.front();
      events.pop();
      switch (event.event) {
        case StreamEvent::StreamAbort:
        case StreamEvent::StreamStoped:
          current_stream_state_ = event.event;
          stream_monitor_cv_.notify_all();
          return;
        case StreamEvent::StreamStart:
          current_stream_state_ = event.event;
          break;
        case StreamEvent::StreamData: {
          qrb_ros_audio_common_msgs::msg::AudioData audio_data_msg;
          audio_data_msg.stream_handle = event.stream_handle;
          audio_data_msg.data.resize(event.s_event_data.data.data_size);
          memcpy(&audio_data_msg.data[0], (void *)event.s_event_data.data.data_ptr,
              event.s_event_data.data.data_size);
          audio_data_pub_->publish(audio_data_msg);
          break;
        }
        case StreamEvent::StreamTimestamp:
          feedback->timestamp = (float)event.s_event_data.usec / 1000000;
          feedback->stream_handle = stream_handle;
          goal_handle->publish_feedback(feedback);
          break;
      }
    }
  }
}

void AudioCommonNode::create_topic()
{
  if (need_topic_) {
    if (stream_type_ == StreamPlayback) {
      rclcpp::SubscriptionOptions sub_options;
      sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
      audio_data_sub_ = this->create_subscription<qrb_ros_audio_common_msgs::msg::AudioData>(
          topic_name_.c_str(), 10,
          std::bind(&AudioCommonNode::on_audio_data, this, std::placeholders::_1), sub_options);
    } else if (stream_type_ == StreamCapture) {
      rclcpp::PublisherOptions pub_options;
      pub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
      audio_data_pub_ = this->create_publisher<qrb_ros_audio_common_msgs::msg::AudioData>(
          topic_name_.c_str(), 10, pub_options);
    }
  }
}

void AudioCommonNode::delete_topic()
{
  if (audio_data_sub_)
    if (stream_type_ == StreamPlayback)
      audio_data_sub_ = nullptr;
    else if (stream_type_ == StreamCapture)
      audio_data_pub_ = nullptr;
}

}  // namespace audio_common
}  // namespace qrb_ros

RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::audio_common::AudioCommonNode)
