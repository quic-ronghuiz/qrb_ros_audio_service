// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_AUDIO_SERVICE__AUDIO_SERVER_HPP_
#define QRB_ROS_AUDIO_SERVICE__AUDIO_SERVER_HPP_

// clang-format off
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>

#include "qrb_ros_audio_service_msgs/srv/audio_request.hpp"
// clang-format on

using AudioService = qrb_ros_audio_service_msgs::srv::AudioRequest;

namespace qrb_ros
{
namespace audio_service
{

class AudioServer : public rclcpp::Node
{
public:
  explicit AudioServer(const rclcpp::NodeOptions & options);

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_{ nullptr };
  rclcpp::Service<AudioService>::SharedPtr server_{ nullptr };
  void service_callback(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<AudioService::Request> request,
      std::shared_ptr<AudioService::Response> response);
  void shutdown_callback();
};

}  // namespace audio_service
}  // namespace qrb_ros

#endif  // QRB_ROS_AUDIO_SERVICE__AUDIO_SERVER_HPP_