// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_AUDIO_MANAGER__MESSAGE_QUEUE_HPP_
#define QRB_AUDIO_MANAGER__MESSAGE_QUEUE_HPP_

#include <condition_variable>
#include <queue>

#include "qrb_audio_manager/stream.hpp"

namespace qrb
{
namespace audio_manager
{

struct as_message
{
  uint32_t stream_handle;
  StreamCommand command;
};

class MessageQueue
{
public:
  void push(const struct as_message & msg);  // Add messages to the queue
  bool pop(struct as_message & msg);         // Get message from queue
  void stop();                               // Send stop signal

private:
  std::queue<as_message> m_queue_;  // Queues for storing messages
  std::mutex m_mutex_;
  std::condition_variable m_cond_;
  bool m_stop_ = false;
};

}  // namespace audio_manager
}  // namespace qrb

#endif  // QRB_AUDIO_MANAGER__MESSAGE_QUEUE_HPP_