// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#define LOG_TAG "MessageQuene"

#include "qrb_audio_manager/message_queue.hpp"

namespace qrb
{
namespace audio_manager
{

void MessageQueue::push(const struct as_message & msg)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  m_queue_.push(msg);
  m_cond_.notify_one();
}

bool MessageQueue::pop(struct as_message & msg)
{
  std::unique_lock<std::mutex> lock(m_mutex_);
  while (m_queue_.empty()) {
    // Wait for new messages or stop signals
    m_cond_.wait(lock, [this] { return !m_queue_.empty() || m_stop_; });
    if (m_stop_)
      return false;  // Exit if stop signal is received
  }
  msg = m_queue_.front();
  m_queue_.pop();

  return true;
}

void MessageQueue::stop()
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  m_stop_ = true;
  m_cond_.notify_all();
}

}  // namespace audio_manager
}  // namespace qrb