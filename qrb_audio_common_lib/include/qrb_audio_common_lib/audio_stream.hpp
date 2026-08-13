// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_AUDIO_COMMON_LIB__AUDIO_STREAM_HPP_
#define QRB_AUDIO_COMMON_LIB__AUDIO_STREAM_HPP_

#include <cstdint>
#include <mutex>
#include <unordered_map>

#include "qrb_audio_common_lib/audio_common_wrapper.hpp"
#include "qrb_audio_common_lib/audio_log.hpp"

namespace qrb
{
namespace audio_common_lib
{

class IAudioStream
{
public:
  virtual ~IAudioStream();

  virtual int start_stream() = 0;
  virtual int stop_stream() = 0;
  virtual int close_stream() = 0;
  virtual int mute_stream(bool mute) = 0;
  virtual int pause_stream(bool pause);
  virtual int write_data(const void * buf, size_t size);

  stream_event_callback_func event_cb = nullptr;
  bool pcm_mode = false;
  int32_t repeat_count = 0;

  static uint32_t register_stream(IAudioStream * stream);
  static void unregister_stream(IAudioStream * stream);
  static IAudioStream * get_stream(uint32_t handle);
  static uint32_t get_handle(IAudioStream * stream);

protected:
  uint32_t handle_ = 0;

private:
  static std::unordered_map<uint32_t, IAudioStream *> stream_registry_;
  static std::mutex registry_mutex_;
};

}  // namespace audio_common_lib
}  // namespace qrb

#endif  // QRB_AUDIO_COMMON_LIB__AUDIO_STREAM_HPP_
