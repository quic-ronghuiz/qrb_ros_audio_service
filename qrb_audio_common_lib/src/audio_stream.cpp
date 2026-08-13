// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_audio_common_lib/audio_stream.hpp"

#include <cerrno>
#include <random>

namespace qrb
{
namespace audio_common_lib
{

std::unordered_map<uint32_t, IAudioStream *> IAudioStream::stream_registry_;
std::mutex IAudioStream::registry_mutex_;

IAudioStream::~IAudioStream()
{
  unregister_stream(this);
}

int IAudioStream::pause_stream(bool pause)
{
  (void)pause;
  return -ENOTSUP;
}

int IAudioStream::write_data(const void * buf, size_t size)
{
  (void)buf;
  (void)size;
  return -ENOTSUP;
}

uint32_t IAudioStream::register_stream(IAudioStream * stream)
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> dist(1, UINT32_MAX);
  uint32_t handle;
  do {
    handle = dist(gen);
  } while (stream_registry_.find(handle) != stream_registry_.end());
  stream->handle_ = handle;
  stream_registry_[handle] = stream;
  return handle;
}

void IAudioStream::unregister_stream(IAudioStream * stream)
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  for (auto it = stream_registry_.begin(); it != stream_registry_.end(); ++it) {
    if (it->second == stream) {
      stream_registry_.erase(it);
      return;
    }
  }
}

IAudioStream * IAudioStream::get_stream(uint32_t handle)
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  auto it = stream_registry_.find(handle);
  return (it != stream_registry_.end()) ? it->second : nullptr;
}

uint32_t IAudioStream::get_handle(IAudioStream * stream)
{
  std::lock_guard<std::mutex> lock(registry_mutex_);
  for (auto & [handle, s] : stream_registry_) {
    if (s == stream)
      return handle;
  }
  return 0;
}

}  // namespace audio_common_lib
}  // namespace qrb