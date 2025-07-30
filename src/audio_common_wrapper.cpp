// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_audio_common_lib/audio_common_wrapper.hpp"

#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <set>
#include <stdexcept>

#include "stdio.h"

namespace qrb
{
namespace audio_common_lib
{

uint32_t audio_stream_open(const audio_stream_info & stream_info,
    stream_event_callback_func event_callback)
{
  uint32_t stream_handle = 0;
  stream_handle = (CommonAudioStream::audio_stream_open(stream_info, event_callback));
  return stream_handle;
}

int audio_stream_start(uint32_t stream_handle)
{
  CommonAudioStream * current_stream = CommonAudioStream::get_stream(stream_handle);

  if (current_stream != nullptr)
    return current_stream->start_stream();
  else {
    LOGE("Invailed stream_handle");
  }

  return -EIO;
}

int audio_stream_mute(uint32_t stream_handle, bool mute)
{
  if (stream_handle != 0) {
    CommonAudioStream * current_stream = CommonAudioStream::get_stream(stream_handle);
    if (current_stream != nullptr)
      return current_stream->mute_stream(mute);
  } else {
    LOGE("Invailed stream_handle");
  }

  return -EIO;
}

int audio_stream_stop(uint32_t stream_handle)
{
  if (stream_handle != 0) {
    CommonAudioStream * current_stream = CommonAudioStream::get_stream(stream_handle);
    if (current_stream != nullptr)
      return current_stream->stop_stream();
  } else {
    LOGE("Invailed stream_handle");
  }

  return -EIO;
}

int audio_stream_close(uint32_t stream_handle)
{
  if (stream_handle != 0) {
    CommonAudioStream * current_stream = CommonAudioStream::get_stream(stream_handle);
    int ret = current_stream->close_stream();
    if (!ret) {
      if (current_stream != nullptr)
        delete current_stream;
      return 0;
    }
  } else {
    LOGE("Invailed stream_handle");
  }
  return -EIO;
}

size_t audio_stream_write(uint32_t stream_handle, size_t length, void * buf)
{
  if (stream_handle == 0) {
    LOGE("Invailed stream_handle");
    return -EIO;
  }

  CommonAudioStream * current_stream = CommonAudioStream::get_stream(stream_handle);
  pa_stream * pulse_stream = (pa_stream *)(current_stream->get_stream_handle());

  void * pulse_buf;
  size_t bytes_written;
  size_t bytes_total_written = 0;

  if (current_stream->get_stream_state() == PA_STREAM_READY) {
    for (;;) {
      bytes_written = length;
      if (pa_stream_begin_write(pulse_stream, &pulse_buf, &bytes_written) < 0) {
        LOGE("failed to pa_stream_begin_write, length %d", length);
        return -EIO;
      }
      if (bytes_written > 0) {
        memcpy(pulse_buf, buf, bytes_written);
        pa_stream_write(pulse_stream, pulse_buf, bytes_written, nullptr, 0, PA_SEEK_RELATIVE);
      }
      if (bytes_written >= length)
        break;
      buf += bytes_written;
      length -= bytes_written;
      bytes_total_written += bytes_written;
    }
  } else {
    LOGE("stream not start");
  }

  return bytes_total_written;
}

}  // namespace audio_common_lib
}  // namespace qrb