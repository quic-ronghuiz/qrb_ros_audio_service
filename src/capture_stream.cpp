// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <functional>
#include <stdexcept>

#include "qrb_audio_common_lib/audio_common.hpp"
#include "stdio.h"

namespace qrb
{
namespace audio_common_lib
{

static const std::multimap<pa_sample_format_t, int> format_map{
  { PA_SAMPLE_U8, SF_FORMAT_PCM_U8 },
  { PA_SAMPLE_S16NE, SF_FORMAT_PCM_16 },
  { PA_SAMPLE_S24NE, SF_FORMAT_PCM_24 },
  { PA_SAMPLE_S32NE, SF_FORMAT_PCM_32 },
  { PA_SAMPLE_ULAW, SF_FORMAT_ULAW },
  { PA_SAMPLE_ALAW, SF_FORMAT_ALAW },
  { PA_SAMPLE_FLOAT32NE, SF_FORMAT_FLOAT },
};

CaptureStream::CaptureStream(string filepath, std::shared_ptr<pa_sample_spec> sample_spec)
  : CommonAudioStream(filepath, sample_spec)
{
  if (!m_file_path.empty()) {
    if (sndfile_open())
      throw std::runtime_error("Open file failed");
  } else {
    LOGD("no file path passed, PCM data mode");
  }

  repeat_count = 0;

  init_pulse_stream();

  pa_stream_set_read_callback(get_stream_handle(), CaptureStream::stream_data_callback, this);
}

int CaptureStream::start_stream()
{
  pa_buffer_attr buffer_attr;
  pa_stream_flags_t flags;
  pa_cvolume volume;
  pa_stream * stream = get_stream_handle();

  LOGD("enter");

  if (stream == nullptr || get_stream_state() != PA_STREAM_UNCONNECTED) {
    LOGE("stream handle(%p) state(%d) error, start fail", stream, get_stream_state());
    return -EPERM;
  }

  buffer_attr.maxlength = (uint32_t)-1;
  buffer_attr.prebuf = (uint32_t)-1;

  buffer_attr.fragsize = buffer_attr.tlength = (uint32_t)-1;
  buffer_attr.minreq = (uint32_t)-1;

  pa_cvolume_set(&volume, m_sample_spec->channels, PA_VOLUME_NORM * mvolume / STREAM_VOL_MAX);

  if (pa_stream_connect_record(stream, /*device*/ nullptr, &buffer_attr, flags)) {
    LOGE("pa_stream_connect_playback() failed: %s",
        pa_strerror(pa_context_errno(CommonAudioStream::pulse_context_)));
    return -EIO;
  }

  pa_threaded_mainloop_lock(CommonAudioStream::pulse_mainloop_);
  while (get_stream_state() != PA_STREAM_READY) {
    pa_threaded_mainloop_wait(CommonAudioStream::pulse_mainloop_);
  }
  pa_threaded_mainloop_unlock(CommonAudioStream::pulse_mainloop_);

  uint32_t stream_index = pa_stream_get_index(stream);

  pa_context_set_source_output_volume(
      CommonAudioStream::pulse_context_, stream_index, &volume, NULL, NULL);

  return 0;
}

int CaptureStream::sndfile_open()
{
  int fd;
  SF_INFO snd_file_info;
  bool match_format = false;

  // Generate SFI info based on the passed parameters
  snd_file_info.samplerate = m_sample_spec->rate;
  snd_file_info.channels = m_sample_spec->channels;

  for (auto it = format_map.begin(); it != format_map.end(); ++it) {
    if (it->first == (m_sample_spec->format)) {
      snd_file_info.format = it->second;
      match_format = true;
      break;
    }
  }

  if (!match_format) {
    LOGE("snd file format not support");
    return SF_ERR_UNRECOGNISED_FORMAT;
  }

  snd_file_info.format |= SF_FORMAT_WAV;

  LOGI("opening %s", m_file_path.c_str());
  if ((fd = open(m_file_path.c_str(), O_WRONLY | O_TRUNC | O_CREAT, 0666)) < 0) {
    LOGE("open %s failed", m_file_path);
    return -ENOMEM;
  }

  snd_file = sf_open_fd(fd, SFM_WRITE, &snd_file_info, 0);
  if (snd_file == nullptr) {
    LOGE("snd file open failed");
    return -SF_ERR_MALFORMED_FILE;
  }

  LOGD("open %s succeed", m_file_path.c_str());

  return SF_ERR_NO_ERROR;
}

size_t CaptureStream::sndfile_transfer_data(const void * data, size_t length)
{
  sf_count_t bytes = 0;
  if (data == nullptr || snd_file == nullptr) {
    LOGE("data or snd_file is nullptr");
    return -SF_ERR_SYSTEM;
  }

  bytes = sf_write_raw(snd_file, data, length);
  return bytes;
}

void CaptureStream::stream_data_callback(pa_stream * stream, size_t length, void * userdata)
{
  assert(stream);
  assert(length > 0);

  CaptureStream * current_stream = static_cast<CaptureStream *>(userdata);

  while (pa_stream_readable_size(stream) > 0) {
    sf_count_t bytes;
    const void * data;

    if (pa_stream_peek(stream, &data, &length) < 0) {
      LOGE("pa_stream_peek() failed(%s)",
          pa_strerror(pa_context_errno(CommonAudioStream::pulse_context_)));
      current_stream->internal_stopstream();
    }

    assert(length > 0);

    if (current_stream->snd_file)
      bytes = current_stream->sndfile_transfer_data(data, length);
    if (current_stream->pcm_mode_) {
      Stream_Event_Data s_event_data;
      uint32_t stream_handle = get_common_stream_handle(current_stream);

      s_event_data.data.data_ptr = (intptr_t)data;
      s_event_data.data.data_size = length;
      current_stream->event_cb(StreamEvent::StreamData, s_event_data, (void *)stream_handle);
      bytes = length;
    }

    if (bytes < (sf_count_t)length) {
      LOGE("write to file fail");
      current_stream->internal_stopstream();
    }

    pa_stream_drop(stream);
  }
}

}  // namespace audio_common_lib
}  // namespace qrb