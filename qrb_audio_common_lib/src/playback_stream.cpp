// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <functional>
#include <stdexcept>

#include "qrb_audio_common_lib/audio_common.hpp"
#include "stdio.h"

#define PLAYBACK_BUFFER_DURATION_MS 300

namespace qrb
{
namespace audio_common_lib
{

const std::multimap<pa_sample_format_t, int> format_map{
  { PA_SAMPLE_S16NE, SF_FORMAT_PCM_S8 },
  { PA_SAMPLE_S16NE, SF_FORMAT_PCM_U8 },
  { PA_SAMPLE_S16NE, SF_FORMAT_PCM_16 },
  { PA_SAMPLE_S24NE, SF_FORMAT_PCM_24 },
  { PA_SAMPLE_S32NE, SF_FORMAT_PCM_32 },
  { PA_SAMPLE_ULAW, SF_FORMAT_ULAW },
  { PA_SAMPLE_ALAW, SF_FORMAT_ALAW },
  { PA_SAMPLE_FLOAT32NE, SF_FORMAT_FLOAT },
  { PA_SAMPLE_FLOAT32NE, SF_FORMAT_DOUBLE },
};

static void stream_drain_complete(pa_stream * stream, int success, void * userdata)
{
  PlaybackStream * current_stream = static_cast<PlaybackStream *>(userdata);

  LOGD("drain complete");

  if ((current_stream->repeat_count == -1) || (current_stream->repeat_count > 0)) {
    std::thread t([current_stream, stream] {
      if ((current_stream->repeat_count == -1) || (current_stream->repeat_count > 0)) {
        size_t writable = 0;

        LOGV("Repeat Count %d", current_stream->repeat_count);
        sf_seek(current_stream->snd_file, 0, SEEK_SET);
        if (current_stream->repeat_count > 0)
          current_stream->repeat_count--;
        pa_usec_t usec;
        pa_stream_get_time(stream, &current_stream->start_usec);

        pa_stream_set_write_callback(stream, PlaybackStream::stream_data_callback, current_stream);
        writable = pa_stream_writable_size(stream);
        PlaybackStream::stream_data_callback(stream, writable, current_stream);
        pa_stream_trigger(stream, NULL, NULL);
      }
    });
    t.detach();
  } else {
    LOGI("EOF\n");
    current_stream->internal_stopstream();
  }
}

PlaybackStream::PlaybackStream(string filepath, std::shared_ptr<pa_sample_spec> sample_spec)
  : CommonAudioStream(filepath, sample_spec)
{
  if (!filepath.empty()) {
    if (sndfile_open()) {
      throw std::runtime_error("Open file failed");
    }
  } else {
    LOGD("no file path passed, PCM data mode\n");
  }

  init_pulse_stream();

  if (!m_file_path.empty()) {
    pa_stream_set_write_callback(get_stream_handle(), PlaybackStream::stream_data_callback, this);
  }
}

int PlaybackStream::start_stream()
{
  pa_buffer_attr buffer_attr;
  pa_stream_flags_t flags =
      static_cast<pa_stream_flags_t>(PA_STREAM_NOFLAGS | PA_STREAM_EARLY_REQUESTS);
  pa_cvolume volume;

  LOGD("enter");

  if (get_stream_handle() == nullptr || get_stream_state() == PA_STREAM_READY) {
    LOGE("stream handle(%p) state(%d) error, start fail", get_stream_handle(), get_stream_state());
    return -EPERM;
  }

  buffer_attr.maxlength = (uint32_t)-1;
  buffer_attr.tlength = (uint32_t)-1;
  buffer_attr.prebuf =
      pa_usec_to_bytes(PLAYBACK_BUFFER_DURATION_MS * PA_USEC_PER_MSEC, m_sample_spec.get());
  buffer_attr.minreq =
      pa_usec_to_bytes(PLAYBACK_BUFFER_DURATION_MS * PA_USEC_PER_MSEC, m_sample_spec.get());
  buffer_attr.fragsize = (uint32_t)-1;

  pa_cvolume_set(&volume, m_sample_spec->channels, PA_VOLUME_NORM * mvolume / STREAM_VOL_MAX);

  if (pa_stream_connect_playback(
          get_stream_handle(), /*device*/ nullptr, &buffer_attr, flags, &volume, nullptr)) {
    LOGE("pa_stream_connect_playback() failed: %s",
        pa_strerror(pa_context_errno(CommonAudioStream::pulse_context_)));
    return -EIO;
  }

  return 0;
}

int PlaybackStream::sndfile_open()
{
  SF_INFO snd_file_info;
  int sf_errno;
  pa_channel_map channel_map;
  bool match_format = false;

  LOGI("opening %s", m_file_path.c_str());
  if ((file_fd = open(m_file_path.c_str(), O_RDONLY, 0666)) < 0) {
    LOGE("open %s failed", m_file_path);
    return -ENOENT;
  }

  snd_file = sf_open_fd(file_fd, SFM_READ, &snd_file_info, 0);
  if (snd_file == nullptr) {
    LOGE("snd file open failed");
    return -SF_ERR_MALFORMED_FILE;
  }

  /* get spec from file */
  sf_errno = sf_command(snd_file, SFC_GET_CURRENT_SF_INFO, &snd_file_info, sizeof(snd_file_info));
  if (sf_errno) {
    LOGE("snd file SFC_GET_CURRENT_SF_INFO failed");
    return -SF_ERR_UNRECOGNISED_FORMAT;
  }

  for (auto it = format_map.begin(); it != format_map.end(); ++it) {
    if (it->second == (snd_file_info.format & SF_FORMAT_SUBMASK)) {
      m_sample_spec->format = it->first;
      match_format = true;
      break;
    }
  }
  if (!match_format) {
    LOGE("snd file format not support");
    return SF_ERR_UNRECOGNISED_FORMAT;
  }

  m_sample_spec->rate = (uint32_t)snd_file_info.samplerate;
  m_sample_spec->channels = (uint8_t)snd_file_info.channels;

  // channel map
  pa_channel_map_init_extend(&channel_map, m_sample_spec->channels, PA_CHANNEL_MAP_DEFAULT);

  /* get spec from file end */
  LOGD("open %s succeed", m_file_path.c_str());

  return SF_ERR_NO_ERROR;
}

size_t PlaybackStream::sndfile_transfer_data(void * data, size_t length)
{
  sf_count_t bytes = 0;
  if (data == nullptr || snd_file == nullptr) {
    LOGE("data or snd_file is nullptr");
    return -SF_ERR_SYSTEM;
  }

  bytes = sf_read_raw(snd_file, data, length);
  return bytes;
}

void PlaybackStream::stream_data_callback(pa_stream * stream, size_t length, void * userdata)
{
  assert(stream);
  assert(length > 0);

  PlaybackStream * current_stream = static_cast<PlaybackStream *>(userdata);

  void * buf_pulse;
  size_t bytes_written;
  size_t bytes_read = 0;

  for (;;) {
    bytes_written = length;
    if ((pa_stream_begin_write(stream, &buf_pulse, &bytes_written)) < 0) {
      LOGE("%s:pa_stream_begin_write failed(%s)", pa_strerror(pa_context_errno(pulse_context_)));
      current_stream->internal_stopstream();
    }

    bytes_read = current_stream->sndfile_transfer_data(buf_pulse, (sf_count_t)bytes_written);

    if (bytes_read > 0)
      if (pa_stream_write(stream, buf_pulse, bytes_written, nullptr, 0, PA_SEEK_RELATIVE)) {
        LOGE("pa_stream_write failed(%s)\n", pa_strerror(pa_context_errno(pulse_context_)));
      } else
        pa_stream_cancel_write(stream);

    if (bytes_read < bytes_written) {
      pa_operation * o;
      pa_stream_set_write_callback(stream, nullptr, nullptr);
      if (!(o = pa_stream_drain(stream, stream_drain_complete, current_stream))) {
        LOGE("pa_stream_drain failed(%s)", pa_strerror(pa_context_errno(pulse_context_)));
      }
      pa_operation_unref(o);
      break;
    }

    /*complete one write*/
    if (bytes_written >= length) {
      break;
    }

    length -= bytes_written;
  }
}

}  // namespace audio_common_lib
}  // namespace qrb
