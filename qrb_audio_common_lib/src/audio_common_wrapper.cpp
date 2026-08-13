// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "qrb_audio_common_lib/alsa_stream.hpp"
#include "qrb_audio_common_lib/audio_stream.hpp"
#include "qrb_audio_common_lib/pulse_stream.hpp"

namespace qrb
{
namespace audio_common_lib
{

static AudioBackend g_backend = AudioBackend::ALSA;
static std::string g_alsa_playback_device;
static std::string g_alsa_capture_device;

static bool is_pulseaudio_available()
{
  const char * runtime_dir = getenv("XDG_RUNTIME_DIR");
  bool result = false;

  if (runtime_dir) {
    std::string path = std::string(runtime_dir) + "/pulse/native";
    if (access(path.c_str(), F_OK) == 0) {
      result = true;
    }
  }

  if (result == false) {
    uid_t uid = getuid();
    std::string path = "/run/user/" + std::to_string(uid) + "/pulse/native";
    if (access(path.c_str(), F_OK) == 0)
      result = true;
  }

  if (result == false) {
    if (access("/run/pulse/native", F_OK) == 0)
      result = true;
  }

  return result;
}

AudioBackend detect_audio_backend()
{
  if (is_pulseaudio_available()) {
    g_backend = AudioBackend::PULSEAUDIO;
    LOGI("Audio backend: PulseAudio");
    return g_backend;
  }
  LOGI("PulseAudio unavailable, checking ALSA...");

  if (AlsaCommonStream::detect_alsa_available(g_alsa_playback_device, g_alsa_capture_device)) {
    g_backend = AudioBackend::ALSA;
    LOGI("Audio backend: ALSA (playback='%s', capture='%s')", g_alsa_playback_device.c_str(),
        g_alsa_capture_device.c_str());
    return g_backend;
  }

  LOGE("No audio backend detected");
  g_backend = AudioBackend::INVALID;
  return g_backend;
}

uint32_t audio_stream_open(const AudioStreamInfo & stream_info,
    stream_event_callback_func event_callback)
{
  if (g_backend == AudioBackend::ALSA) {
    AudioStreamInfo info = stream_info;
    if (info.device.empty()) {
      if (info.type == StreamType::Playback && !g_alsa_playback_device.empty()) {
        info.device = g_alsa_playback_device;
      } else if (info.type == StreamType::Capture && !g_alsa_capture_device.empty()) {
        info.device = g_alsa_capture_device;
      }
    }
    return AlsaCommonStream::audio_stream_open(info, event_callback);
  } else if (g_backend == AudioBackend::PULSEAUDIO) {
    return PulseCommonStream::audio_stream_open(stream_info, event_callback);
  } else {
    throw std::runtime_error("No supported backend is available");
  }
}

int audio_stream_start(uint32_t stream_handle)
{
  if (stream_handle == 0) {
    LOGE("audio_stream_start: Invalid stream_handle");
    return -EIO;
  }
  IAudioStream * stream = IAudioStream::get_stream(stream_handle);
  if (!stream) {
    LOGE("audio_stream_start: Invalid stream");
    return -EIO;
  }
  return stream->start_stream();
}

int audio_stream_mute(uint32_t stream_handle, bool mute)
{
  if (stream_handle == 0) {
    LOGE("audio_stream_mute: Invalid stream_handle");
    return -EIO;
  }
  IAudioStream * stream = IAudioStream::get_stream(stream_handle);
  if (!stream) {
    LOGE("audio_stream_mute: Invalid stream");
    return -EIO;
  }
  return stream->mute_stream(mute);
}

int audio_stream_stop(uint32_t stream_handle)
{
  if (stream_handle == 0) {
    LOGE("audio_stream_stop: Invalid stream_handle");
    return -EIO;
  }
  IAudioStream * stream = IAudioStream::get_stream(stream_handle);
  if (!stream) {
    LOGE("audio_stream_stop: Invalid stream");
    return -EIO;
  }
  return stream->stop_stream();
}

int audio_stream_close(uint32_t stream_handle)
{
  if (stream_handle == 0) {
    LOGE("audio_stream_close: Invalid stream_handle");
    return -EIO;
  }
  IAudioStream * stream = IAudioStream::get_stream(stream_handle);
  if (!stream) {
    LOGE("audio_stream_close: Invalid stream");
    return -EIO;
  }
  int ret = stream->close_stream();
  delete stream;  // ~IAudioStream() calls unregister_stream()
  return ret;
}

int audio_stream_write(uint32_t stream_handle, const void * buf, size_t length)
{
  if (stream_handle == 0) {
    LOGE("audio_stream_write: Invalid stream_handle");
    return -EIO;
  }
  IAudioStream * stream = IAudioStream::get_stream(stream_handle);
  if (!stream) {
    LOGE("audio_stream_write: Invalid stream");
    return -EIO;
  }
  return stream->write_data(buf, length);
}

}  // namespace audio_common_lib
}  // namespace qrb
