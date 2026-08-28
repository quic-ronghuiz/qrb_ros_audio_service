// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_audio_common_lib/alsa_stream.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <vector>

using namespace std;

namespace qrb
{
namespace audio_common_lib
{

static void apply_volume(void * buf, size_t bytes, uint8_t format, uint8_t volume)
{
  if (volume >= 100)
    return;

  const float scale = volume / 100.0f;

  switch (format) {
    case 8: {
      uint8_t * s = static_cast<uint8_t *>(buf);
      for (size_t i = 0; i < bytes; i++)
        s[i] = static_cast<uint8_t>(128 + (s[i] - 128) * scale);
      break;
    }
    case 16: {
      int16_t * s = static_cast<int16_t *>(buf);
      size_t n = bytes / 2;
      for (size_t i = 0; i < n; i++)
        s[i] = static_cast<int16_t>(s[i] * scale);
      break;
    }
    case 24: {
      uint8_t * s = static_cast<uint8_t *>(buf);
      size_t n = bytes / 3;
      for (size_t i = 0; i < n; i++) {
        int32_t v = (s[i * 3 + 2] << 16) | (s[i * 3 + 1] << 8) | s[i * 3];
        if (v & 0x800000)
          v |= 0xFF000000;
        v = static_cast<int32_t>(v * scale);
        s[i * 3] = v & 0xFF;
        s[i * 3 + 1] = (v >> 8) & 0xFF;
        s[i * 3 + 2] = (v >> 16) & 0xFF;
      }
      break;
    }
    case 32: {
      int32_t * s = static_cast<int32_t *>(buf);
      size_t n = bytes / 4;
      for (size_t i = 0; i < n; i++)
        s[i] = static_cast<int32_t>(s[i] * scale);
      break;
    }
    default:
      break;
  }
}

AlsaCommonStream::AlsaCommonStream(const AudioStreamInfo & info, stream_event_callback_func cb)
  : stream_info_(info)
{
  event_cb = cb;
  pcm_mode = info.pcm_mode;
  repeat_count = info.repeat;
  volume_ = (info.volume > 0 && info.volume <= 100) ? info.volume : 100;
}

AlsaCommonStream::~AlsaCommonStream()
{
  if (pcm_handle_) {
    snd_pcm_close(pcm_handle_);
    pcm_handle_ = nullptr;
  }
}

uint32_t AlsaCommonStream::audio_stream_open(const AudioStreamInfo & info,
    stream_event_callback_func event_cb)
{
  AlsaCommonStream * stream = nullptr;
  try {
    if (info.type == StreamType::Playback) {
      stream = new AlsaPlaybackStream(info, event_cb);
    } else if (info.type == StreamType::Capture) {
      stream = new AlsaCaptureStream(info, event_cb);
    } else {
      return 0;
    }
  } catch (const std::exception & e) {
    LOGE("%s", e.what());
    return 0;
  }

  return IAudioStream::register_stream(stream);
}

bool AlsaCommonStream::detect_alsa_available(std::string & playback_device,
    std::string & capture_device)
{
  bool playback_ok = false;
  bool capture_ok = false;

  for (const auto * dev : kAlsaPlaybackProbeDevices) {
    snd_pcm_t * pcm = nullptr;
    int err = snd_pcm_open(&pcm, dev, SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK);
    if (err >= 0) {
      snd_pcm_close(pcm);
      playback_device = dev;
      playback_ok = true;
      LOGI("ALSA playback probe succeeded on device '%s'", dev);
      break;
    }
    LOGD("ALSA playback probe failed on device '%s' (%s)", dev, snd_strerror(err));
  }

  for (const auto * dev : kAlsaCaptureProbeDevices) {
    snd_pcm_t * pcm = nullptr;
    int err = snd_pcm_open(&pcm, dev, SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK);
    if (err >= 0) {
      snd_pcm_close(pcm);
      capture_device = dev;
      capture_ok = true;
      LOGI("ALSA capture probe succeeded on device '%s'", dev);
      break;
    }
    LOGD("ALSA capture probe failed on device '%s' (%s)", dev, snd_strerror(err));
  }

  if (!playback_ok)
    LOGI("No ALSA playback device is available");
  if (!capture_ok)
    LOGI("No ALSA capture device is available");

  return playback_ok || capture_ok;
}

snd_pcm_format_t AlsaCommonStream::get_alsa_format() const
{
  switch (stream_info_.format) {
    case 8:
      return SND_PCM_FORMAT_U8;
    case 16:
      return SND_PCM_FORMAT_S16_LE;
    case 24:
      return SND_PCM_FORMAT_S24_3LE;
    case 32:
      return SND_PCM_FORMAT_S32_LE;
    default:
      return SND_PCM_FORMAT_S16_LE;
  }
}

int AlsaCommonStream::open_pcm(snd_pcm_stream_t direction)
{
  std::vector<std::string> candidates;
  if (!stream_info_.device.empty()) {
    candidates.push_back(stream_info_.device);
  } else {
    const auto & probe_list = (direction == SND_PCM_STREAM_PLAYBACK) ? kAlsaPlaybackProbeDevices :
                                                                       kAlsaCaptureProbeDevices;
    candidates.assign(probe_list.begin(), probe_list.end());
  }

  int err = -ENODEV;
  for (const auto & dev : candidates) {
    err = snd_pcm_open(&pcm_handle_, dev.c_str(), direction, 0);
    if (err >= 0) {
      resolved_device_ = dev;
      LOGD("opened device '%s'", resolved_device_.c_str());
      return 0;
    }
    LOGI("device '%s' unavailable (%s)", dev.c_str(), snd_strerror(err));
  }

  LOGE("failed to open any candidate device");
  return err;
}

int AlsaCommonStream::set_hw_params(snd_pcm_uframes_t & period_frames,
    snd_pcm_uframes_t & buffer_frames)
{
  snd_pcm_hw_params_t * params;
  snd_pcm_hw_params_alloca(&params);

  int err = snd_pcm_hw_params_any(pcm_handle_, params);
  if (err < 0) {
    LOGE("snd_pcm_hw_params_any: %s", snd_strerror(err));
    goto exit;
  }

  err = snd_pcm_hw_params_set_access(pcm_handle_, params, SND_PCM_ACCESS_RW_INTERLEAVED);
  if (err < 0) {
    LOGE("snd_pcm_hw_params_set_access: %s", snd_strerror(err));
    goto exit;
  }

  err = snd_pcm_hw_params_set_format(pcm_handle_, params, get_alsa_format());
  if (err < 0) {
    LOGE("snd_pcm_hw_params_set_format: %s", snd_strerror(err));
    goto exit;
  }

  err = snd_pcm_hw_params_set_rate_near(pcm_handle_, params, &stream_info_.rate, nullptr);
  if (err < 0) {
    LOGE("snd_pcm_hw_params_set_rate_near: %s", snd_strerror(err));
    goto exit;
  }

  err = snd_pcm_hw_params_set_channels(pcm_handle_, params, stream_info_.channels);
  if (err < 0) {
    LOGE("snd_pcm_hw_params_set_channels: %s", snd_strerror(err));
    goto exit;
  }

  err = snd_pcm_hw_params_set_period_size_near(pcm_handle_, params, &period_frames, nullptr);
  if (err < 0) {
    LOGE("snd_pcm_hw_params_set_period_size_near: %s", snd_strerror(err));
    goto exit;
  }

  err = snd_pcm_hw_params_set_buffer_size_near(pcm_handle_, params, &buffer_frames);
  if (err < 0) {
    LOGE("snd_pcm_hw_params_set_buffer_size_near: %s", snd_strerror(err));
    goto exit;
  }

  err = snd_pcm_hw_params(pcm_handle_, params);
  if (err < 0) {
    LOGE("snd_pcm_hw_params: %s", snd_strerror(err));
    goto exit;
  }

  err = snd_pcm_prepare(pcm_handle_);
  if (err < 0) {
    LOGE("snd_pcm_prepare: %s", snd_strerror(err));
    goto exit;
  }

  period_frames_ = period_frames;

exit:
  return err;
}

int AlsaCommonStream::stop_stream()
{
  running_ = false;
  if (pcm_handle_) {
    snd_pcm_drop(pcm_handle_);
  }
  if (worker_thread_.joinable())
    worker_thread_.join();

  return 0;
}

int AlsaCommonStream::close_stream()
{
  stop_stream();
  if (pcm_handle_) {
    snd_pcm_close(pcm_handle_);
    pcm_handle_ = nullptr;
  }
  return 0;
}

int AlsaCommonStream::mute_stream(bool mute)
{
  muted_ = mute;
  LOGD("mute: %s", mute ? "on" : "off");
  return 0;
}

AlsaPlaybackStream::AlsaPlaybackStream(const AudioStreamInfo & info, stream_event_callback_func cb)
  : AlsaCommonStream(info, cb)
{
  if (!pcm_mode && !info.file_path.empty()) {
    file_fd_ = open(info.file_path.c_str(), O_RDONLY);
    if (file_fd_ < 0)
      throw std::runtime_error("failed to open file: " + info.file_path);

    SF_INFO sf_info{};
    snd_file_ = sf_open_fd(file_fd_, SFM_READ, &sf_info, 0);
    if (!snd_file_) {
      close(file_fd_);
      file_fd_ = -1;
      throw std::runtime_error("failed to open sndfile: " + info.file_path);
    }

    stream_info_.rate = static_cast<uint32_t>(sf_info.samplerate);
    stream_info_.channels = static_cast<uint8_t>(sf_info.channels);

    switch (sf_info.format & SF_FORMAT_SUBMASK) {
      case SF_FORMAT_PCM_S8:
      case SF_FORMAT_PCM_U8:
        stream_info_.format = 8;
        break;
      case SF_FORMAT_PCM_16:
        stream_info_.format = 16;
        break;
      case SF_FORMAT_PCM_24:
        stream_info_.format = 24;
        break;
      case SF_FORMAT_PCM_32:
        stream_info_.format = 32;
        break;
      default:
        break;
    }
  }

  if (open_pcm(SND_PCM_STREAM_PLAYBACK) < 0) {
    if (snd_file_) {
      sf_close(snd_file_);
      snd_file_ = nullptr;
    }
    if (file_fd_ >= 0) {
      close(file_fd_);
      file_fd_ = -1;
    }
    throw std::runtime_error("failed to open playback device");
  }

  {
    snd_pcm_uframes_t period_frames =
        static_cast<snd_pcm_uframes_t>(stream_info_.rate) * ALSA_PERIOD_DURATION_MS / 1000;
    snd_pcm_uframes_t buffer_frames = period_frames * ALSA_BUFFER_PERIOD_COUNT;

    if (set_hw_params(period_frames, buffer_frames) < 0) {
      snd_pcm_close(pcm_handle_);
      pcm_handle_ = nullptr;
      if (snd_file_) {
        sf_close(snd_file_);
        snd_file_ = nullptr;
      }
      if (file_fd_ >= 0) {
        close(file_fd_);
        file_fd_ = -1;
      }
      throw std::runtime_error("failed to set hw params for playback");
    }
  }
}

AlsaPlaybackStream::~AlsaPlaybackStream()
{
  {
    std::lock_guard<std::mutex> lock(pcm_buf_mutex_);
    pcm_stop_requested_ = true;
  }
  stop_stream();
  if (snd_file_) {
    sf_close(snd_file_);
    snd_file_ = nullptr;
  }

  if (file_fd_ >= 0) {
    close(file_fd_);
    file_fd_ = -1;
  }
}

int AlsaPlaybackStream::start_stream()
{
  running_ = true;
  StreamEventData dummy;
  event_cb(StreamEvent::StreamStart, dummy, (void *)(intptr_t)handle_);

  if (pcm_mode) {
    worker_thread_ = std::thread(&AlsaPlaybackStream::pcm_playback_thread, this);
  } else {
    worker_thread_ = std::thread(&AlsaPlaybackStream::file_playback_thread, this);
  }
  return 0;
}

void AlsaPlaybackStream::file_playback_thread()
{
  const snd_pcm_uframes_t period_frames = period_frames_;
  const size_t frame_bytes = stream_info_.channels * (stream_info_.format / 8);
  const size_t buf_size = period_frames * frame_bytes;
  std::vector<uint8_t> buf(buf_size);

  do {
    if (snd_file_)
      sf_seek(snd_file_, 0, SEEK_SET);

    while (running_) {
      sf_count_t bytes_read = 0;
      if (snd_file_)
        bytes_read = sf_read_raw(snd_file_, buf.data(), buf_size);

      if (bytes_read <= 0)
        break;

      if (muted_) {
        memset(buf.data(), 0, bytes_read);
      } else {
        apply_volume(buf.data(), bytes_read, stream_info_.format, volume_);
      }
      snd_pcm_uframes_t frames = bytes_read / frame_bytes;
      snd_pcm_sframes_t written = snd_pcm_writei(pcm_handle_, buf.data(), frames);
      if (written < 0) {
        written = snd_pcm_recover(pcm_handle_, written, 0);
        if (written < 0) {
          LOGE("snd_pcm_writei: %s", snd_strerror(written));
          running_ = false;
          break;
        }
      }
    }

    if (repeat_count > 0)
      repeat_count--;
  } while (running_ && repeat_count != 0);

  if (running_) {
    snd_pcm_drain(pcm_handle_);
    running_ = false;
    StreamEventData dummy;
    event_cb(StreamEvent::StreamEos, dummy, (void *)(intptr_t)handle_);
    event_cb(StreamEvent::StreamStoped, dummy, (void *)(intptr_t)handle_);
  }
}

void AlsaPlaybackStream::pcm_playback_thread()
{
  const size_t frame_bytes = stream_info_.channels * (stream_info_.format / 8);

  while (running_) {
    if (pcm_stop_requested_)
      break;

    std::vector<uint8_t> data;
    {
      std::lock_guard<std::mutex> lock(pcm_buf_mutex_);
      if (!pcm_queue_.empty()) {
        data = std::move(pcm_queue_.front());
        pcm_queue_.pop();
      }
    }

    if (data.empty()) {
      // No data available; do not feed silence to the PCM device.
      // Just wait a bit and check the queue again.
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    if (muted_)
      memset(data.data(), 0, data.size());
    else
      apply_volume(data.data(), data.size(), stream_info_.format, volume_);

    size_t offset = 0;
    while (offset < data.size() && running_) {
      snd_pcm_uframes_t frames = (data.size() - offset) / frame_bytes;
      snd_pcm_sframes_t written = snd_pcm_writei(pcm_handle_, data.data() + offset, frames);
      if (written < 0) {
        written = snd_pcm_recover(pcm_handle_, written, 0);
        if (written < 0) {
          break;
        }
      }
      offset += written * frame_bytes;
    }
  }
}

int AlsaPlaybackStream::write_data(const void * buf, size_t size)
{
  if (!pcm_mode)
    return -EINVAL;

  std::lock_guard<std::mutex> lock(pcm_buf_mutex_);
  pcm_queue_.push(std::vector<uint8_t>(
      static_cast<const uint8_t *>(buf), static_cast<const uint8_t *>(buf) + size));
  return static_cast<int>(size);
}

AlsaCaptureStream::AlsaCaptureStream(const AudioStreamInfo & info, stream_event_callback_func cb)
  : AlsaCommonStream(info, cb)
{
  if (open_pcm(SND_PCM_STREAM_CAPTURE) < 0) {
    throw std::runtime_error("failed to open capture device");
  }

  {
    snd_pcm_uframes_t period_frames =
        static_cast<snd_pcm_uframes_t>(stream_info_.rate) * ALSA_PERIOD_DURATION_MS / 1000;
    snd_pcm_uframes_t buffer_frames = period_frames * ALSA_BUFFER_PERIOD_COUNT;

    if (set_hw_params(period_frames, buffer_frames) < 0) {
      snd_pcm_close(pcm_handle_);
      pcm_handle_ = nullptr;
      throw std::runtime_error("failed to set hw params for capture");
    }
  }

  if (!info.file_path.empty()) {
    file_fd_ = open(info.file_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (file_fd_ < 0) {
      snd_pcm_close(pcm_handle_);
      pcm_handle_ = nullptr;
      throw std::runtime_error("failed to open file: " + info.file_path);
    }
    SF_INFO sf_info{};
    sf_info.samplerate = static_cast<int>(info.rate);
    sf_info.channels = info.channels;
    sf_info.format = SF_FORMAT_WAV;

    switch (info.format) {
      case 8:
        sf_info.format = sf_info.format | SF_FORMAT_PCM_S8;
        break;
      case 16:
        sf_info.format = sf_info.format | SF_FORMAT_PCM_16;
        break;
      case 24:
        sf_info.format = sf_info.format | SF_FORMAT_PCM_24;
        break;
      case 32:
        sf_info.format = sf_info.format | SF_FORMAT_PCM_32;
        break;
      default:
        sf_info.format = sf_info.format | SF_FORMAT_PCM_16;
        break;
    }

    snd_file_ = sf_open_fd(file_fd_, SFM_WRITE, &sf_info, 0);
    if (!snd_file_) {
      close(file_fd_);
      file_fd_ = -1;
      snd_pcm_close(pcm_handle_);
      pcm_handle_ = nullptr;
      throw std::runtime_error("failed to open sndfile for write: " + info.file_path);
    }
  }
}

AlsaCaptureStream::~AlsaCaptureStream()
{
  stop_stream();

  if (snd_file_) {
    sf_close(snd_file_);
    snd_file_ = nullptr;
  }

  if (file_fd_ >= 0) {
    close(file_fd_);
    file_fd_ = -1;
  }
}

int AlsaCaptureStream::start_stream()
{
  running_ = true;
  StreamEventData dummy;
  event_cb(StreamEvent::StreamStart, dummy, (void *)(intptr_t)handle_);
  worker_thread_ = std::thread(&AlsaCaptureStream::capture_thread, this);
  return 0;
}

void AlsaCaptureStream::capture_thread()
{
  const snd_pcm_uframes_t period_frames = period_frames_;
  const size_t frame_bytes = stream_info_.channels * (stream_info_.format / 8);
  const size_t buf_size = period_frames * frame_bytes;
  std::vector<uint8_t> buf(buf_size);

  while (running_) {
    std::chrono::steady_clock::time_point read_start = std::chrono::steady_clock::now();

    snd_pcm_sframes_t frames_read = snd_pcm_readi(pcm_handle_, buf.data(), period_frames);
    if (frames_read < 0) {
      frames_read = snd_pcm_recover(pcm_handle_, frames_read, 0);
      if (frames_read < 0) {
        LOGE("snd_pcm_readi: %s", snd_strerror(frames_read));
        break;
      }
      continue;
    }

    size_t bytes = static_cast<size_t>(frames_read) * frame_bytes;

    if (snd_file_)
      sf_write_raw(snd_file_, buf.data(), bytes);

    if (pcm_mode) {
      StreamEventData s_event_data;

      s_event_data.data_buf =
          std::make_shared<std::vector<uint8_t>>(buf.begin(), buf.begin() + bytes);

      auto read_usec = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - read_start)
                           .count();
      s_event_data.usec = static_cast<uint64_t>(read_usec);

      event_cb(StreamEvent::StreamData, s_event_data, (void *)(intptr_t)handle_);
    }
  }

  StreamEventData dummy;
  event_cb(StreamEvent::StreamStoped, dummy, (void *)(intptr_t)handle_);
}

}  // namespace audio_common_lib
}  // namespace qrb
