// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_AUDIO_COMMON_LIB__ALSA_STREAM_HPP_
#define QRB_AUDIO_COMMON_LIB__ALSA_STREAM_HPP_

#include <atomic>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "alsa/asoundlib.h"
#include "qrb_audio_common_lib/audio_log.hpp"
#include "qrb_audio_common_lib/audio_stream.hpp"
#include "sndfile.h"

namespace qrb
{
namespace audio_common_lib
{

#define ALSA_DEFAULT_DEVICE "default"
#define ALSA_DEFAULT_PERIOD_FRAMES 1024
#define ALSA_DEFAULT_BUFFER_FRAMES 8192

// Device names to try, ordered by priority (first entry tried first).
inline const std::vector<const char *> kAlsaPlaybackProbeDevices = {
  ALSA_DEFAULT_DEVICE,
};

inline const std::vector<const char *> kAlsaCaptureProbeDevices = {
  ALSA_DEFAULT_DEVICE,
};

class AlsaCommonStream : public IAudioStream
{
public:
  virtual ~AlsaCommonStream();
  virtual int start_stream() = 0;
  int stop_stream() override;
  int close_stream() override;
  int mute_stream(bool mute) override;

  static uint32_t audio_stream_open(const AudioStreamInfo & info,
      stream_event_callback_func event_cb);
  static bool detect_alsa_available(std::string & playback_device, std::string & capture_device);

protected:
  AlsaCommonStream(const AudioStreamInfo & info, stream_event_callback_func cb);

  int open_pcm(snd_pcm_stream_t direction);
  int set_hw_params(snd_pcm_uframes_t period_frames, snd_pcm_uframes_t buffer_frames);
  snd_pcm_format_t get_alsa_format() const;

  AudioStreamInfo stream_info_;
  snd_pcm_t * pcm_handle_ = nullptr;
  std::atomic<bool> running_{ false };
  std::thread worker_thread_;
  bool muted_ = false;
  uint8_t volume_ = 100;
  std::string resolved_device_;
};

class AlsaPlaybackStream : public AlsaCommonStream
{
public:
  AlsaPlaybackStream(const AudioStreamInfo & info, stream_event_callback_func cb);
  ~AlsaPlaybackStream() override;
  int start_stream() override;
  int write_data(const void * buf, size_t size) override;

private:
  void file_playback_thread();
  void pcm_playback_thread();

  SNDFILE * snd_file_ = nullptr;
  int file_fd_ = -1;

  std::mutex pcm_buf_mutex_;
  std::queue<std::vector<uint8_t>> pcm_queue_;
  bool pcm_stop_requested_ = false;
};

class AlsaCaptureStream : public AlsaCommonStream
{
public:
  AlsaCaptureStream(const AudioStreamInfo & info, stream_event_callback_func cb);
  ~AlsaCaptureStream() override;
  int start_stream() override;

private:
  void capture_thread();

  SNDFILE * snd_file_ = nullptr;
  int file_fd_ = -1;
};

}  // namespace audio_common_lib
}  // namespace qrb

#endif  // QRB_AUDIO_COMMON_LIB__ALSA_STREAM_HPP_
