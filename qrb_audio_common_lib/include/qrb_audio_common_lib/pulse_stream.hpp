// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_AUDIO_COMMON_LIB__PULSE_STREAM_HPP_
#define QRB_AUDIO_COMMON_LIB__PULSE_STREAM_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>

#include "pulse/pulseaudio.h"
#include "qrb_audio_common_lib/audio_log.hpp"
#include "qrb_audio_common_lib/audio_stream.hpp"
#include "sndfile.h"

namespace qrb
{
namespace audio_common_lib
{

#define DEFAULT_VALUE (uint32_t) - 1
#define TIME_EVENT_USEC 50000
#define STREAM_VOL_MAX 100
#define STREAM_VOL_MIN 0

class PulseCommonStream : public IAudioStream
{
private:
  pa_stream_state_t stream_state_ = PA_STREAM_UNCONNECTED;
  pa_stream * stream_handle_ = nullptr;
  static pa_context_state_t pulse_context_state_;

  static pa_threaded_mainloop * init_pulse_env();
  static void clean_pulse_mainloop();

protected:
  static pa_threaded_mainloop * pulse_mainloop_;
  static pa_mainloop_api * pulse_mainloop_api_;
  static pa_context * pulse_context_;
  static pa_time_event * pulse_context_time_event_;
  static std::set<PulseCommonStream *> timestamp_streams_;
  int file_fd_ = 0;

  PulseCommonStream(std::string filepath, std::shared_ptr<pa_sample_spec> sample_spec);
  void init_pulse_stream();
  static void pulse_stream_update_timing_callback(pa_stream * stream, int success, void * userdata);
  static void pulse_context_time_event_callback(pa_mainloop_api * m,
      pa_time_event * e,
      const struct timeval * t,
      void * userdata);
  static int register_stream_timestamp_event(PulseCommonStream * stream_handle);
  static void deregister_stream_timestamp_event(PulseCommonStream * stream_handle);
  static void pulse_context_state_callback(pa_context * context, void * userdata);
  static void pulse_stream_state_callback(pa_stream * stream, void * userdata);
  static void stream_underflow_callback(pa_stream * stream, void * userdata);
  static void stream_overflow_callback(pa_stream * stream, void * userdata);
  static void check_context_ready();

public:
  std::shared_ptr<pa_sample_spec> sample_spec;
  float stream_volume = 100;
  pa_usec_t start_usec = 0;

  std::string file_path;
  SNDFILE * snd_file = nullptr;

  virtual ~PulseCommonStream();
  static uint32_t audio_stream_open(const AudioStreamInfo & stream_info,
      stream_event_callback_func event_callback);

  virtual int start_stream() = 0;
  int pause_stream(bool pause) override;
  int mute_stream(bool mute) override;
  int stop_stream() override;
  void internal_stopstream();
  int close_stream() override;
  virtual int sndfile_open() = 0;

  void set_stream_state(pa_stream_state_t next_stream_state) { stream_state_ = next_stream_state; }
  pa_stream_state_t get_stream_state(void) { return stream_state_; }

  pa_stream * get_stream_handle(void) { return stream_handle_; }
};

class PlaybackStream : public PulseCommonStream
{
public:
  PlaybackStream(std::string filepath, std::shared_ptr<pa_sample_spec> sample_spec);
  virtual int sndfile_open();
  int start_stream() override;
  int write_data(const void * buf, size_t size) override;
  static void stream_data_callback(pa_stream * stream, size_t length, void * userdata);
  size_t sndfile_transfer_data(void * data, size_t length);
};

class CaptureStream : public PulseCommonStream
{
public:
  CaptureStream(std::string filepath, std::shared_ptr<pa_sample_spec> sample_spec);
  virtual int sndfile_open();
  int start_stream() override;
  static void stream_data_callback(pa_stream * stream, size_t length, void * userdata);
  size_t sndfile_transfer_data(const void * data, size_t length);
};

}  // namespace audio_common_lib
}  // namespace qrb

#endif  // QRB_AUDIO_COMMON_LIB__PULSE_STREAM_HPP_
