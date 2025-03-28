// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_AUDIO_COMMON__AUDIO_COMMON_HPP_
#define QRB_AUDIO_COMMON__AUDIO_COMMON_HPP_

#include <pulse/pulseaudio.h>
#include <sndfile.h>

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>

using namespace std;

namespace qrb
{
namespace audio_common_lib
{

#define DEFAULT_VALUE (uint32_t) - 1
#define TIME_EVENT_USEC 50000
#define STREAM_VOL_MAX 100
#define STREAM_VOL_MIN 0

#define LOG_ERROR (0x1)
#define LOG_INFO (0x2)
#define LOG_DEBUG (0x4)
#define LOG_VERBOSE (0x8)

#define LOGV(format, ...)                                                                          \
  do {                                                                                             \
    if (log_lvl & LOG_VERBOSE) {                                                                   \
      fprintf(stdout, "[VERBOSE] %s: " format "\n", __func__, ##__VA_ARGS__);                      \
      fflush(stdout);                                                                              \
    }                                                                                              \
  } while (0)

#define LOGD(format, ...)                                                                          \
  do {                                                                                             \
    if (log_lvl & LOG_DEBUG) {                                                                     \
      fprintf(stdout, "[DEBUG] %s: " format "\n", __func__, ##__VA_ARGS__);                        \
      fflush(stdout);                                                                              \
    }                                                                                              \
  } while (0)

#define LOGI(format, ...)                                                                          \
  do {                                                                                             \
    if (log_lvl & LOG_INFO) {                                                                      \
      fprintf(stdout, "[INFO] %s: " format "\n", __func__, ##__VA_ARGS__);                         \
      fflush(stdout);                                                                              \
    }                                                                                              \
  } while (0)

#define LOGE(format, ...)                                                                          \
  do {                                                                                             \
    if (log_lvl & LOG_ERROR) {                                                                     \
      fprintf(stderr, "[ERROR] %s: " format "\n", __func__, ##__VA_ARGS__);                        \
    }                                                                                              \
  } while (0)

extern unsigned int log_lvl;

typedef enum
{
  StreamInvalid,
  StreamPlayback,
  StreamCapture,
} stream_type;

enum class StreamEvent
{
  StreamStart,
  StreamStoped,
  StreamAbort,
  StreamTimestamp,
  StreamData,
};

union Stream_Event_Data
{
  struct
  {
    intptr_t data_ptr;
    size_t data_size;
  } data;
  uint64_t usec;
};

struct audio_stream_info
{
  uint8_t format = 16;
  uint32_t rate = 16000;
  uint8_t channels = 2;
  stream_type type = StreamInvalid;
  string file_path = "";
  uint8_t volume = 0;
  string device = "";
  bool need_timestamp = false;
  bool pcm_mode = false;
  int32_t repeat = 0;
};

using stream_event_callback_func = std::function<void(StreamEvent, Stream_Event_Data, void *)>;

class CommonAudioStream
{
private:
  pa_stream_state_t stream_state_ = PA_STREAM_UNCONNECTED;
  pa_stream * stream_handle_ = nullptr;
  static pa_context_state_t pulse_context_state_;
  static std::unordered_map<uint32_t, CommonAudioStream *> stream_list_;

  static pa_threaded_mainloop * init_pulse_env();
  static void clean_pulse_mainloop();
  static uint32_t add_stream(CommonAudioStream * stream);
  static void delete_stream(CommonAudioStream * stream);

protected:
  bool pcm_mode_ = false;
  static pa_threaded_mainloop * pulse_mainloop_;
  static pa_mainloop_api * pulse_mainloop_api_;
  static pa_context * pulse_context_;
  static pa_time_event * pulse_context_time_event_;
  static std::set<CommonAudioStream *> timestamp_streams_;

  CommonAudioStream(string filepath, shared_ptr<pa_sample_spec> sample_spec);
  void init_pulse_stream();
  static void pulse_stream_update_timing_callback(pa_stream * stream, int success, void * userdata);
  static void pulse_context_time_event_callback(pa_mainloop_api * m,
      pa_time_event * e,
      const struct timeval * t,
      void * userdata);
  static int register_stream_timestamp_event(CommonAudioStream * stream_handle);
  static void deregister_stream_timestamp_event(CommonAudioStream * stream_handle);
  static void pulse_context_state_callback(pa_context * context, void * userdata);
  static void pulse_stream_state_callback(pa_stream * stream, void * userdata);
  static void stream_underflow_callback(pa_stream * stream, void * userdata);
  static void stream_overflow_callback(pa_stream * stream, void * userdata);
  static void check_context_ready();
  static uint32_t get_common_stream_handle(CommonAudioStream * stream);

public:
  stream_event_callback_func event_cb;
  std::shared_ptr<pa_sample_spec> m_sample_spec;
  float mvolume = 50;
  int32_t repeat_count = 0;
  pa_usec_t start_usec = 0;

  string m_file_path;
  SNDFILE * snd_file = nullptr;

  virtual ~CommonAudioStream();
  static uint32_t audio_stream_open(const audio_stream_info & stream_info,
      stream_event_callback_func event_callback);

  virtual int start_stream() = 0;
  int pause_stream(bool pause);
  int mute_stream(bool mute);
  int stop_stream();
  void internal_stopstream();
  int close_stream();
  virtual int sndfile_open() = 0;

  void set_stream_state(pa_stream_state_t next_stream_state) { stream_state_ = next_stream_state; }
  pa_stream_state_t get_stream_state(void) { return stream_state_; }

  pa_stream * get_stream_handle(void) { return stream_handle_; }
  static CommonAudioStream * get_stream(uint32_t handle);
};

class PlaybackStream : public CommonAudioStream
{
public:
  PlaybackStream(string filepath, std::shared_ptr<pa_sample_spec> sample_spec);
  virtual int sndfile_open();
  int start_stream();
  static void stream_data_callback(pa_stream * stream, size_t length, void * userdata);
  size_t sndfile_transfer_data(void * data, size_t length);
};

class CaptureStream : public CommonAudioStream
{
public:
  CaptureStream(string filepath, std::shared_ptr<pa_sample_spec> sample_spec);
  virtual int sndfile_open();
  int start_stream();
  static void stream_data_callback(pa_stream * stream, size_t length, void * userdata);
  size_t sndfile_transfer_data(const void * data, size_t length);
};

}  // namespace audio_common_lib
}  // namespace qrb

#endif
