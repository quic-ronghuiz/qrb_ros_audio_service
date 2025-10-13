// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_audio_common_lib/audio_common.hpp"

#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <random>
#include <stdexcept>

#include "stdio.h"

namespace qrb
{
namespace audio_common_lib
{

unsigned int log_lvl = (LOG_DEBUG | LOG_INFO | LOG_ERROR);

pa_threaded_mainloop * CommonAudioStream::pulse_mainloop_ = nullptr;
pa_mainloop_api * CommonAudioStream::pulse_mainloop_api_ = nullptr;
pa_context * CommonAudioStream::pulse_context_ = nullptr;
pa_time_event * CommonAudioStream::pulse_context_time_event_ = nullptr;
pa_context_state_t CommonAudioStream::pulse_context_state_ = PA_CONTEXT_UNCONNECTED;
std::set<CommonAudioStream *> CommonAudioStream::timestamp_streams_;
std::unordered_map<uint32_t, CommonAudioStream *> CommonAudioStream::stream_list_;

uint32_t CommonAudioStream::add_stream(CommonAudioStream * stream)
{
  uint32_t handle = 0;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> dist(1, UINT32_MAX);

  for (;;) {
    handle = dist(gen);
    if (stream_list_.find(handle) == stream_list_.end())
      break;
  }

  stream_list_[handle] = stream;
  return handle;
}

void CommonAudioStream::delete_stream(CommonAudioStream * stream)
{
  for (auto it = stream_list_.begin(); it != stream_list_.end();) {
    if (it->second == stream) {
      stream_list_.erase(it->first);
      break;
    } else {
      ++it;
    }
  }
}

uint32_t CommonAudioStream::get_common_stream_handle(CommonAudioStream * stream)
{
  uint32_t handle = 0;

  for (auto it = stream_list_.begin(); it != stream_list_.end();) {
    if (it->second == stream) {
      handle = it->first;
      break;
    } else {
      ++it;
    }
  }

  return handle;
}

CommonAudioStream * CommonAudioStream::get_stream(uint32_t handle)
{
  auto it = stream_list_.find(handle);
  if (it != stream_list_.end()) {
    return it->second;
  } else {
    return nullptr;
  }
}

void CommonAudioStream::pulse_stream_update_timing_callback(pa_stream * stream,
    int success,
    void * userdata)
{
  pa_usec_t l;
  pa_usec_t usec;
  int negative = 0;

  CommonAudioStream * current_stream = static_cast<CommonAudioStream *>(userdata);

  uint32_t stream_handle = get_common_stream_handle(current_stream);

  if (!success || pa_stream_get_time(stream, &usec) < 0 ||
      pa_stream_get_latency(stream, &l, &negative) < 0) {
    LOGE("failed to get latency: %s", pa_strerror(pa_context_errno(pulse_context_)));
    return;
  }

  Stream_Event_Data stream_event_data = {
    .usec = usec - current_stream->start_usec,
  };

  current_stream->event_cb(StreamEvent::StreamTimestamp, stream_event_data, (void *)stream_handle);
}

void CommonAudioStream::pulse_context_time_event_callback(pa_mainloop_api * m,
    pa_time_event * e,
    const struct timeval * t,
    void * userdata)
{
  for (CommonAudioStream * stream : timestamp_streams_) {
    if (stream && stream->get_stream_state() == PA_STREAM_READY) {
      pa_operation * o;
      if (!(o = pa_stream_update_timing_info(
                stream->get_stream_handle(), pulse_stream_update_timing_callback, stream)))
        LOGE("pa_stream_update_timing_info() failed: %s",
            pa_strerror(pa_context_errno(pulse_context_)));
      else
        pa_operation_unref(o);
    }
  }

  pa_context_rttime_restart(pulse_context_, e, pa_rtclock_now() + TIME_EVENT_USEC);
}

int CommonAudioStream::register_stream_timestamp_event(CommonAudioStream * stream_handle)
{
  timestamp_streams_.insert(stream_handle);

  if (pulse_context_time_event_ == nullptr) {
    pulse_context_time_event_ = pa_context_rttime_new(pulse_context_,
        pa_rtclock_now() + TIME_EVENT_USEC, pulse_context_time_event_callback, nullptr);
    if (!pulse_context_time_event_) {
      LOGE("pa_context_rttime_new failed");
      return -1;
    } else {
      LOGD("init context time event succeed");
    }
  }

  return 0;
}

void CommonAudioStream::deregister_stream_timestamp_event(CommonAudioStream * stream_handle)
{
  timestamp_streams_.erase(stream_handle);

  if ((timestamp_streams_.size() == 0) && pulse_context_time_event_) {
    pulse_mainloop_api_->time_free(pulse_context_time_event_);
    pulse_context_time_event_ = nullptr;
  }
}

void CommonAudioStream::pulse_context_state_callback(pa_context * context, void * userdata)
{
  assert(context);

  pulse_context_state_ = pa_context_get_state(context);
  LOGD("pulse_context_state_ = %d", pulse_context_state_);
  switch (pulse_context_state_) {
    case PA_CONTEXT_CONNECTING:
    case PA_CONTEXT_AUTHORIZING:
    case PA_CONTEXT_SETTING_NAME:
      break;

    case PA_CONTEXT_READY:
      break;

    case PA_CONTEXT_TERMINATED:
      break;

    case PA_CONTEXT_FAILED:
      LOGI("pulse context state PA_CONTEXT_FAILED");
      std::thread(clean_pulse_mainloop).detach();
      break;

    default:
      LOGE("pulse context state update error: %s", pa_strerror(pa_context_errno(context)));
  }
}

void CommonAudioStream::pulse_stream_state_callback(pa_stream * stream, void * userdata)
{
  assert(stream);

  CommonAudioStream * current_stream = static_cast<CommonAudioStream *>(userdata);
  uint32_t stream_handle = get_common_stream_handle(current_stream);

  current_stream->set_stream_state(pa_stream_get_state(stream));
  LOGD("enter status = %d", current_stream->get_stream_state());

  Stream_Event_Data dummy_data;

  switch (current_stream->get_stream_state()) {
    case PA_STREAM_CREATING:
      break;
    case PA_STREAM_TERMINATED:
      if (current_stream->repeat_count == 0)
        current_stream->event_cb(StreamEvent::StreamStoped, dummy_data, (void *)stream_handle);
      break;
    case PA_STREAM_READY:
      break;
    case PA_STREAM_FAILED:
      current_stream->event_cb(StreamEvent::StreamAbort, dummy_data, (void *)stream_handle);
      break;
    default:
      LOGE("pulse stream state update failed");
  }

  if (pulse_mainloop_)
    pa_threaded_mainloop_signal(pulse_mainloop_, 0);
}

void CommonAudioStream::stream_underflow_callback(pa_stream * stream, void * userdata)
{
  assert(stream);
  LOGD("Stream underrun.");
}

void CommonAudioStream::stream_overflow_callback(pa_stream * stream, void * userdata)
{
  assert(stream);
  LOGD("Stream overrun.");
}

pa_threaded_mainloop * CommonAudioStream::init_pulse_env()
{
  static bool first_init = false;
  bool need_start_mainloop = false;

  if (pulse_mainloop_ != nullptr && pulse_context_ != nullptr)
    return pulse_mainloop_;

  if (pulse_mainloop_ == nullptr) {
    first_init = true;
    need_start_mainloop = true;
    pulse_mainloop_ = pa_threaded_mainloop_new();
    if (!pulse_mainloop_) {
      LOGE("create pulse_mainloop_ fail");
      return nullptr;
    }
  }

  if (pulse_mainloop_api_ == nullptr)
    pulse_mainloop_api_ = pa_threaded_mainloop_get_api(pulse_mainloop_);

  if (pulse_context_ == nullptr)
    pulse_context_ = pa_context_new(pulse_mainloop_api_, "ros_context");

  if (pulse_context_ == nullptr) {
    LOGE("pulseaudio context init fail");
    return nullptr;
  }

  pa_context_set_state_callback(pulse_context_, pulse_context_state_callback, nullptr);

  if (pa_context_connect(pulse_context_, nullptr, PA_CONTEXT_NOFLAGS, nullptr) < 0) {
    LOGE("connect pulseaudio server fail %s", pa_strerror(pa_context_errno(pulse_context_)));
    return nullptr;
  }

  if (need_start_mainloop)
    if (pa_threaded_mainloop_start(pulse_mainloop_) < 0) {
      LOGE("pa_threaded_mainloop_start() failed.");
      pa_threaded_mainloop_free(pulse_mainloop_);
      pulse_mainloop_ = nullptr;
      return nullptr;
    }

  if (first_init)
    atexit(clean_pulse_mainloop);

  return pulse_mainloop_;
}

void CommonAudioStream::clean_pulse_mainloop()
{
  LOGD("enter");
  if (pulse_context_time_event_) {
    if (pulse_mainloop_api_)
      pulse_mainloop_api_->time_free(pulse_context_time_event_);
    pulse_context_time_event_ = nullptr;
  }

  if (!stream_list_.empty()) {
    for (auto it = stream_list_.begin(); it != stream_list_.end();) {
      CommonAudioStream * current_stream = it->second;
      ++it;
      delete current_stream;
    }
  }

  if (pulse_context_) {
    pa_context_unref(pulse_context_);
    pulse_context_ = nullptr;
  }

  if (pulse_mainloop_) {
    pa_threaded_mainloop_stop(pulse_mainloop_);
    pa_threaded_mainloop_free(pulse_mainloop_);
    pulse_mainloop_ = nullptr;
    pulse_mainloop_api_ = nullptr;
  };
}

void CommonAudioStream::check_context_ready()
{
  for (;;) {
    if (pulse_context_state_ == PA_CONTEXT_READY)
      break;
    usleep(5000);
  }
}

CommonAudioStream::CommonAudioStream(string filepath, std::shared_ptr<pa_sample_spec> sample_spec)
  : m_file_path(filepath), m_sample_spec(sample_spec)
{
  if (CommonAudioStream::pulse_context_ == nullptr) {
    CommonAudioStream::init_pulse_env();
    CommonAudioStream::check_context_ready();
  }
}

CommonAudioStream::~CommonAudioStream()
{
  LOGD("delete Stream handle %u", get_common_stream_handle(this));
  switch (stream_state_) {
    case PA_STREAM_READY:
      stop_stream();
    case PA_STREAM_TERMINATED:
    case PA_STREAM_UNCONNECTED:
      close_stream();
      break;
    case PA_STREAM_FAILED:
      stop_stream();
      close_stream();
      break;
    default:
      break;
  }

  delete_stream(this);

  if (snd_file) {
    sf_close(snd_file);
    snd_file = nullptr;
  }

  if (file_fd) {
    close(file_fd);
  }
}

uint32_t CommonAudioStream::audio_stream_open(const audio_stream_info & stream_info,
    stream_event_callback_func event_callback)
{
  uint32_t stream_handle = 0;
  CommonAudioStream * stream = nullptr;
  std::shared_ptr<pa_sample_spec> stream_sample_spec = std::make_shared<pa_sample_spec>();

  stream_sample_spec->rate = stream_info.rate;
  stream_sample_spec->channels = stream_info.channels;

  switch (stream_info.format) {
    case 8:
      stream_sample_spec->format = PA_SAMPLE_U8;
      break;
    case 16:
      stream_sample_spec->format = PA_SAMPLE_S16NE;
      break;
    case 24:
      stream_sample_spec->format = PA_SAMPLE_S24NE;
      break;
    case 32:
      stream_sample_spec->format = PA_SAMPLE_S32NE;
      break;
  }

  if (stream_info.file_path.empty() && (!pa_sample_spec_valid(stream_sample_spec.get()))) {
    LOGE("Stream sample spec is invalid");
    return stream_handle;
  }

  try {
    if (stream_info.type == StreamPlayback)
      stream = new PlaybackStream(string(stream_info.file_path), stream_sample_spec);
    else if (stream_info.type == StreamCapture)
      stream = new CaptureStream(string(stream_info.file_path), stream_sample_spec);
    else
      stream = nullptr;
  } catch (const std::runtime_error & e) {
    LOGE("create stream error:%s", e.what());
    stream = nullptr;
  }

  if (!stream)
    return stream_handle;
  LOGD("create stream succeed");

  if (stream_info.volume && stream_info.volume > STREAM_VOL_MIN &&
      stream_info.volume <= STREAM_VOL_MAX)
    stream->mvolume = stream_info.volume;
  else {
    LOGE("Wrong volume");
    return stream_handle;
  }

  stream->repeat_count = stream_info.repeat;

  stream->event_cb = event_callback;
  stream->pcm_mode_ = stream_info.pcm_mode;

  if (stream_info.need_timestamp) {
    register_stream_timestamp_event(stream);
  }

  return add_stream(stream);
}

void CommonAudioStream::init_pulse_stream()
{
  stream_handle_ = pa_stream_new(CommonAudioStream::pulse_context_,
      std::to_string(intptr_t(this)).c_str(), m_sample_spec.get(), nullptr);
  if (stream_handle_ == nullptr) {
    LOGE("create stream:%s", pa_strerror(pa_context_errno(CommonAudioStream::pulse_context_)));
    throw std::runtime_error("create pulseaudio stream return nullptr");
  }

  pa_stream_set_state_callback(
      stream_handle_, CommonAudioStream::pulse_stream_state_callback, this);

  pa_stream_set_underflow_callback(
      stream_handle_, CommonAudioStream::stream_underflow_callback, nullptr);
  pa_stream_set_overflow_callback(
      stream_handle_, CommonAudioStream::stream_overflow_callback, nullptr);
}

int CommonAudioStream::pause_stream(bool pause)
{
  if (stream_state_ != PA_STREAM_READY) {
    LOGE("stream state(%d) error, pause fail", stream_state_);
    return -1;
  }

  pa_operation * op_pause = pa_stream_cork(stream_handle_, pause, nullptr, nullptr);
  if (op_pause) {
    pa_operation_unref(op_pause);
  }

  return 0;
}

int CommonAudioStream::mute_stream(bool mute)
{
  uint32_t stream_index;
  pa_operation * op;

  stream_index = pa_stream_get_index(stream_handle_);
  op = pa_context_set_sink_input_mute(
      CommonAudioStream::pulse_context_, stream_index, mute, nullptr, nullptr);
  if (!op) {
    LOGE("failed to set stream(%p) to %s", static_cast<void *>(stream_handle_),
        mute ? "mute" : "unmute");
    pa_operation_unref(op);
  }

  return 0;
}

int CommonAudioStream::stop_stream()
{
  LOGD("stream state(%d)", stream_state_);

  if (stream_state_ == PA_STREAM_TERMINATED) {
    LOGI("stream state(%d) already in stop", stream_state_);
    return 0;
  }

  if (stream_state_ != PA_STREAM_READY || stream_state_ == PA_STREAM_FAILED) {
    LOGE("stream state(%d) error, Stop fail", stream_state_);
    return -EPERM;
  }

  CommonAudioStream::deregister_stream_timestamp_event(this);

  repeat_count = 0;

  if (pa_stream_disconnect(stream_handle_) < 0) {  // after disconnect state will be
                                                   // PA_STREAM_TERMINATED(4)
    LOGE("Disconnect Stream fail");
  }

  pa_threaded_mainloop_lock(CommonAudioStream::pulse_mainloop_);
  while (stream_state_ != PA_STREAM_TERMINATED) {
    pa_threaded_mainloop_wait(CommonAudioStream::pulse_mainloop_);
  }
  pa_threaded_mainloop_unlock(CommonAudioStream::pulse_mainloop_);

  LOGD("complete, stream_state_ = %d", stream_state_);
  return 0;
}

void CommonAudioStream::internal_stopstream()
{
  std::thread t(&CommonAudioStream::stop_stream, this);
  t.detach();
}

int CommonAudioStream::close_stream()
{
  LOGD("stream state(%d), stream_handle_(%u)", stream_state_, get_common_stream_handle(this));

  if ((stream_state_ == PA_STREAM_CREATING) || (stream_state_ == PA_STREAM_READY)) {
    LOGE("stream state(%d) error, Close fail", stream_state_);
    return -EPERM;
  }
  if (stream_handle_) {
    pa_stream_set_state_callback(stream_handle_, nullptr, nullptr);
    pa_stream_set_underflow_callback(stream_handle_, nullptr, nullptr);
    pa_stream_set_overflow_callback(stream_handle_, nullptr, nullptr);
    pa_stream_set_write_callback(stream_handle_, nullptr, nullptr);
    pa_stream_set_read_callback(stream_handle_, nullptr, nullptr);

    pa_stream_unref(stream_handle_);
    stream_handle_ = nullptr;
  }

  if (stream_handle_ == nullptr)
    return 0;
  return -EPERM;
}

}  // namespace audio_common_lib
}  // namespace qrb
