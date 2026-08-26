// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_audio_common_lib/pulse_stream.hpp"

#include <fcntl.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <map>
#include <stdexcept>
#include <thread>

using namespace std;

#define PLAYBACK_BUFFER_DURATION_MS 300
#define CAPTURE_FRAGMENT_DURATION_MS 20
#define RECORD_GAIN 8

namespace qrb
{
namespace audio_common_lib
{

unsigned int log_lvl = (LOG_DEBUG | LOG_INFO | LOG_ERROR);

pa_threaded_mainloop * PulseCommonStream::pulse_mainloop_ = nullptr;
pa_mainloop_api * PulseCommonStream::pulse_mainloop_api_ = nullptr;
pa_context * PulseCommonStream::pulse_context_ = nullptr;
pa_time_event * PulseCommonStream::pulse_context_time_event_ = nullptr;
pa_context_state_t PulseCommonStream::pulse_context_state_ = PA_CONTEXT_UNCONNECTED;
std::set<PulseCommonStream *> PulseCommonStream::timestamp_streams_;

static const std::multimap<pa_sample_format_t, int> playback_format_map{
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

static const std::multimap<pa_sample_format_t, int> capture_format_map{
  { PA_SAMPLE_U8, SF_FORMAT_PCM_U8 },
  { PA_SAMPLE_S16NE, SF_FORMAT_PCM_16 },
  { PA_SAMPLE_S24NE, SF_FORMAT_PCM_24 },
  { PA_SAMPLE_S32NE, SF_FORMAT_PCM_32 },
  { PA_SAMPLE_ULAW, SF_FORMAT_ULAW },
  { PA_SAMPLE_ALAW, SF_FORMAT_ALAW },
  { PA_SAMPLE_FLOAT32NE, SF_FORMAT_FLOAT },
};

void PulseCommonStream::pulse_stream_update_timing_callback(pa_stream * stream,
    int success,
    void * userdata)
{
  pa_usec_t latency = 0;
  pa_usec_t usec = 0;
  int negative = 0;

  PulseCommonStream * current_stream = static_cast<PulseCommonStream *>(userdata);
  uint32_t stream_handle = IAudioStream::get_handle(current_stream);

  if (!success || pa_stream_get_time(stream, &usec) < 0 ||
      pa_stream_get_latency(stream, &latency, &negative) < 0) {
    LOGE("failed to get latency: %s", pa_strerror(pa_context_errno(pulse_context_)));
    return;
  }

  StreamEventData stream_event_data = {
    .usec = usec - current_stream->start_usec,
  };

  current_stream->event_cb(StreamEvent::StreamTimestamp, stream_event_data, (void *)stream_handle);
}

void PulseCommonStream::pulse_context_time_event_callback(pa_mainloop_api * m,
    pa_time_event * e,
    const struct timeval * t,
    void * userdata)
{
  for (PulseCommonStream * stream : timestamp_streams_) {
    if (stream && stream->get_stream_state() == PA_STREAM_READY) {
      pa_operation * o;
      if (!(o = pa_stream_update_timing_info(
                stream->get_stream_handle(), pulse_stream_update_timing_callback, stream))) {
        LOGE("pa_stream_update_timing_info() failed: %s",
            pa_strerror(pa_context_errno(pulse_context_)));
      } else {
        pa_operation_unref(o);
      }
    }
  }

  pa_context_rttime_restart(pulse_context_, e, pa_rtclock_now() + TIME_EVENT_USEC);
}

int PulseCommonStream::register_stream_timestamp_event(PulseCommonStream * stream_handle)
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

void PulseCommonStream::deregister_stream_timestamp_event(PulseCommonStream * stream_handle)
{
  timestamp_streams_.erase(stream_handle);

  if ((timestamp_streams_.size() == 0) && pulse_context_time_event_) {
    pulse_mainloop_api_->time_free(pulse_context_time_event_);
    pulse_context_time_event_ = nullptr;
  }
}

void PulseCommonStream::pulse_context_state_callback(pa_context * context, void * userdata)
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

void PulseCommonStream::pulse_stream_state_callback(pa_stream * stream, void * userdata)
{
  assert(stream);

  PulseCommonStream * current_stream = static_cast<PulseCommonStream *>(userdata);
  uint32_t stream_handle = IAudioStream::get_handle(current_stream);

  current_stream->set_stream_state(pa_stream_get_state(stream));
  LOGD("enter status = %d", current_stream->get_stream_state());

  StreamEventData dummy_data{};

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

void PulseCommonStream::stream_underflow_callback(pa_stream * stream, void * userdata)
{
  assert(stream);
  LOGD("Stream underrun.");
}

void PulseCommonStream::stream_overflow_callback(pa_stream * stream, void * userdata)
{
  assert(stream);
  LOGD("Stream overrun.");
}

pa_threaded_mainloop * PulseCommonStream::init_pulse_env()
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

  if (need_start_mainloop) {
    if (pa_threaded_mainloop_start(pulse_mainloop_) < 0) {
      LOGE("pa_threaded_mainloop_start() failed.");
      pa_threaded_mainloop_free(pulse_mainloop_);
      pulse_mainloop_ = nullptr;
      return nullptr;
    }
  }

  if (first_init)
    atexit(clean_pulse_mainloop);

  return pulse_mainloop_;
}

void PulseCommonStream::clean_pulse_mainloop()
{
  LOGD("enter");
  if (pulse_context_time_event_) {
    if (pulse_mainloop_api_)
      pulse_mainloop_api_->time_free(pulse_context_time_event_);

    pulse_context_time_event_ = nullptr;
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

void PulseCommonStream::check_context_ready()
{
  for (;;) {
    if (pulse_context_state_ == PA_CONTEXT_READY)
      break;
    usleep(5000);
  }
}

PulseCommonStream::PulseCommonStream(string filepath, std::shared_ptr<pa_sample_spec> sample_spec)
  : file_path(filepath), sample_spec(sample_spec)
{
  if (PulseCommonStream::pulse_context_ == nullptr) {
    PulseCommonStream::init_pulse_env();
    PulseCommonStream::check_context_ready();
  }
}

PulseCommonStream::~PulseCommonStream()
{
  LOGD("delete Stream handle %u", IAudioStream::get_handle(this));
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

  if (snd_file) {
    sf_close(snd_file);
    snd_file = nullptr;
  }
  if (file_fd_) {
    close(file_fd_);
  }
}

uint32_t PulseCommonStream::audio_stream_open(const AudioStreamInfo & stream_info,
    stream_event_callback_func event_callback)
{
  uint32_t stream_handle = 0;
  PulseCommonStream * stream = nullptr;
  std::shared_ptr<pa_sample_spec> streasample_spec = std::make_shared<pa_sample_spec>();

  streasample_spec->rate = stream_info.rate;
  streasample_spec->channels = stream_info.channels;

  switch (stream_info.format) {
    case 8:
      streasample_spec->format = PA_SAMPLE_U8;
      break;
    case 16:
      streasample_spec->format = PA_SAMPLE_S16NE;
      break;
    case 24:
      streasample_spec->format = PA_SAMPLE_S24NE;
      break;
    case 32:
      streasample_spec->format = PA_SAMPLE_S32NE;
      break;
    default:
      streasample_spec->format = PA_SAMPLE_S16NE;
      break;
  }

  if (stream_info.file_path.empty() && (!pa_sample_spec_valid(streasample_spec.get()))) {
    LOGE("Stream sample spec is invalid");
    return stream_handle;
  }

  try {
    if (stream_info.type == StreamType::Playback)
      stream = new PlaybackStream(string(stream_info.file_path), streasample_spec);
    else if (stream_info.type == StreamType::Capture)
      stream = new CaptureStream(string(stream_info.file_path), streasample_spec);
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
    stream->stream_volume = stream_info.volume;
  else {
    LOGE("Wrong volume");
    return stream_handle;
  }

  stream->repeat_count = stream_info.repeat;
  stream->event_cb = event_callback;
  stream->pcm_mode = stream_info.pcm_mode;

  if (stream_info.need_timestamp)
    register_stream_timestamp_event(stream);

  return IAudioStream::register_stream(stream);
}

void PulseCommonStream::init_pulse_stream()
{
  pa_channel_map channel_map;

  pa_channel_map_init_extend(&channel_map, sample_spec->channels, PA_CHANNEL_MAP_DEFAULT);

  stream_handle_ = pa_stream_new(PulseCommonStream::pulse_context_,
      std::to_string(intptr_t(this)).c_str(), sample_spec.get(), &channel_map);
  if (stream_handle_ == nullptr) {
    LOGE("create stream:%s", pa_strerror(pa_context_errno(PulseCommonStream::pulse_context_)));
    throw std::runtime_error("create pulseaudio stream return nullptr");
  }

  pa_stream_set_state_callback(
      stream_handle_, PulseCommonStream::pulse_stream_state_callback, this);
  pa_stream_set_underflow_callback(
      stream_handle_, PulseCommonStream::stream_underflow_callback, nullptr);
  pa_stream_set_overflow_callback(
      stream_handle_, PulseCommonStream::stream_overflow_callback, nullptr);
}

int PulseCommonStream::pause_stream(bool pause)
{
  if (stream_state_ != PA_STREAM_READY) {
    LOGE("stream state(%d) error, pause fail", stream_state_);
    return -1;
  }

  pa_operation * op_pause = pa_stream_cork(stream_handle_, pause, nullptr, nullptr);
  if (op_pause)
    pa_operation_unref(op_pause);

  return 0;
}

int PulseCommonStream::mute_stream(bool mute)
{
  uint32_t stream_index = pa_stream_get_index(stream_handle_);
  pa_operation * op = pa_context_set_sink_input_mute(
      PulseCommonStream::pulse_context_, stream_index, mute, nullptr, nullptr);
  if (!op) {
    LOGE("failed to set stream(%p) to %s", static_cast<void *>(stream_handle_),
        mute ? "mute" : "unmute");
    pa_operation_unref(op);
  }

  return 0;
}

int PulseCommonStream::stop_stream()
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

  PulseCommonStream::deregister_stream_timestamp_event(this);
  repeat_count = 0;

  if (pa_stream_disconnect(stream_handle_) < 0) {
    LOGE("Disconnect Stream fail");
  }

  pa_threaded_mainloop_lock(PulseCommonStream::pulse_mainloop_);
  while (stream_state_ != PA_STREAM_TERMINATED)
    pa_threaded_mainloop_wait(PulseCommonStream::pulse_mainloop_);
  pa_threaded_mainloop_unlock(PulseCommonStream::pulse_mainloop_);

  LOGD("complete, stream_state_ = %d", stream_state_);
  return 0;
}

void PulseCommonStream::internal_stopstream()
{
  std::thread t(&PulseCommonStream::stop_stream, this);
  t.detach();
}

int PulseCommonStream::close_stream()
{
  LOGD("stream state(%d), handle(%u)", stream_state_, IAudioStream::get_handle(this));

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
        pa_stream_get_time(stream, &current_stream->start_usec);

        pa_stream_set_write_callback(stream, PlaybackStream::stream_data_callback, current_stream);
        writable = pa_stream_writable_size(stream);
        PlaybackStream::stream_data_callback(stream, writable, current_stream);
        pa_stream_trigger(stream, nullptr, nullptr);
      }
    });
    t.detach();
  } else {
    LOGI("EOS\n");
    uint32_t stream_handle = IAudioStream::get_handle(current_stream);
    StreamEventData dummy_data{};
    current_stream->event_cb(StreamEvent::StreamEos, dummy_data, (void *)(intptr_t)stream_handle);
    current_stream->internal_stopstream();
  }
}

PlaybackStream::PlaybackStream(string filepath, std::shared_ptr<pa_sample_spec> sample_spec)
  : PulseCommonStream(filepath, sample_spec)
{
  if (!filepath.empty()) {
    if (sndfile_open())
      throw std::runtime_error("Open file failed");
  } else {
    LOGD("no file path passed, PCM data mode\n");
  }

  init_pulse_stream();

  if (!file_path.empty()) {
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
    LOGE("stream handle(%p) state(%d) error, start fail", static_cast<void *>(get_stream_handle()),
        get_stream_state());
    return -EPERM;
  }

  buffer_attr.maxlength = (uint32_t)-1;
  buffer_attr.tlength = (uint32_t)-1;
  buffer_attr.prebuf =
      pa_usec_to_bytes(PLAYBACK_BUFFER_DURATION_MS * PA_USEC_PER_MSEC, sample_spec.get());
  buffer_attr.minreq =
      pa_usec_to_bytes(PLAYBACK_BUFFER_DURATION_MS * PA_USEC_PER_MSEC, sample_spec.get());
  buffer_attr.fragsize = (uint32_t)-1;

  pa_cvolume_set(&volume, sample_spec->channels, PA_VOLUME_NORM * stream_volume / STREAM_VOL_MAX);

  if (pa_stream_connect_playback(
          get_stream_handle(), nullptr, &buffer_attr, flags, &volume, nullptr)) {
    LOGE("pa_stream_connect_playback() failed: %s",
        pa_strerror(pa_context_errno(PulseCommonStream::pulse_context_)));
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

  LOGI("opening %s", file_path.c_str());
  if ((file_fd_ = open(file_path.c_str(), O_RDONLY, 0666)) < 0) {
    LOGE("open %s failed", file_path.c_str());
    return -ENOENT;
  }

  snd_file = sf_open_fd(file_fd_, SFM_READ, &snd_file_info, 0);
  if (snd_file == nullptr) {
    LOGE("snd file open failed");
    return -SF_ERR_MALFORMED_FILE;
  }

  sf_errno = sf_command(snd_file, SFC_GET_CURRENT_SF_INFO, &snd_file_info, sizeof(snd_file_info));
  if (sf_errno) {
    LOGE("snd file SFC_GET_CURRENT_SF_INFO failed");
    return -SF_ERR_UNRECOGNISED_FORMAT;
  }

  for (auto it = playback_format_map.begin(); it != playback_format_map.end(); ++it) {
    if (it->second == (snd_file_info.format & SF_FORMAT_SUBMASK)) {
      sample_spec->format = it->first;
      match_format = true;
      break;
    }
  }
  if (!match_format) {
    LOGE("snd file format not support");
    return SF_ERR_UNRECOGNISED_FORMAT;
  }

  sample_spec->rate = (uint32_t)snd_file_info.samplerate;
  sample_spec->channels = (uint8_t)snd_file_info.channels;

  pa_channel_map_init_extend(&channel_map, sample_spec->channels, PA_CHANNEL_MAP_DEFAULT);

  LOGD("open %s succeed", file_path.c_str());

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

int PlaybackStream::write_data(const void * buf, size_t length)
{
  if (!pcm_mode)
    return -EINVAL;

  pa_stream * pulse_stream = get_stream_handle();
  void * pulse_buf;
  size_t bytes_written;
  int bytes_total_written = 0;

  if (get_stream_state() == PA_STREAM_READY) {
    size_t remaining = length;
    while (remaining > 0) {
      bytes_written = remaining;
      if (pa_stream_begin_write(pulse_stream, &pulse_buf, &bytes_written) < 0) {
        return -EIO;
      }
      if (bytes_written > 0) {
        memcpy(pulse_buf, static_cast<const uint8_t *>(buf) + bytes_total_written, bytes_written);
        pa_stream_write(pulse_stream, pulse_buf, bytes_written, nullptr, 0, PA_SEEK_RELATIVE);
        bytes_total_written += static_cast<int>(bytes_written);
        remaining -= bytes_written;
      } else {
        break;
      }
    }
  }
  return bytes_total_written;
}

void PlaybackStream::stream_data_callback(pa_stream * stream, size_t length, void * userdata)
{
  assert(stream);
  assert(length > 0);

  PlaybackStream * current_stream = static_cast<PlaybackStream *>(userdata);

  void * buf_pulse = nullptr;
  size_t bytes_written = 0;
  size_t bytes_read = 0;

  for (;;) {
    bytes_written = length;
    if ((pa_stream_begin_write(stream, &buf_pulse, &bytes_written)) < 0) {
      LOGE("pa_stream_begin_write failed(%s)", pa_strerror(pa_context_errno(pulse_context_)));
      current_stream->internal_stopstream();
    }

    bytes_read = current_stream->sndfile_transfer_data(buf_pulse, (sf_count_t)bytes_written);

    if (bytes_read > 0) {
      if (pa_stream_write(stream, buf_pulse, bytes_written, nullptr, 0, PA_SEEK_RELATIVE)) {
        LOGE("pa_stream_write failed(%s)\n", pa_strerror(pa_context_errno(pulse_context_)));
      }
    } else {
      pa_stream_cancel_write(stream);
    }

    if (bytes_read < bytes_written) {
      pa_operation * o;
      pa_stream_set_write_callback(stream, nullptr, nullptr);
      if (!(o = pa_stream_drain(stream, stream_drain_complete, current_stream))) {
        LOGE("pa_stream_drain failed(%s)", pa_strerror(pa_context_errno(pulse_context_)));
      }
      pa_operation_unref(o);
      break;
    }

    if (bytes_written >= length) {
      break;
    }
    length -= bytes_written;
  }
}

CaptureStream::CaptureStream(string filepath, std::shared_ptr<pa_sample_spec> sample_spec)
  : PulseCommonStream(filepath, sample_spec)
{
  if (!file_path.empty()) {
    if (sndfile_open()) {
      throw std::runtime_error("Open file failed");
    }
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
  pa_stream_flags_t flags = PA_STREAM_NOFLAGS;
  pa_cvolume volume;
  pa_stream * stream = get_stream_handle();

  LOGD("enter");

  if (stream == nullptr || get_stream_state() != PA_STREAM_UNCONNECTED) {
    LOGE("stream handle(%p) state(%d) error, start fail", static_cast<void *>(stream),
        get_stream_state());
    return -EPERM;
  }

  buffer_attr.maxlength = (uint32_t)-1;
  buffer_attr.fragsize =
      pa_usec_to_bytes(CAPTURE_FRAGMENT_DURATION_MS * PA_USEC_PER_MSEC, sample_spec.get());
  buffer_attr.prebuf = (uint32_t)-1;
  buffer_attr.tlength = (uint32_t)-1;
  buffer_attr.minreq = (uint32_t)-1;

  pa_cvolume_set(&volume, sample_spec->channels,
      PA_VOLUME_NORM * stream_volume / STREAM_VOL_MAX * RECORD_GAIN);

  if (pa_stream_connect_record(stream, nullptr, &buffer_attr, flags)) {
    LOGE("pa_stream_connect_record() failed: %s",
        pa_strerror(pa_context_errno(PulseCommonStream::pulse_context_)));
    return -EIO;
  }

  pa_threaded_mainloop_lock(PulseCommonStream::pulse_mainloop_);
  while (get_stream_state() != PA_STREAM_READY)
    pa_threaded_mainloop_wait(PulseCommonStream::pulse_mainloop_);
  pa_threaded_mainloop_unlock(PulseCommonStream::pulse_mainloop_);

  uint32_t stream_index = pa_stream_get_index(stream);
  pa_context_set_source_output_volume(
      PulseCommonStream::pulse_context_, stream_index, &volume, nullptr, nullptr);

  return 0;
}

int CaptureStream::sndfile_open()
{
  SF_INFO snd_file_info;
  bool match_format = false;

  snd_file_info.samplerate = sample_spec->rate;
  snd_file_info.channels = sample_spec->channels;

  for (auto it = capture_format_map.begin(); it != capture_format_map.end(); ++it) {
    if (it->first == (sample_spec->format)) {
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

  LOGI("opening %s", file_path.c_str());
  if ((file_fd_ = open(file_path.c_str(), O_WRONLY | O_TRUNC | O_CREAT, 0666)) < 0) {
    LOGE("open %s failed", file_path.c_str());
    return -ENOMEM;
  }

  snd_file = sf_open_fd(file_fd_, SFM_WRITE, &snd_file_info, 0);
  if (snd_file == nullptr) {
    LOGE("snd file open failed");
    return -SF_ERR_MALFORMED_FILE;
  }

  LOGD("open %s succeed", file_path.c_str());

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
    std::chrono::steady_clock::time_point read_start = std::chrono::steady_clock::now();

    if (pa_stream_peek(stream, &data, &length) < 0) {
      LOGE("pa_stream_peek() failed(%s)",
          pa_strerror(pa_context_errno(PulseCommonStream::pulse_context_)));
      current_stream->internal_stopstream();
    }

    assert(length > 0);

    if (current_stream->snd_file)
      bytes = current_stream->sndfile_transfer_data(data, length);

    if (current_stream->pcm_mode) {
      StreamEventData s_event_data;
      uint32_t stream_handle = IAudioStream::get_handle(current_stream);

      s_event_data.data_buf =
          std::make_shared<std::vector<uint8_t>>(static_cast<const uint8_t *>(data),
              static_cast<const uint8_t *>(data) + length);

      auto read_usec = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - read_start)
                            .count();
      s_event_data.usec = static_cast<uint64_t>(read_usec);

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
