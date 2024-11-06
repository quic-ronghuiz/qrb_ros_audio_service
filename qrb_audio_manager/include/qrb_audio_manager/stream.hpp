// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_AUDIO_MANAGER__STREAM_HPP_
#define QRB_AUDIO_MANAGER__STREAM_HPP_

#include <cstdint>
#include <functional>
#include <iostream>
#include <map>

#include "qrb_audio_manager/am_log.hpp"

namespace qrb
{
namespace audio_manager
{

// Invalid domain
#define DOMAIN_ID_INVALID -1

// Audio common domain
#define DOMAIN_ID_AUDIO_COMMON 0

// MAX domain
#define DOMAIN_ID_MAX 1

enum class StreamType
{
  RECORD = 0,
  PLAYBACK
};

enum class StreamState
{
  INVALID_STATE = -1,
  INIT,
  OPENED,
  STARTED,
  STOPPED,
  CLOSED
};

enum class StreamCommand
{
  OPEN = 0,
  START,
  STOP,
  CLOSE,
  MUTE
};

struct StreamConfigs
{
  uint32_t stream_handle{ 0 };
  StreamType type;
  uint8_t channels;
  uint32_t sample_rate;
  uint8_t sample_format;
  std::string coding_format;
  uint32_t bitrate;
  StreamState state{ StreamState::INVALID_STATE };
  std::string source;
  std::string play_mode;
  int8_t repeat{ 0 };
  uint8_t volume{ 50 };
  bool mute{ false };
  bool pub_pcm{ false };
};

struct DomainConfigs
{
  int domain_id{ DOMAIN_ID_INVALID };
  bool use_async{ false };
  std::function<bool(const void * const, StreamCommand, uint32_t &)> cb{ nullptr };
  uint32_t handle{ 0 };
};

extern std::map<StreamType, std::string> audio_stream_type_name;

extern std::map<StreamCommand, std::string> audio_stream_cmd_name;

class Stream
{
public:
  virtual bool open();
  virtual bool start();
  virtual bool stop();
  virtual bool close();
  const StreamConfigs & get_stream_configs() { return stream_configs_; };
  bool get_domain_async_mode(uint8_t domain_id);
  uint32_t get_domain_handle(uint8_t domain_id);
  void set_domain_handle(uint8_t domain_id, uint32_t handle);
  static void register_callback(int domain_id,
      std::function<bool(const void * const, StreamCommand, uint32_t &)> cb,
      bool use_async);

protected:
  StreamConfigs stream_configs_;
  std::function<bool(const void * const, StreamCommand, uint32_t &)> get_domain_cb(
      uint8_t domain_id);

private:
  uint32_t domain_handle_[DOMAIN_ID_MAX]{ 0 };
  static DomainConfigs domain_configs_[DOMAIN_ID_MAX];
};

class RecordStream : public Stream
{
public:
  RecordStream(uint32_t stream_handle,
      uint32_t sample_rate,
      uint8_t channels,
      uint8_t sample_format,
      std::string coding_format,
      std::string source,
      bool pub_pcm);
};

class PlaybackStream : public Stream
{
public:
  PlaybackStream(uint32_t stream_handle,
      std::string source,
      std::string coding_format,
      uint8_t volume,
      std::string play_mode,
      int8_t repeat);
  bool set_mute(bool mute);
};

}  // namespace audio_manager
}  // namespace qrb

#endif  // QRB_AUDIO_MANAGER__STREAM_HPP_