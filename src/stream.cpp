// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#define LOG_TAG "Stream"

// clang-format off
#include <sstream>

#include "qrb_audio_manager/stream.hpp"
// clang-format on

namespace qrb
{
namespace audio_manager
{

// clang-format off
std::map<StreamType, std::string> audio_stream_type_name {
  { StreamType::RECORD, "record" },
  { StreamType::PLAYBACK, "playback" }
};

std::map<StreamCommand, std::string> audio_stream_cmd_name {
  { StreamCommand::OPEN, "open" },
  { StreamCommand::START, "start" },
  { StreamCommand::STOP, "stop" },
  { StreamCommand::CLOSE, "close" },
  { StreamCommand::MUTE, "mute" }
};
// clang-format on

DomainConfigs Stream::domain_configs_[DOMAIN_ID_MAX];

Stream::Stream(uint32_t stream_handle,
    uint32_t sample_rate,
    uint8_t channels,
    uint8_t sample_format,
    std::string coding_format)
{
  stream_configs_.stream_handle = stream_handle;
  stream_configs_.sample_rate = sample_rate;
  stream_configs_.channels = channels;
  stream_configs_.sample_format = sample_format;
  stream_configs_.coding_format = coding_format;
}

bool Stream::open()
{
  auto ret = false;
  auto domain_cb = get_domain_cb(DOMAIN_ID_AUDIO_COMMON);
  auto domain_handle = get_domain_handle(DOMAIN_ID_AUDIO_COMMON);

  if (!domain_cb) {
    AM_LOGE(LOG_TAG, "domain cb is nullptr");
    return ret;
  }

  if (stream_configs_.state == StreamState::INIT) {
    ret = domain_cb(
        static_cast<const void * const>(&stream_configs_), StreamCommand::OPEN, domain_handle);
  } else {
    AM_LOGE(LOG_TAG, "state " << static_cast<int>(stream_configs_.state));
  }

  if (ret) {
    set_domain_handle(DOMAIN_ID_AUDIO_COMMON, domain_handle);
    stream_configs_.state = StreamState::OPENED;
  }

  return ret;
}

bool Stream::start()
{
  auto ret = false;
  auto domain_cb = get_domain_cb(DOMAIN_ID_AUDIO_COMMON);

  if (!domain_cb) {
    AM_LOGE(LOG_TAG, "domain cb is nullptr");
    return ret;
  }

  if (stream_configs_.state == StreamState::OPENED) {
    auto domain_handle = get_domain_handle(DOMAIN_ID_AUDIO_COMMON);
    AM_LOGD(LOG_TAG, "start " << audio_stream_type_name[stream_configs_.type] +
                                     " stream, audio common handle is 0x"
                              << (std::ostringstream() << std::hex << domain_handle).str());
    ret = domain_cb(
        static_cast<const void * const>(&stream_configs_), StreamCommand::START, domain_handle);
  } else {
    AM_LOGE(LOG_TAG, "state " << static_cast<int>(stream_configs_.state));
  }

  if (ret)
    stream_configs_.state = StreamState::STARTED;

  return ret;
}

bool Stream::stop()
{
  auto ret = false;
  auto domain_cb = get_domain_cb(DOMAIN_ID_AUDIO_COMMON);

  if (!domain_cb) {
    AM_LOGE(LOG_TAG, "domain cb is nullptr");
    return ret;
  }

  if (stream_configs_.state == StreamState::STARTED) {
    auto domain_handle = get_domain_handle(DOMAIN_ID_AUDIO_COMMON);
    AM_LOGD(LOG_TAG, "stop " << audio_stream_type_name[stream_configs_.type] +
                                    " stream, audio common handle is 0x"
                             << (std::ostringstream() << std::hex << domain_handle).str());
    ret = domain_cb(
        static_cast<const void * const>(&stream_configs_), StreamCommand::STOP, domain_handle);
  } else {
    AM_LOGE(LOG_TAG, "state " << static_cast<int>(stream_configs_.state));
  }

  if (ret)
    stream_configs_.state = StreamState::STOPPED;

  return ret;
}

bool Stream::close()
{
  auto ret = false;
  auto domain_cb = get_domain_cb(DOMAIN_ID_AUDIO_COMMON);

  if (!domain_cb) {
    AM_LOGE(LOG_TAG, "domain cb is nullptr.");
    return ret;
  }

  if (stream_configs_.state == StreamState::STOPPED) {
    auto domain_handle = get_domain_handle(DOMAIN_ID_AUDIO_COMMON);
    AM_LOGD(LOG_TAG, "close " << audio_stream_type_name[stream_configs_.type] +
                                     " stream, audio common handle is 0x"
                              << (std::ostringstream() << std::hex << domain_handle).str());
    ret = domain_cb(
        static_cast<const void * const>(&stream_configs_), StreamCommand::CLOSE, domain_handle);
  } else {
    AM_LOGE(LOG_TAG, "state " << static_cast<int>(stream_configs_.state));
  }

  if (ret)
    stream_configs_.state = StreamState::CLOSED;

  return ret;
}

void Stream::register_callback(int domain_id,
    std::function<bool(const void * const, StreamCommand, uint32_t &)> cb,
    bool use_async)
{
  if ((domain_id <= DOMAIN_ID_INVALID) || (domain_id >= DOMAIN_ID_MAX)) {
    AM_LOGE(LOG_TAG, "invalid domain id");
    return;
  }

  for (int i = 0; i < DOMAIN_ID_MAX; i++) {
    if (domain_configs_[i].domain_id == DOMAIN_ID_INVALID) {
      domain_configs_[i].domain_id = domain_id;
      domain_configs_[i].cb = cb;
      domain_configs_[i].use_async = use_async;
      break;
    }
  }
}

std::function<bool(const void * const, StreamCommand, uint32_t &)> Stream::get_domain_cb(
    uint8_t domain_id)
{
  std::function<bool(const void * const, StreamCommand, uint32_t &)> cb = nullptr;

  if ((domain_id <= DOMAIN_ID_INVALID) || (domain_id >= DOMAIN_ID_MAX)) {
    AM_LOGE(LOG_TAG, "invalid domain id");
    return nullptr;
  }

  for (int i = 0; i < DOMAIN_ID_MAX; i++) {
    if (domain_configs_[i].domain_id == domain_id) {
      cb = domain_configs_[i].cb;
      break;
    }
  }

  return cb;
}

bool Stream::get_domain_async_mode(uint8_t domain_id)
{
  if ((domain_id <= DOMAIN_ID_INVALID) || (domain_id >= DOMAIN_ID_MAX))
    throw std::runtime_error("invalid domain id");

  for (int i = 0; i < DOMAIN_ID_MAX; i++) {
    if (domain_configs_[i].domain_id == domain_id)
      return domain_configs_[i].use_async;
  }

  throw std::runtime_error("failed to get domain async mode");
}

uint32_t Stream::get_domain_handle(uint8_t domain_id)
{
  if ((domain_id <= DOMAIN_ID_INVALID) || (domain_id >= DOMAIN_ID_MAX)) {
    AM_LOGE(LOG_TAG, "invalid domain id");
    return 0;
  }

  return domain_handle_[domain_id];
}

void Stream::set_domain_handle(uint8_t domain_id, uint32_t handle)
{
  if ((domain_id <= DOMAIN_ID_INVALID) || (domain_id >= DOMAIN_ID_MAX)) {
    AM_LOGE(LOG_TAG, "invalid domain id");
    return;
  }

  if (0 == handle) {
    AM_LOGE(LOG_TAG, "invalid handle 0");
    return;
  }

  domain_handle_[domain_id] = handle;
}

}  // namespace audio_manager
}  // namespace qrb