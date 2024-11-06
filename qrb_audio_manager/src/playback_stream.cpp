// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#define LOG_TAG "PlaybackStream"

#include <sstream>

#include "qrb_audio_manager/stream.hpp"

namespace qrb
{
namespace audio_manager
{

PlaybackStream::PlaybackStream(uint32_t stream_handle,
    std::string source,
    std::string coding_format,
    uint8_t volume,
    std::string play_mode,
    int8_t repeat)
{
  AM_LOGD(LOG_TAG, "stream handle 0x" << (std::ostringstream() << std::hex << stream_handle).str()
                                      << ", source " << source << ", coding_format "
                                      << coding_format << ", volume " << static_cast<int>(volume)
                                      << ", play_mode " << play_mode << ", repeat "
                                      << static_cast<int>(repeat));

  stream_configs_.stream_handle = stream_handle;
  stream_configs_.type = StreamType::PLAYBACK;
  stream_configs_.source = source;
  stream_configs_.coding_format = coding_format;
  stream_configs_.volume = (volume == 0) ? 80 : volume;
  stream_configs_.play_mode = play_mode;
  stream_configs_.repeat = repeat;

  stream_configs_.state = StreamState::INIT;
}

bool PlaybackStream::set_mute(bool mute)
{
  bool ret = false;
  bool tmp_mute = stream_configs_.mute;
  auto domain_cb = get_domain_cb(DOMAIN_ID_AUDIO_COMMON);

  if (!domain_cb) {
    AM_LOGE(LOG_TAG, "domain cb is nullptr");
    return ret;
  }

  if (stream_configs_.state == StreamState::STARTED) {
    stream_configs_.mute = mute;
    auto domain_handle = get_domain_handle(DOMAIN_ID_AUDIO_COMMON);
    ret = domain_cb(
        static_cast<const void * const>(&stream_configs_), StreamCommand::MUTE, domain_handle);
  } else {
    stream_configs_.mute = tmp_mute;
    AM_LOGE(LOG_TAG, "state" << static_cast<int>(stream_configs_.state));
  }

  return ret;
}

}  // namespace audio_manager
}  // namespace qrb