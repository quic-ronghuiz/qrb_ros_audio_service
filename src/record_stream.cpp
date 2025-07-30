// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#define LOG_TAG "RecordStream"

#include <sstream>

#include "qrb_audio_manager/stream.hpp"

namespace qrb
{
namespace audio_manager
{

RecordStream::RecordStream(uint32_t stream_handle,
    uint32_t sample_rate,
    uint8_t channels,
    uint8_t sample_format,
    std::string coding_format,
    std::string source,
    bool pub_pcm,
    std::string pub_name)
  : Stream(stream_handle, sample_rate, channels, sample_format, coding_format)
{
  AM_LOGD(LOG_TAG, "stream handle 0x" << (std::ostringstream() << std::hex << stream_handle).str()
                                      << ", source " << source << ", sample_rate " << sample_rate
                                      << ", channels " << static_cast<int>(channels)
                                      << ", sample_format " << static_cast<int>(sample_format)
                                      << ", coding_format " << coding_format << ", pub_pcm "
                                      << pub_pcm << ", pub_name " << pub_name);

  stream_configs_.type = StreamType::RECORD;
  stream_configs_.source = source;
  stream_configs_.pub_pcm = pub_pcm;
  stream_configs_.pub_name = pub_name;

  stream_configs_.state = StreamState::INIT;
}

}  // namespace audio_manager
}  // namespace qrb