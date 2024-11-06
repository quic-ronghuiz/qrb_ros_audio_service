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
    bool pub_pcm)
{
  AM_LOGD(LOG_TAG, "stream handle 0x" << (std::ostringstream() << std::hex << stream_handle).str()
                                      << ", source " << source << ", sample_rate " << sample_rate
                                      << ", channels " << static_cast<int>(channels)
                                      << ", sample_format " << static_cast<int>(sample_format)
                                      << ", coding_format " << coding_format << ", pub_pcm "
                                      << pub_pcm);

  stream_configs_.stream_handle = stream_handle;
  stream_configs_.type = StreamType::RECORD;
  stream_configs_.sample_rate = sample_rate;
  stream_configs_.channels = channels;
  stream_configs_.sample_format = sample_format;
  stream_configs_.coding_format = coding_format;
  stream_configs_.source = source;
  stream_configs_.pub_pcm = pub_pcm;

  stream_configs_.state = StreamState::INIT;
}

}  // namespace audio_manager
}  // namespace qrb