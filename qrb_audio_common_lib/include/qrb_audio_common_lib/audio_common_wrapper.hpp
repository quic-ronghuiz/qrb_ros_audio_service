// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_AUDIO_COMMON__AUDIO_COMMON_WAPPER_HPP_
#define QRB_AUDIO_COMMON__AUDIO_COMMON_WAPPER_HPP_

#include <pulse/pulseaudio.h>
#include <sndfile.h>

#include <string>

#include "qrb_audio_common_lib/audio_common.hpp"
#include "qrb_audio_common_lib/audio_common_wrapper.hpp"

using namespace std;

namespace qrb
{
namespace audio_common_lib
{

uint32_t audio_stream_open(const audio_stream_info & stream_info,
    stream_event_callback_func event_callback);
int audio_stream_start(uint32_t stream_handle);
int audio_stream_mute(uint32_t stream_handle, bool mute);
int audio_stream_stop(uint32_t stream_handle);
int audio_stream_close(uint32_t stream_handle);
size_t audio_stream_write(uint32_t stream_handle, size_t length, void * buf);

}  // namespace audio_common_lib
}  // namespace qrb

#endif