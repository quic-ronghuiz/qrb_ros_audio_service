// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_AUDIO_MANAGER__AM_LOG_HPP_
#define QRB_AUDIO_MANAGER__AM_LOG_HPP_

#include <cstdint>

extern uint32_t am_log_lvl;

namespace qrb
{
namespace audio_manager
{

#define AM_ERROR (0x1)
#define AM_INFO (0x2)
#define AM_DEBUG (0x4)
#define AM_VERBOSE (0x8)

#define AM_LOGV(tag, message)                                                                      \
  if (am_log_lvl & AM_VERBOSE) {                                                                   \
    std::ostringstream oss;                                                                        \
    oss << tag << ": " << __func__ << ": " << __LINE__ << ": " << message;                         \
    std::cout << oss.str() << std::endl;                                                           \
  }

#define AM_LOGD(tag, message)                                                                      \
  if (am_log_lvl & AM_DEBUG) {                                                                     \
    std::ostringstream oss;                                                                        \
    oss << tag << ": " << __func__ << ": " << __LINE__ << ": " << message;                         \
    std::cout << oss.str() << std::endl;                                                           \
  }

#define AM_LOGI(tag, message)                                                                      \
  if (am_log_lvl & AM_INFO) {                                                                      \
    std::ostringstream oss;                                                                        \
    oss << tag << ": " << __func__ << ": " << __LINE__ << ": " << message;                         \
    std::cout << oss.str() << std::endl;                                                           \
  }

#define AM_LOGE(tag, message)                                                                      \
  if (am_log_lvl & AM_ERROR) {                                                                     \
    std::ostringstream oss;                                                                        \
    oss << tag << ": " << __func__ << ": " << __LINE__ << ": " << message;                         \
    std::cout << oss.str() << std::endl;                                                           \
  }

}  // namespace audio_manager
}  // namespace qrb

#endif  // QRB_AUDIO_MANAGER__AM_LOG_HPP_