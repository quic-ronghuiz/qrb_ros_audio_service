// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_AUDIO_COMMON_LIB__AUDIO_LOG_HPP_
#define QRB_AUDIO_COMMON_LIB__AUDIO_LOG_HPP_

#include <cstdio>

namespace qrb
{
namespace audio_common_lib
{

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

}  // namespace audio_common_lib
}  // namespace qrb

#endif  // QRB_AUDIO_COMMON_LIB__AUDIO_LOG_HPP_
