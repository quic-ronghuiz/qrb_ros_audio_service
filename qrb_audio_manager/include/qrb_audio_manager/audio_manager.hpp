// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_AUDIO_MANAGER__AUDIO_MANAGER_HPP_
#define QRB_AUDIO_MANAGER__AUDIO_MANAGER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "qrb_audio_manager/message_queue.hpp"
#include "qrb_audio_manager/stream.hpp"

namespace qrb
{
namespace audio_manager
{

// The interface provided by Audio Manager
class IAudioManager
{
public:
  virtual uint32_t create_playback_stream(std::string source,
      uint32_t sample_rate,
      uint8_t channels,
      uint8_t sample_format,
      std::string coding_format,
      uint8_t volume,
      std::string play_mode,
      int8_t repeat,
      std::string subs_name) = 0;
  virtual uint32_t create_record_stream(uint32_t sample_rate,
      uint8_t channels,
      uint8_t sample_format,
      std::string coding_format,
      std::string source,
      bool pub_pcm,
      std::string pub_name) = 0;
  virtual bool start_stream(uint32_t stream_handle) = 0;
  virtual bool stop_stream(uint32_t stream_handle) = 0;
  virtual bool release_stream(uint32_t stream_handle) = 0;
  virtual bool mute_stream(uint32_t stream_handle, bool mute) = 0;
  virtual bool write_stream(uint32_t stream_handle, const void * data, size_t size) = 0;
  virtual const std::map<std::string, std::string> & get_buildin_sounds() = 0;
  virtual void clean() = 0;
};

enum class AudioManagerCommand
{
  PLAY = 0,
  CREATE,
  START,
  STOP,
  MUTE,
  RELEASE,
  GETBUILDINSOUND
};

enum class AudioManagerPlayMode
{
  NORMAL = 1,
  ONE_TOUCH
};

extern std::map<std::string, AudioManagerCommand> audio_manager_cmd_name;

extern std::map<std::string, AudioManagerPlayMode> audio_manager_play_mode;

using stream_data_cb_t =
    std::function<void(uint32_t, std::shared_ptr<std::vector<uint8_t>>, uint64_t)>;

class AudioManager : public IAudioManager
{
public:
  static AudioManager * get_instance();
  uint32_t create_playback_stream(std::string source,
      uint32_t sample_rate,
      uint8_t channels,
      uint8_t sample_format,
      std::string coding_format,
      uint8_t volume,
      std::string play_mode,
      int8_t repeat,
      std::string subs_name) override;
  uint32_t create_record_stream(uint32_t sample_rate,
      uint8_t channels,
      uint8_t sample_format,
      std::string coding_format,
      std::string source,
      bool pub_pcm,
      std::string pub_name) override;
  bool start_stream(uint32_t stream_handle) override;
  bool stop_stream(uint32_t stream_handle) override;
  bool release_stream(uint32_t stream_handle) override;
  bool mute_stream(uint32_t stream_handle, bool mute) override;
  bool write_stream(uint32_t stream_handle, const void * data, size_t size) override;
  const std::map<std::string, std::string> & get_buildin_sounds() override;
  static void set_stream_data_callback(stream_data_cb_t cb);
  static void on_task_completed(int domain,
      uint32_t stream_handle,
      uint32_t audio_domain_handle,
      std::string cmd,
      bool result);
  void clean() { streams_.clear(); };

private:
  AudioManager() : worker_([this] { this->worker_thread(std::ref(queue_)); }) {}
  ~AudioManager()
  {
    queue_.stop();
    if (worker_.joinable())
      worker_.join();
  }
  static AudioManager * instance_;
  static stream_data_cb_t stream_data_cb_;
  std::unordered_map<uint32_t, std::shared_ptr<Stream>> streams_{};
  std::map<std::string, std::string> buildin_sounds_{};
  MessageQueue queue_;
  std::thread worker_;
  std::shared_ptr<Stream> find_stream(uint32_t stream_handle);
  uint32_t generate_key();
  void worker_thread(MessageQueue & queue);
  void load_buildin_sounds(const std::string & path);
};

}  // namespace audio_manager
}  // namespace qrb

#endif  // QRB_AUDIO_MANAGER__AUDIO_MANAGER_HPP_
