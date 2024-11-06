// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_AUDIO_MANAGER__INTERFACE_HPP_
#define QRB_AUDIO_MANAGER__INTERFACE_HPP_

#include <string>
#include <thread>
#include <unordered_map>

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
      std::string coding_format,
      uint8_t volume,
      std::string play_mode,
      int8_t repeat) = 0;
  virtual uint32_t create_record_stream(uint32_t sample_rate,
      uint8_t channels,
      uint8_t sample_format,
      std::string coding_format,
      std::string source,
      bool pub_pcm) = 0;
  virtual bool start_stream(uint32_t stream_handle) = 0;
  virtual bool stop_stream(uint32_t stream_handle) = 0;
  virtual bool release_stream(uint32_t stream_handle) = 0;
  virtual bool mute_stream(uint32_t stream_handle, bool mute) = 0;
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

class AudioManager : public IAudioManager
{
public:
  AudioManager() : worker_([this] { this->worker_thread(std::ref(queue_)); }) {}
  ~AudioManager()
  {
    queue_.stop();
    if (worker_.joinable())
      worker_.join();
  }
  static AudioManager * getInstance();
  uint32_t create_playback_stream(std::string source,
      std::string coding_format,
      uint8_t volume,
      std::string play_mode,
      int8_t repeat) override;
  uint32_t create_record_stream(uint32_t sample_rate,
      uint8_t channels,
      uint8_t sample_format,
      std::string coding_format,
      std::string source,
      bool pub_pcm) override;
  bool start_stream(uint32_t stream_handle) override;
  bool stop_stream(uint32_t stream_handle) override;
  bool release_stream(uint32_t stream_handle) override;
  bool mute_stream(uint32_t stream_handle, bool mute) override;
  const std::map<std::string, std::string> & get_buildin_sounds() override;
  static void on_task_completed(int domain,
      uint32_t stream_handle,
      uint32_t audio_domain_handle,
      std::string cmd,
      bool result);
  void clean() { streams_.clear(); };

private:
  static AudioManager * instance_;
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

#endif  // QRB_AUDIO_MANAGER__INTERFACE_HPP_