// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#define LOG_TAG "AudioManager"

// clang-format off
#include <string>
#include <random>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>

#include "qrb_audio_manager/stream.hpp"
#include "qrb_audio_manager/audio_manager.hpp"
// clang-format on

#define BUILDIN_SOUNDS_PATH "/opt/qcom/qirp-sdk/usr/share/qrb-audio-manager/sounds"

uint32_t am_log_lvl = (AM_ERROR | AM_INFO | AM_DEBUG);

namespace qrb
{
namespace audio_manager
{

// clang-format off
std::map<std::string, AudioManagerCommand> audio_manager_cmd_name {
  { "play", AudioManagerCommand::PLAY },
  { "create", AudioManagerCommand::CREATE },
  { "start", AudioManagerCommand::START },
  { "stop", AudioManagerCommand::STOP },
  { "release", AudioManagerCommand::RELEASE },
  { "mute", AudioManagerCommand::MUTE },
  { "get-buildin-sound", AudioManagerCommand::GETBUILDINSOUND }
};

std::map<std::string, AudioManagerPlayMode> audio_manager_play_mode {
  { "normal", AudioManagerPlayMode::NORMAL },
  { "one-touch", AudioManagerPlayMode::ONE_TOUCH }
};
// clang-format on

AudioManager * AudioManager::instance_ = nullptr;

AudioManager * AudioManager::get_instance()
{
  if (instance_ == nullptr) {
    instance_ = new AudioManager();
    instance_->load_buildin_sounds(BUILDIN_SOUNDS_PATH);
  }

  return instance_;
}

uint32_t AudioManager::create_playback_stream(std::string source,
    uint32_t sample_rate,
    uint8_t channels,
    uint8_t sample_format,
    std::string coding_format,
    uint8_t volume,
    std::string play_mode,
    int8_t repeat,
    std::string subs_name)
{
  auto stream_handle = generate_key();
  while (streams_.find(stream_handle) != streams_.end()) {
    stream_handle = generate_key();
  }

  auto it = buildin_sounds_.find(source);
  if (it != buildin_sounds_.end())
    source = it->second;

  std::shared_ptr<Stream> stream_ptr = std::make_shared<PlaybackStream>(stream_handle, source,
      sample_rate, channels, sample_format, coding_format, volume, play_mode, repeat, subs_name);

  if (!stream_ptr->open()) {
    stream_ptr = nullptr;
    throw std::runtime_error("create playback stream failed");
  }

  streams_[stream_handle] = stream_ptr;

  AM_LOGI(LOG_TAG, "create playback stream, handle 0x"
                       << (std::ostringstream() << std::hex << stream_handle).str() << " success");

  try {
    if (!stream_ptr->get_domain_async_mode(DOMAIN_ID_AUDIO_COMMON))
      on_task_completed(DOMAIN_ID_AUDIO_COMMON, stream_handle,
          stream_ptr->get_domain_handle(DOMAIN_ID_AUDIO_COMMON),
          audio_stream_cmd_name[StreamCommand::OPEN], true);
  } catch (const std::exception & e) {
    AM_LOGE(LOG_TAG, "caught exception: " << e.what());
  }

  return stream_handle;
}

uint32_t AudioManager::create_record_stream(uint32_t sample_rate,
    uint8_t channels,
    uint8_t sample_format,
    std::string coding_format,
    std::string source,
    bool pub_pcm,
    std::string pub_name)
{
  auto stream_handle = generate_key();
  while (streams_.find(stream_handle) != streams_.end()) {
    stream_handle = generate_key();
  }

  std::shared_ptr<Stream> stream_ptr = std::make_shared<RecordStream>(stream_handle, sample_rate,
      channels, sample_format, coding_format, source, pub_pcm, pub_name);

  if (!stream_ptr->open()) {
    stream_ptr = nullptr;
    throw std::runtime_error("create record stream failed");
  }

  streams_[stream_handle] = stream_ptr;

  AM_LOGI(LOG_TAG, "create record streaml, handle 0x" +
                       (std::ostringstream() << std::hex << stream_handle).str() + " success");

  return stream_handle;
}

bool AudioManager::start_stream(uint32_t stream_handle)
{
  auto ret = false;

  auto stream = find_stream(stream_handle);
  if (stream) {
    AM_LOGD(LOG_TAG,
        "start stream handle 0x" << (std::ostringstream() << std::hex << stream_handle).str());
    ret = stream->start();
  }

  if (!ret) {
    streams_.erase(stream_handle);
    AM_LOGE(LOG_TAG,
        "erase stream, handle 0x" << (std::ostringstream() << std::hex << stream_handle).str());
  }

  return ret;
}

bool AudioManager::stop_stream(uint32_t stream_handle)
{
  auto ret = false;

  auto stream = find_stream(stream_handle);
  if (stream) {
    AM_LOGD(LOG_TAG,
        "stop stream handle 0x" << (std::ostringstream() << std::hex << stream_handle).str());
    ret = stream->stop();

    try {
      if (!stream->get_domain_async_mode(DOMAIN_ID_AUDIO_COMMON))
        on_task_completed(DOMAIN_ID_AUDIO_COMMON, stream_handle,
            stream->get_domain_handle(DOMAIN_ID_AUDIO_COMMON),
            audio_stream_cmd_name[StreamCommand::STOP], ret);
    } catch (const std::exception & e) {
      AM_LOGE(LOG_TAG, "caught exception: " << e.what());
    }

    if (!ret) {
      streams_.erase(stream_handle);
      AM_LOGE(LOG_TAG,
          "erase stream, handle 0x" << (std::ostringstream() << std::hex << stream_handle).str());
    }
  }

  return ret;
}

bool AudioManager::release_stream(uint32_t stream_handle)
{
  auto ret = false;

  auto stream = find_stream(stream_handle);
  if (stream) {
    AM_LOGD(LOG_TAG,
        "release stream handle 0x" << (std::ostringstream() << std::hex << stream_handle).str());
    ret = stream->close();
  }

  streams_.erase(stream_handle);

  return ret;
}

bool AudioManager::mute_stream(uint32_t stream_handle, bool mute)
{
  auto ret = false;

  auto stream = find_stream(stream_handle);
  if (stream && (stream->get_stream_configs().type == StreamType::PLAYBACK) &&
      (stream->get_stream_configs().mute) != mute) {
    AM_LOGD(LOG_TAG,
        "mute stream handle 0x" << (std::ostringstream() << std::hex << stream_handle).str());
    auto playback_stream = std::static_pointer_cast<PlaybackStream>(stream);
    ret = playback_stream->set_mute(mute);
  }

  return ret;
}

std::shared_ptr<Stream> AudioManager::find_stream(uint32_t stream_handle)
{
  auto stream = streams_.find(stream_handle);
  auto stream_handle_str = (std::ostringstream() << std::hex << stream_handle).str();

  if (stream != streams_.end()) {
    AM_LOGV(LOG_TAG, "find stream handle 0x" << stream_handle_str);
    return stream->second;
  }

  return nullptr;
}

uint32_t AudioManager::generate_key()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> dist(1, UINT32_MAX);

  return dist(gen);
}

void AudioManager::worker_thread(MessageQueue & queue)
{
  struct as_message msg;

  while (queue.pop(msg)) {
    auto stream_handle = msg.stream_handle;
    auto stream_handle_str = (std::ostringstream() << std::hex << stream_handle).str();

    auto cmd = msg.command;
    auto cmd_str = audio_stream_cmd_name[static_cast<StreamCommand>(cmd)];

    AM_LOGI(LOG_TAG, "worker thread received message, stream handle 0x" << stream_handle_str
                                                                        << ", command " << cmd_str);

    auto stream = find_stream(stream_handle);
    if (!stream) {
      AM_LOGE(LOG_TAG, "invalid stream handle 0x" << stream_handle_str);
      continue;
    }

    switch (static_cast<StreamCommand>(cmd)) {
      case StreamCommand::START:
        start_stream(stream_handle);
        break;
      case StreamCommand::STOP:
        stop_stream(stream_handle);
        break;
      case StreamCommand::CLOSE:
        release_stream(stream_handle);
        break;
      default:
        AM_LOGE(LOG_TAG, "unsupport command " << cmd_str);
        break;
    }
  }
}

void AudioManager::on_task_completed(int domain,
    uint32_t stream_handle,
    uint32_t audio_domain_handle,
    std::string cmd,
    bool result)
{
  std::shared_ptr<Stream> stream = nullptr;

  auto stream_handle_str = (std::ostringstream() << std::hex << stream_handle).str();
  auto audio_domain_handle_str = (std::ostringstream() << std::hex << audio_domain_handle).str();

  AM_LOGI(LOG_TAG, "domain " << domain << ", command " << cmd << ", result " << result
                             << ", stream_handle 0x" << stream_handle_str
                             << ", audio_domain_handle 0x" << audio_domain_handle_str);

  switch (domain) {
    case DOMAIN_ID_AUDIO_COMMON:
      if (audio_stream_cmd_name[StreamCommand::OPEN] == cmd) {
        for (const auto & kv : instance_->streams_) {
          if (kv.first == stream_handle) {
            stream = kv.second;
            break;
          }
        }

        if (!stream) {
          AM_LOGE(LOG_TAG, "invalid stream handle 0x" << stream_handle_str);
          return;
        }
        if (!stream->get_domain_handle(DOMAIN_ID_AUDIO_COMMON)) {
          stream->set_domain_handle(DOMAIN_ID_AUDIO_COMMON, audio_domain_handle);
          AM_LOGI(LOG_TAG, "set audio common handle 0x" << audio_domain_handle_str
                                                        << " for stream handle 0x"
                                                        << stream_handle_str);
        }
      }

      if (!stream) {
        for (const auto & kv : instance_->streams_) {
          if (kv.second->get_domain_handle(DOMAIN_ID_AUDIO_COMMON) == audio_domain_handle) {
            stream_handle = kv.first;
            stream_handle_str = (std::ostringstream() << std::hex << stream_handle).str();
            stream = kv.second;
            AM_LOGV(LOG_TAG, "found stream, handle is 0x" << stream_handle_str);
            break;
          }
        }
      }

      if (!result) {
        instance_->streams_.erase(stream_handle);
        AM_LOGE(LOG_TAG, "erase stream, handle 0x" << stream_handle_str);
        return;
      }

      if (stream && (stream->get_stream_configs().play_mode == "one-touch")) {
        struct as_message msg;
        msg.stream_handle = stream_handle;

        if (audio_stream_cmd_name[StreamCommand::OPEN] == cmd)
          msg.command = StreamCommand::START;
        else if (audio_stream_cmd_name[StreamCommand::START] == cmd)
          msg.command = StreamCommand::STOP;
        else if (audio_stream_cmd_name[StreamCommand::STOP] == cmd)
          msg.command = StreamCommand::CLOSE;
        else
          break;

        AM_LOGI(LOG_TAG, "handle one touch playback for stream handle 0x"
                             << stream_handle_str << " next command is "
                             << audio_stream_cmd_name[msg.command]);

        instance_->queue_.push(msg);
      }
      break;
    default:
      AM_LOGE(LOG_TAG, "unsupport domain " << domain);
      break;
  }
}

const std::map<std::string, std::string> & AudioManager::get_buildin_sounds()
{
  return buildin_sounds_;
}

void AudioManager::load_buildin_sounds(const std::string & path)
{
  DIR * dirp = opendir(path.c_str());
  if (dirp == nullptr) {
    return;
  }

  struct dirent * dp;
  while ((dp = readdir(dirp)) != nullptr) {
    std::string filename(dp->d_name);

    if (filename == "." || filename == "..") {
      continue;
    }

    size_t last_dot = filename.find_last_of(".");
    if (last_dot != std::string::npos) {
      filename = filename.substr(0, last_dot);
    }

    std::string filepath = path + "/" + dp->d_name;
    buildin_sounds_[filename] = filepath;
  }

  closedir(dirp);
}

}  // namespace audio_manager
}  // namespace qrb