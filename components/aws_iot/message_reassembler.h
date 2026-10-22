#pragma once

#include <cstddef>
#include <string>

namespace esphome::aws_iot {

/// Joins the chunks of one incoming MQTT message. esp-mqtt splits messages that exceed its receive buffer.
class MessageReassembler {
 public:
  /// Largest message that is joined. Bigger ones are dropped.
  static constexpr size_t MAX_MESSAGE_SIZE = 128 * 1024;

  /// Adds one chunk. Returns true once the message is complete.
  /// `topic` is only valid in the first chunk (offset 0).
  bool add(const char *topic, size_t topic_len, const char *data, size_t data_len, size_t offset, size_t total_len);

  const std::string &topic() const { return this->topic_; }
  const std::string &payload() const { return this->payload_; }
  void reset();

 protected:
  std::string topic_;
  std::string payload_;
  bool active_{false};
};

}  // namespace esphome::aws_iot
