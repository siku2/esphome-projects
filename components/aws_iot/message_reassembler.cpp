#include "message_reassembler.h"

#include "esphome/core/log.h"

namespace esphome::aws_iot {

static const char *const TAG = "aws_iot";

bool MessageReassembler::add(const char *topic, size_t topic_len, const char *data, size_t data_len, size_t offset,
                             size_t total_len) {
  if (offset == 0) {
    this->reset();
    if (total_len > MAX_MESSAGE_SIZE) {
      ESP_LOGW(TAG, "Dropping %u byte message, the limit is %u", static_cast<unsigned>(total_len),
               static_cast<unsigned>(MAX_MESSAGE_SIZE));
      return false;
    }
    this->topic_.assign(topic, topic_len);
    this->payload_.reserve(total_len);
    this->active_ = true;
  }
  if (!this->active_ || offset != this->payload_.size()) {
    this->reset();
    return false;
  }

  this->payload_.append(data, data_len);
  if (this->payload_.size() < total_len)
    return false;
  this->active_ = false;
  return true;
}

void MessageReassembler::reset() {
  this->topic_.clear();
  this->payload_.clear();
  this->active_ = false;
}

}  // namespace esphome::aws_iot
