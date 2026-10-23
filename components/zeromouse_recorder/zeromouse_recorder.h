#pragma once

#ifdef USE_ESP32

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "esphome/components/burst_camera/burst_camera.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/s3/s3.h"
#include "esphome/components/zeromouse/zeromouse.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

namespace esphome::zeromouse_recorder {

/// Uploads every event with its frames and metadata to an S3 bucket as training data.
class ZeroMouseRecorder : public Component {
 public:
  void set_s3(s3::S3Client *s3) { this->s3_ = s3; }
  void set_controller(zeromouse::ZeroMouse *controller) { this->controller_ = controller; }
  void set_camera(burst_camera::BurstCamera *camera) { this->camera_ = camera; }
  void set_enabled(switch_::Switch *enabled) { this->enabled_ = enabled; }
  void set_secondary_jpeg_quality(uint8_t quality) { this->secondary_quality_ = quality; }
  void set_max_pending(uint8_t max_pending) { this->max_pending_ = max_pending; }
  /// Limit for the image bytes held by pending events.
  void set_max_pending_bytes(uint32_t bytes) { this->max_pending_bytes_ = bytes; }
  void set_board(const std::string &board) { this->board_ = board; }

  /// Arguments are the event folder, the cloud event id (0 if not published) and the class name (empty if unknown).
  Trigger<std::string, uint32_t, std::string> *get_recorded_trigger() { return &this->recorded_trigger_; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

 protected:
  enum class Phase : uint8_t { AWAITING_PUBLISH, AWAITING_CLASSIFICATION, QUEUED, UPLOADING, BACKOFF, DONE };

  struct Frame {
    int32_t ms;
    uint32_t bytes;
    uint32_t bytes_hq;
  };

  struct Air {
    int32_t v;
    int32_t t;
    int32_t s;
    int32_t e;
  };

  struct Body {
    RAMUniquePtr<uint8_t[]> data;
    size_t size{0};
  };

  struct Step {
    enum class Kind : uint8_t { FRAME, FRAME_HQ, JSON } kind;
    uint8_t index;
  };

  struct Record {
    uint32_t seq;
    Phase phase{Phase::AWAITING_PUBLISH};
    std::vector<std::shared_ptr<burst_camera::BurstCameraImage>> images;
    std::vector<Frame> frames;
    std::vector<Air> air;
    zeromouse::ScanParams settings;
    zeromouse::ScanResult scan;
    std::string scan_result;
    int32_t ambient;
    uint16_t free_value_raw;
    int64_t time_zero_us;
    int32_t pir_to_t0_ms;
    time_t epoch;
    uint8_t quality_hq;
    bool published{false};
    uint32_t cloud_event_id{0};
    bool has_class{false};
    std::string class_name;
    int class_index{0};
    bool block{false};
    std::string folder;
    uint8_t attempts{0};
    bool dropped{false};
    bool resend{false};
    bool recorded{false};
    std::vector<Step> steps;
    std::shared_ptr<Body> body;
  };
  using RecordPtr = std::shared_ptr<Record>;

  bool is_enabled_() const { return this->enabled_ == nullptr || this->enabled_->state; }
  void update_quality_();
  void on_captured_(const zeromouse::CapturedEvent &event);
  void on_secondary_ready_();
  void on_published_(bool published, uint32_t event_id);
  void on_classification_(const std::string &class_name, int class_index, bool block);
  void name_folder_(Record &record);
  void queue_(const RecordPtr &record);
  void pump_();
  void start_attempt_(const RecordPtr &record);
  void run_step_(const RecordPtr &record, size_t index);
  void finish_attempt_(const RecordPtr &record, bool ok);
  void resend_json_(const RecordPtr &record);
  void remove_(const RecordPtr &record);
  void drop_record_(const RecordPtr &record);
  size_t record_bytes_(const Record &record) const;
  size_t drop_hq_(Record &record);
  void enforce_budget_(size_t reserve);
  void update_hq_(Record &record) const;
  static bool is_secondary_pending_(const Record &record);
  std::shared_ptr<Body> build_json_(const Record &record) const;
  std::string step_path_(const Record &record, const Step &step) const;

  s3::S3Client *s3_{nullptr};
  zeromouse::ZeroMouse *controller_{nullptr};
  burst_camera::BurstCamera *camera_{nullptr};
  switch_::Switch *enabled_{nullptr};
  std::string board_;
  uint8_t secondary_quality_{80};
  uint8_t max_pending_{3};
  uint32_t max_pending_bytes_{3000000};

  ESPPreferenceObject counter_pref_;
  uint32_t counter_{0};
  uint32_t next_seq_{1};
  std::deque<RecordPtr> records_;
  RecordPtr last_;
  RecordPtr in_flight_;

  Trigger<std::string, uint32_t, std::string> recorded_trigger_;
};

}  // namespace esphome::zeromouse_recorder

#endif
