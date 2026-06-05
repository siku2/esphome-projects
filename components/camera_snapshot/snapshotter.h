#pragma once

#include <esphome/components/camera/camera.h>
#include <esphome/core/component.h>

#include "decoder.h"
#include "esp_jpeg_common.h"

namespace esphome::camera::snapshot {

class SnapshotListener {
 public:
  virtual bool is_listening() const;
  virtual void on_snapshot(const Snapshot &snapshot) {}
};

enum class Phase {
  IDLE,
  PRE_SNAPSHOT,
  WAITING_FOR_IMAGE,
  POST_SNAPSHOT,
  NOTIFYING,
  ON_SNAPSHOT,
};

class Snapshotter : public PollingComponent, public CameraListener {
 public:
  void set_rotate(jpeg_rotate_t rotate) { this->rotate_ = rotate; }
  void set_drain_frame_buffer_count(size_t count) { this->drain_frame_buffer_count_ = count; }

  Trigger<> *get_pre_snapshot_trigger() { return &pre_snapshot_; }
  Trigger<> *get_post_snapshot_trigger() { return &post_snapshot_; }
  Trigger<const Snapshot &> *get_on_snapshot_trigger() { return &on_snapshot_; }

  void add_listener(SnapshotListener *listener) { this->listeners_.push_back(listener); }

 protected:
  static Decoder global_decoder;

  Camera *camera_{nullptr};
  jpeg_rotate_t rotate_{JPEG_ROTATE_0D};
  size_t drain_frame_buffer_count_{0};
  size_t remaining_drain_frame_buffer_count_{0};
  Phase phase_{Phase::IDLE};
  size_t notifying_index_{0};
  std::shared_ptr<CameraImage> pending_image_{nullptr};
  std::optional<Snapshot> snapshot_{};
  std::vector<SnapshotListener *> listeners_{};
  Trigger<> pre_snapshot_;
  Trigger<> post_snapshot_;
  Trigger<const Snapshot &> on_snapshot_;

  void setup() override;
  void dump_config() override;
  void loop() override;

  void update() override;

  void on_camera_image(const std::shared_ptr<CameraImage> &image) override;
  void on_stream_start() override;
  void on_stream_stop() override;

  bool notify_next();
};
}  // namespace esphome::camera::snapshot
