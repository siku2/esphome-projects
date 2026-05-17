#pragma once

#include <esphome/components/camera/camera.h>
#include <esphome/core/component.h>

#include "decoder.h"

namespace esphome::camera::snapshot {
enum class Phase {
  IDLE,
  PRE_SNAPSHOT,
  WAITING_FOR_IMAGE,
  POST_SNAPSHOT,
  ON_SNAPSHOT,
};

class Snapshotter : public PollingComponent, public CameraListener {
 public:
  Trigger<> *get_pre_snapshot_trigger() { return &pre_snapshot_; }
  Trigger<> *get_post_snapshot_trigger() { return &post_snapshot_; }
  Trigger<const Snapshot &> *get_on_snapshot_trigger() { return &on_snapshot_; }

 protected:
  static Decoder global_decoder;

  Camera *camera_{nullptr};
  Phase phase_{Phase::IDLE};
  std::shared_ptr<CameraImage> pending_image_{nullptr};
  std::optional<Snapshot> snapshot_{};
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
};
}  // namespace esphome::camera::snapshot
