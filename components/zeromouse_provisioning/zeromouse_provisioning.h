#pragma once

#ifdef USE_ESP32

#include <cstdint>
#include <string>

#include "esphome/components/aws_iot/aws_iot.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/s3/s3.h"
#include "esphome/components/zeromouse/zeromouse.h"
#include "esphome/components/zeromouse_cloud/zeromouse_cloud.h"
#include "esphome/core/component.h"

namespace esphome::zeromouse_provisioning {

/// Keeps the cloud and S3 credentials in NVS so that the firmware image holds none. They cannot be read back.
class ZeroMouseProvisioning : public Component {
 public:
  void set_aws_iot(aws_iot::AwsIotClient *aws_iot) { this->aws_iot_ = aws_iot; }
  void set_cloud(zeromouse_cloud::ZeroMouseCloud *cloud) { this->cloud_ = cloud; }
  void set_controller(zeromouse::ZeroMouse *controller) { this->controller_ = controller; }
  void set_s3(s3::S3Client *s3) { this->s3_ = s3; }
  void set_cloud_provisioned_sensor(binary_sensor::BinarySensor *sensor) { this->cloud_provisioned_ = sensor; }
  void set_s3_provisioned_sensor(binary_sensor::BinarySensor *sensor) { this->s3_provisioned_ = sensor; }

  void setup() override;
  void dump_config() override;
  /// Runs before the components that use the credentials.
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  /// Stores the credentials and restarts. The counters are ignored if one of them is negative.
  bool provision_cloud(const std::string &thing_name, const std::string &certificate, const std::string &private_key,
                       const std::string &owner_id, const std::string &identity_id, int32_t event_count,
                       int32_t boot_count);
  /// Stores the S3 credentials and restarts.
  bool provision_s3(const std::string &access_key_id, const std::string &secret_access_key);
  /// Removes all stored credentials and restarts.
  void clear();

 protected:
  void restart_();

  aws_iot::AwsIotClient *aws_iot_{nullptr};
  zeromouse_cloud::ZeroMouseCloud *cloud_{nullptr};
  zeromouse::ZeroMouse *controller_{nullptr};
  s3::S3Client *s3_{nullptr};
  binary_sensor::BinarySensor *cloud_provisioned_{nullptr};
  binary_sensor::BinarySensor *s3_provisioned_{nullptr};
  bool has_cloud_{false};
  bool has_s3_{false};
};

}  // namespace esphome::zeromouse_provisioning

#endif
