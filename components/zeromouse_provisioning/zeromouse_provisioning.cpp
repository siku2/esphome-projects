#ifdef USE_ESP32

#include "zeromouse_provisioning.h"

#include <esp_random.h>
#include <mbedtls/pk.h>
#include <mbedtls/x509_crt.h>
#include <nvs.h>

#include <initializer_list>
#include <utility>

#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

namespace esphome::zeromouse_provisioning {

static const char *const TAG = "zeromouse_provisioning";
static const char *const NAMESPACE = "zm_prov";
static const char *const KEY_THING = "thing";
static const char *const KEY_CERT = "cert";
static const char *const KEY_KEY = "key";
static const char *const KEY_OWNER = "owner";
static const char *const KEY_IDENTITY = "identity";
static const char *const KEY_S3_ID = "s3_id";
static const char *const KEY_S3_SECRET = "s3_secret";
static constexpr uint32_t RESTART_DELAY_MS = 1000;

namespace {

int fill_random(void *, unsigned char *output, size_t length) {
  esp_fill_random(output, length);
  return 0;
}

std::string normalize_pem(const std::string &pem) {
  const size_t first = pem.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
    return "";
  const size_t last = pem.find_last_not_of(" \t\r\n");
  return pem.substr(first, last - first + 1) + "\n";
}

bool valid_thing_name(const std::string &name) {
  for (const char c : name) {
    const bool ok =
        (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == ':' || c == '-';
    if (!ok)
      return false;
  }
  return true;
}

bool read_string(nvs_handle_t handle, const char *key, std::string &out) {
  size_t length = 0;
  if (nvs_get_str(handle, key, nullptr, &length) != ESP_OK || length <= 1)
    return false;
  std::string value(length, '\0');
  if (nvs_get_str(handle, key, value.data(), &length) != ESP_OK)
    return false;
  value.resize(length - 1);
  out = std::move(value);
  return true;
}

bool write_strings(std::initializer_list<std::pair<const char *, const std::string *>> items) {
  nvs_handle_t handle;
  if (nvs_open(NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS");
    return false;
  }
  esp_err_t err = ESP_OK;
  for (const auto &item : items) {
    err = nvs_set_str(handle, item.first, item.second->c_str());
    if (err != ESP_OK)
      break;
  }
  if (err == ESP_OK)
    err = nvs_commit(handle);
  nvs_close(handle);
  if (err != ESP_OK)
    ESP_LOGE(TAG, "Failed to write to NVS: %s", esp_err_to_name(err));
  return err == ESP_OK;
}

bool check_certificate_and_key(const std::string &certificate, const std::string &private_key) {
  mbedtls_x509_crt crt;
  mbedtls_pk_context pk;
  mbedtls_x509_crt_init(&crt);
  mbedtls_pk_init(&pk);
  bool ok = false;
  // The length of a PEM buffer includes the terminating NUL.
  if (mbedtls_x509_crt_parse(&crt, reinterpret_cast<const unsigned char *>(certificate.c_str()),
                             certificate.size() + 1) != 0) {
    ESP_LOGE(TAG, "The certificate is not valid PEM");
  } else if (mbedtls_pk_parse_key(&pk, reinterpret_cast<const unsigned char *>(private_key.c_str()),
                                  private_key.size() + 1, nullptr, 0, fill_random, nullptr) != 0) {
    ESP_LOGE(TAG, "The private key is not valid PEM");
  } else if (mbedtls_pk_check_pair(&crt.pk, &pk, fill_random, nullptr) != 0) {
    ESP_LOGE(TAG, "The private key does not match the certificate");
  } else {
    ok = true;
  }
  mbedtls_pk_free(&pk);
  mbedtls_x509_crt_free(&crt);
  return ok;
}

}  // namespace

void ZeroMouseProvisioning::setup() {
  std::string thing, cert, key, owner, identity, s3_id, s3_secret;
  nvs_handle_t handle;
  if (nvs_open(NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
    read_string(handle, KEY_THING, thing);
    read_string(handle, KEY_CERT, cert);
    read_string(handle, KEY_KEY, key);
    read_string(handle, KEY_OWNER, owner);
    read_string(handle, KEY_IDENTITY, identity);
    read_string(handle, KEY_S3_ID, s3_id);
    read_string(handle, KEY_S3_SECRET, s3_secret);
    nvs_close(handle);
  }

  this->has_cloud_ = !thing.empty() && !cert.empty() && !key.empty() && !owner.empty() && !identity.empty();
  if (this->has_cloud_) {
    this->aws_iot_->set_client_id(thing);
    this->aws_iot_->set_certificate(cert);
    this->aws_iot_->set_private_key(key);
    this->cloud_->set_thing_name(thing);
    this->cloud_->set_owner_id(owner);
    this->cloud_->set_identity_id(identity);
  }
  this->has_s3_ = this->s3_ != nullptr && !s3_id.empty() && !s3_secret.empty();
  if (this->has_s3_) {
    this->s3_->set_access_key_id(s3_id);
    this->s3_->set_secret_access_key(s3_secret);
  }

  if (this->cloud_provisioned_ != nullptr)
    this->cloud_provisioned_->publish_state(this->has_cloud_);
  if (this->s3_provisioned_ != nullptr)
    this->s3_provisioned_->publish_state(this->has_s3_);
}

void ZeroMouseProvisioning::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ZeroMOUSE Provisioning:\n"
                "  Cloud credentials: %s\n"
                "  S3 credentials: %s",
                YESNO(this->has_cloud_), YESNO(this->has_s3_));
}

bool ZeroMouseProvisioning::provision_cloud(const std::string &thing_name, const std::string &certificate,
                                            const std::string &private_key, const std::string &owner_id,
                                            const std::string &identity_id, int32_t event_count, int32_t boot_count) {
  const std::string cert = normalize_pem(certificate);
  const std::string key = normalize_pem(private_key);
  if (thing_name.empty() || cert.empty() || key.empty() || owner_id.empty() || identity_id.empty()) {
    ESP_LOGE(TAG, "A cloud credential is empty");
    return false;
  }
  if (!valid_thing_name(thing_name)) {
    ESP_LOGE(TAG, "The thing name has invalid characters");
    return false;
  }
  if (!check_certificate_and_key(cert, key))
    return false;
  if (!write_strings({{KEY_THING, &thing_name},
                      {KEY_CERT, &cert},
                      {KEY_KEY, &key},
                      {KEY_OWNER, &owner_id},
                      {KEY_IDENTITY, &identity_id}}))
    return false;
  if (event_count >= 0 && boot_count >= 0)
    this->cloud_->store_counters(static_cast<uint32_t>(event_count), static_cast<uint32_t>(boot_count));
  ESP_LOGI(TAG, "Cloud credentials stored, restarting");
  this->restart_();
  return true;
}

bool ZeroMouseProvisioning::provision_s3(const std::string &access_key_id, const std::string &secret_access_key) {
  if (access_key_id.empty() || secret_access_key.empty()) {
    ESP_LOGE(TAG, "An S3 credential is empty");
    return false;
  }
  if (!write_strings({{KEY_S3_ID, &access_key_id}, {KEY_S3_SECRET, &secret_access_key}}))
    return false;
  ESP_LOGI(TAG, "S3 credentials stored, restarting");
  this->restart_();
  return true;
}

void ZeroMouseProvisioning::clear() {
  nvs_handle_t handle;
  esp_err_t err = nvs_open(NAMESPACE, NVS_READWRITE, &handle);
  if (err == ESP_OK) {
    err = nvs_erase_all(handle);
    if (err == ESP_OK)
      err = nvs_commit(handle);
    nvs_close(handle);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to clear NVS: %s", esp_err_to_name(err));
    return;
  }
  ESP_LOGI(TAG, "Credentials cleared, restarting");
  this->restart_();
}

void ZeroMouseProvisioning::restart_() {
  global_preferences->sync();
  this->controller_->abort_event("provisioning");
  this->set_timeout(RESTART_DELAY_MS, []() { App.safe_reboot(); });
}

}  // namespace esphome::zeromouse_provisioning

#endif
