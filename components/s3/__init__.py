import re
from urllib.parse import urlparse

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
from esphome.const import CONF_ID
from esphome.types import ConfigType

DEPENDENCIES = ["esp32", "network"]

CONF_ENDPOINT = "endpoint"
CONF_BUCKET = "bucket"
CONF_REGION = "region"
CONF_ACCESS_KEY_ID = "access_key_id"
CONF_SECRET_ACCESS_KEY = "secret_access_key"
CONF_ROOT_CA = "root_ca"

s3_ns = cg.esphome_ns.namespace("s3")
S3Client = s3_ns.class_("S3Client", cg.Component)


_PEM_BEGIN = re.compile(r"-----BEGIN [A-Z0-9 ]+-----")
_PEM_END = re.compile(r"-----END [A-Z0-9 ]+-----")


def pem(value: object) -> str:
    value = cv.string_strict(value).strip()
    if not _PEM_BEGIN.search(value) or not _PEM_END.search(value):
        raise cv.Invalid("Expected a PEM block with BEGIN and END markers")
    return value + "\n"


def endpoint_url(value: object) -> str:
    value = cv.url(value)
    if urlparse(value).scheme not in ("http", "https"):
        raise cv.Invalid("Expected an http or https URL")
    return value.rstrip("/")


def _require_ca_for_https(config: ConfigType) -> ConfigType:
    if CONF_ROOT_CA in config and not config[CONF_ENDPOINT].startswith("https://"):
        raise cv.Invalid("root_ca only applies to https endpoints")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(S3Client),
            cv.Required(CONF_ENDPOINT): endpoint_url,
            cv.Required(CONF_BUCKET): cv.All(cv.string_strict, cv.Length(min=1)),
            cv.Optional(CONF_REGION, default="garage"): cv.All(
                cv.string_strict, cv.Length(min=1)
            ),
            cv.Required(CONF_ACCESS_KEY_ID): cv.All(cv.string_strict, cv.Length(min=1)),
            cv.Required(CONF_SECRET_ACCESS_KEY): cv.sensitive(
                cv.All(cv.string_strict, cv.Length(min=1))
            ),
            cv.Optional(CONF_ROOT_CA): pem,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _require_ca_for_https,
    cv.only_with_framework("esp-idf"),
)


async def to_code(config: ConfigType) -> None:
    esp32.include_builtin_idf_component("esp_http_client")
    esp32.include_builtin_idf_component("esp-tls")
    # Keeps idle TLS sessions small, which matters with AWS IoT on the same heap.
    esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_DYNAMIC_BUFFER", True)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_endpoint(config[CONF_ENDPOINT]))
    cg.add(var.set_bucket(config[CONF_BUCKET]))
    cg.add(var.set_region(config[CONF_REGION]))
    cg.add(var.set_access_key_id(config[CONF_ACCESS_KEY_ID]))
    cg.add(var.set_secret_access_key(config[CONF_SECRET_ACCESS_KEY]))
    if root_ca := config.get(CONF_ROOT_CA):
        cg.add(var.set_root_ca(root_ca))
    elif config[CONF_ENDPOINT].startswith("https://"):
        esp32.require_certificate_bundle()
