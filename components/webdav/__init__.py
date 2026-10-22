import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import esp32
from esphome.const import CONF_ID, CONF_PASSWORD, CONF_URL, CONF_USERNAME
from esphome.types import ConfigType

DEPENDENCIES = ["esp32", "network"]

CONF_ROOT_CA = "root_ca"

webdav_ns = cg.esphome_ns.namespace("webdav")
WebDav = webdav_ns.class_("WebDav", cg.Component)


_PEM_BEGIN = re.compile(r"-----BEGIN [A-Z0-9 ]+-----")
_PEM_END = re.compile(r"-----END [A-Z0-9 ]+-----")


def pem(value: object) -> str:
    value = cv.string_strict(value).strip()
    if not _PEM_BEGIN.search(value) or not _PEM_END.search(value):
        raise cv.Invalid("Expected a PEM block with BEGIN and END markers")
    return value + "\n"


def https_url(value: object) -> str:
    value = cv.url(value)
    if not value.startswith("https://"):
        raise cv.Invalid("Expected an https URL")
    return value.rstrip("/")


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WebDav),
            cv.Required(CONF_URL): https_url,
            cv.Required(CONF_USERNAME): cv.All(cv.string_strict, cv.Length(min=1)),
            cv.Required(CONF_PASSWORD): cv.sensitive(
                cv.All(cv.string_strict, cv.Length(min=1))
            ),
            cv.Optional(CONF_ROOT_CA): pem,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_with_framework("esp-idf"),
)


async def to_code(config: ConfigType) -> None:
    esp32.include_builtin_idf_component("esp_http_client")
    esp32.include_builtin_idf_component("esp-tls")
    # Keeps idle TLS sessions small, which matters with AWS IoT on the same heap.
    esp32.add_idf_sdkconfig_option("CONFIG_MBEDTLS_DYNAMIC_BUFFER", True)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_url(config[CONF_URL]))
    cg.add(var.set_username(config[CONF_USERNAME]))
    cg.add(var.set_password(config[CONF_PASSWORD]))
    if root_ca := config.get(CONF_ROOT_CA):
        cg.add(var.set_root_ca(root_ca))
    else:
        esp32.require_certificate_bundle()
