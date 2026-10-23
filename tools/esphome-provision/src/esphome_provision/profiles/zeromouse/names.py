"""Entity names from the translations of the ZeroMOUSE project."""

import yaml

from esphome_provision.common import ProvisionError
from esphome_provision.secrets import repo_root


def entity_names(key: str) -> set[str]:
    translations = repo_root() / "projects/zeromouse/translations"
    names: set[str] = set()
    for language in ("de", "en"):
        data = yaml.safe_load((translations / f"{language}.yaml").read_text())
        if not isinstance(data, dict) or not isinstance(data.get(key), str):
            raise ProvisionError(f"The {language} translation has no {key}")
        names.add(data[key])
    return names
