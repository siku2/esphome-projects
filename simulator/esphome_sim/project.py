"""Discover simulator projects from the `tests/<name>-host.yaml` entry files."""

from collections.abc import Hashable
from dataclasses import dataclass
from pathlib import Path

import yaml

from esphome_sim.errors import SimError

HOST = "127.0.0.1"
PORT = 6053
SUFFIX = "-host.yaml"
ESPHOME_TAGS = ("!include", "!secret", "!lambda", "!extend", "!remove", "!env_var")


@dataclass(slots=True, frozen=True)
class Project:
    """A host build that the simulator can run and connect to."""

    name: str
    path: Path
    api_key: str
    host: str = HOST
    port: int = PORT


class EsphomeLoader(yaml.SafeLoader):
    """A safe loader that turns ESPHome tags into plain YAML values."""


def _construct_tagged(loader: yaml.SafeLoader, node: yaml.Node) -> object:
    if isinstance(node, yaml.MappingNode):
        return loader.construct_mapping(node)
    if isinstance(node, yaml.SequenceNode):
        return loader.construct_sequence(node)
    if isinstance(node, yaml.ScalarNode):
        return loader.construct_scalar(node)
    raise yaml.constructor.ConstructorError(
        None, None, f"unexpected node for tag {node.tag}", node.start_mark
    )


for _tag in ESPHOME_TAGS:
    EsphomeLoader.add_constructor(_tag, _construct_tagged)


def find_repo_root(start: Path) -> Path:
    """Return the closest directory at or above `start` that contains flake.nix."""
    for directory in (start, *start.parents):
        if (directory / "flake.nix").is_file():
            return directory
    raise SimError(f"no flake.nix found in {start} or its parents, pass --root")


def list_projects(repo_root: Path) -> list[str]:
    """Return the names of all simulator projects, sorted."""
    tests = repo_root / "tests"
    return sorted(p.name.removesuffix(SUFFIX) for p in tests.glob(f"*{SUFFIX}"))


def project_path(repo_root: Path, name: str) -> Path:
    """Return the entry file of the project `name`."""
    return repo_root / "tests" / f"{name}{SUFFIX}"


def load_project(repo_root: Path, name: str) -> Project:
    """Load the project `name` and read its API key."""
    path = project_path(repo_root, name)
    if not path.is_file():
        known = ", ".join(list_projects(repo_root)) or "none"
        raise SimError(f"unknown project {name!r}, known projects: {known}")
    return Project(name=name, path=path, api_key=read_api_key(path))


def read_api_key(path: Path) -> str:
    """Read `packages.*.vars.api_encryption_key` from an entry file."""
    try:
        with path.open(encoding="utf-8") as file:
            document: object = yaml.load(file, Loader=EsphomeLoader)
    except (OSError, yaml.YAMLError) as err:
        raise SimError(f"{path}: cannot read: {err}") from err

    root = _mapping(document, path, "")
    packages = _mapping(root.get("packages"), path, "packages")
    keys: set[str] = set()
    for package_name, package in packages.items():
        where = f"packages.{package_name}"
        if not isinstance(package, dict):
            continue
        vars_ = package.get("vars")
        if vars_ is None:
            continue
        key = _mapping(vars_, path, f"{where}.vars").get("api_encryption_key")
        if key is None:
            continue
        if not isinstance(key, str) or not key:
            raise SimError(
                f"{path}: {where}.vars.api_encryption_key must be a non-empty string"
            )
        keys.add(key)
    if not keys:
        raise SimError(f"{path}: no packages.*.vars.api_encryption_key found")
    if len(keys) > 1:
        raise SimError(f"{path}: packages define different api_encryption_key values")
    return keys.pop()


def _mapping(value: object, path: Path, key: str) -> dict[Hashable, object]:
    if not isinstance(value, dict):
        where = key or "the document root"
        raise SimError(f"{path}: {where} must be a mapping")
    return value
