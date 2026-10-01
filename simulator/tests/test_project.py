"""Tests for project discovery and YAML parsing."""

from pathlib import Path

import pytest

from esphome_sim.errors import SimError
from esphome_sim.project import find_repo_root, list_projects, load_project

ENTRY = """\
packages:
  core: !include
    file: ../projects/demo/host.yaml
    vars:
      name: "demo"
      api_encryption_key: "c2VjcmV0"
      password: !secret ota_password
      path: !env_var HOME
"""


def make_repo(tmp_path: Path, files: dict[str, str]) -> Path:
    (tmp_path / "flake.nix").write_text("{}")
    (tmp_path / "tests").mkdir()
    for name, text in files.items():
        (tmp_path / "tests" / name).write_text(text)
    return tmp_path


def test_list_projects_only_host_files(tmp_path: Path) -> None:
    root = make_repo(
        tmp_path, {"b-host.yaml": ENTRY, "a-host.yaml": ENTRY, "a.yaml": ENTRY}
    )
    assert list_projects(root) == ["a", "b"]


def test_load_project_reads_key_through_tags(tmp_path: Path) -> None:
    root = make_repo(tmp_path, {"demo-host.yaml": ENTRY})
    project = load_project(root, "demo")
    assert project.name == "demo"
    assert project.api_key == "c2VjcmV0"
    assert project.path == root / "tests" / "demo-host.yaml"
    assert (project.host, project.port) == ("127.0.0.1", 6053)


def test_scalar_include_and_lambda_tags(tmp_path: Path) -> None:
    text = (
        "packages:\n"
        "  base: !include ../packages/base.yaml\n"
        "  core:\n"
        "    vars:\n"
        "      api_encryption_key: k\n"
        "      code: !lambda return 1;\n"
        "      gone: !remove\n"
    )
    root = make_repo(tmp_path, {"demo-host.yaml": text})
    assert load_project(root, "demo").api_key == "k"


def test_unknown_project_lists_known(tmp_path: Path) -> None:
    root = make_repo(tmp_path, {"demo-host.yaml": ENTRY})
    with pytest.raises(SimError, match="known projects: demo"):
        load_project(root, "other")


def test_missing_key(tmp_path: Path) -> None:
    root = make_repo(tmp_path, {"demo-host.yaml": "packages:\n  core:\n    vars: {}\n"})
    with pytest.raises(SimError, match="demo-host.yaml: no packages"):
        load_project(root, "demo")


def test_key_must_be_string(tmp_path: Path) -> None:
    text = "packages:\n  core:\n    vars:\n      api_encryption_key: 5\n"
    root = make_repo(tmp_path, {"demo-host.yaml": text})
    with pytest.raises(SimError, match=r"packages.core.vars.api_encryption_key"):
        load_project(root, "demo")


def test_packages_must_be_mapping(tmp_path: Path) -> None:
    root = make_repo(tmp_path, {"demo-host.yaml": "packages: [1, 2]\n"})
    with pytest.raises(SimError, match="packages must be a mapping"):
        load_project(root, "demo")


def test_invalid_yaml(tmp_path: Path) -> None:
    root = make_repo(tmp_path, {"demo-host.yaml": "packages: [\n"})
    with pytest.raises(SimError, match="cannot read"):
        load_project(root, "demo")


def test_find_repo_root_walks_up(tmp_path: Path) -> None:
    root = make_repo(tmp_path, {})
    nested = root / "a" / "b"
    nested.mkdir(parents=True)
    assert find_repo_root(nested) == root
