#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "httpx",
#     "jinja2",
# ]
# ///

import argparse
import dataclasses
import hashlib
import logging
import os
import shutil
import subprocess
from collections.abc import Iterable
from pathlib import Path
from typing import Any, Literal, Protocol, cast

import httpx
import jinja2

_LOGGER = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).parent
_PROJECTS_DIR = _PROJECT_ROOT / "projects"

_PAGES_TEMPLATES_DIR = _PROJECT_ROOT / "pages/templates"


class Args(Protocol):
    pass


def parse_args() -> Args:
    parser = argparse.ArgumentParser("x.py")
    _subparsers = parser.add_subparsers()

    args = parser.parse_args()
    return cast(Args, args)


def main() -> None:
    _args = parse_args()

    logging.basicConfig(level=logging.INFO)

    generate_pages(out_dir=_PROJECT_ROOT / "public")
    generate_interactive_bom(
        _PROJECTS_DIR / "warema-cover/hardware/warema-cover.kicad_pcb",
        _PROJECT_ROOT / "public",
    )


def generate_pages(*, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(_PAGES_TEMPLATES_DIR),
        autoescape=jinja2.select_autoescape(),
    )
    generate_project_page(
        env,
        _PROJECTS_DIR / "warema-cover",
        out_dir=out_dir / "warema-cover",
        global_out_dir=out_dir,
    )


@dataclasses.dataclass(kw_only=True)
class ProjectContext:
    name: str

    @dataclasses.dataclass(kw_only=True)
    class Kicad:
        files: list[Path]

    kicad: Kicad | None


def generate_project_page(
    env: jinja2.Environment, project_dir: Path, *, out_dir: Path, global_out_dir: Path
) -> None:
    project = ProjectContext(name=project_dir.name, kicad=None)
    if (project_dir / "hardware").is_dir():
        files = (
            list(project_dir.glob("hardware/*.kicad_pcb"))
            + list(project_dir.glob("hardware/*.kicad_pro"))
            + list(project_dir.glob("hardware/*.kicad_sch"))
        )
        if files:
            project.kicad = ProjectContext.Kicad(files=files)

    def get_asset_url(
        uri: Path | str,
        *,
        context: Literal["local", "global"] = "local",
        preserve_file_name: bool = False,
    ) -> str:
        assert context in ("local", "global"), "context must be 'local' or 'global'"

        if context == "local":
            asset_out_dir = out_dir
            url_prefix = ""
        else:
            asset_out_dir = global_out_dir
            url_prefix = "../"

        asset_out_dir.mkdir(parents=True, exist_ok=True)

        def get_asset_path(orig_filename: str, uid_parts: Iterable[str | bytes]) -> tuple[Path, str]:
            uid_builder = hashlib.sha256(usedforsecurity=False)
            for part in uid_parts:
                if isinstance(part, str):
                    part = part.encode("utf-8")
                uid_builder.update(part)
            uid = uid_builder.hexdigest()[:8]

            if preserve_file_name:
                local_asset_dir = asset_out_dir / uid
                local_asset_dir.mkdir(exist_ok=True)
                return (local_asset_dir / orig_filename, f"{url_prefix}{uid}/{orig_filename}")

            stem, _, suffix = orig_filename.partition(".")
            return (asset_out_dir / f"{stem}-{uid}.{suffix}", f"{url_prefix}{stem}-{uid}.{suffix}")

        if isinstance(uri, str) and (
            uri.startswith("http://") or uri.startswith("https://")
        ):
            with httpx.stream("GET", uri) as resp:
                etag = resp.headers.get("ETag")
                (asset_path, asset_url) = get_asset_path(Path(resp.url.path).name or resp.url.host.replace(".", "_"), [str(resp.url), etag])
                if not asset_path.exists():
                    _LOGGER.info("Downloading %s to %s", uri, asset_path)
                    with asset_path.open("wb+") as f:
                        for chunk in resp.iter_bytes():
                            f.write(chunk)

                return asset_url

        local_path = Path(uri)
        if local_path.is_absolute():
            local_path = local_path.relative_to(_PROJECT_ROOT)

        (asset_path, asset_url) = get_asset_path(local_path.name, [local_path.read_bytes()])

        if not asset_path.exists():
            _LOGGER.info("Copying %s to %s", local_path, asset_path)
            shutil.copy(local_path, asset_path)

        return asset_url

    render_ctx: dict[str, Any] = {"project": project, "get_asset_url": get_asset_url}

    out_dir.mkdir(exist_ok=True)
    template = env.get_template("project.html")
    template.stream(render_ctx).dump(  # type: ignore
        str(out_dir / "index.html"),
        encoding="utf-8",
    )


def generate_interactive_bom(
    pcb_file: Path,
    dest_dir: Path,
    *,
    dark_mode: bool = True,
) -> None:
    """
    See: <https://github.com/openscopeproject/InteractiveHtmlBom/wiki/Usage>
    """
    args: list[str] = [
        "xvfb-run",
        "--auto-servernum",
        "--server-args",
        "-screen 0 1024x768x24",
        "generate_interactive_bom",
        "--no-browser",
        "--dest-dir",
        str(dest_dir),
    ]
    if dark_mode:
        args.append("--dark-mode")
    args.append(str(pcb_file))
    _LOGGER.info("Generating interactive BOM for %s", pcb_file)
    env = {
        **os.environ,
        "INTERACTIVE_HTML_BOM_NO_DISPLAY": "true",
    }
    try:
        subprocess.run(
            args,
            check=True,
            stdin=subprocess.DEVNULL,
            capture_output=True,
            timeout=60,
            env=env,
        )
    except subprocess.CalledProcessError as exc:
        _LOGGER.error(
            "Failed to generate interactive BOM for %s: %s",
            pcb_file,
            exc.stdout.decode("utf-8", errors="replace"),
        )
        msg = "Failed to generate interactive BOM"
        raise RuntimeError(msg)


if __name__ == "__main__":
    main()
