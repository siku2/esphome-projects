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
import shutil
import subprocess
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
        uri: Path | str, *, context: Literal["local", "global"] = "local"
    ) -> str:
        assert context in ("local", "global"), "context must be 'local' or 'global'"

        if context == "local":
            asset_out_dir = out_dir
            url_prefix = ""
        else:
            asset_out_dir = global_out_dir
            url_prefix = "../"

        asset_out_dir.mkdir(parents=True, exist_ok=True)

        if isinstance(uri, str) and (
            uri.startswith("http://") or uri.startswith("https://")
        ):
            with httpx.stream("GET", uri) as resp:
                etag = resp.headers.get("ETag")
                url_path = Path(resp.url.path)
                stem = url_path.name or resp.url.host.replace(".", "_")
                suffix = url_path.suffix
                uid_builder = hashlib.sha256(usedforsecurity=False)
                uid_builder.update(str(resp.url).encode("utf-8"))
                uid_builder.update(etag.encode("utf-8") if etag else b"")
                uid = uid_builder.hexdigest()[:8]

                filename = f"{stem}-{uid}{suffix}"
                path = asset_out_dir / filename
                if not path.exists():
                    with path.open("wb+") as f:
                        for chunk in resp.iter_bytes():
                            f.write(chunk)

                return f"{url_prefix}{filename}"

        local_path = Path(uri)
        if local_path.is_absolute():
            local_path = local_path.relative_to(_PROJECT_ROOT)

        uid = hashlib.sha256(
            local_path.read_bytes(), usedforsecurity=False
        ).hexdigest()[:8]

        filename = f"{local_path.stem}-{uid}{local_path.suffix}"
        path = asset_out_dir / filename
        if not path.exists():
            shutil.copy(local_path, path)

        return f"{url_prefix}{filename}"

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
    subprocess.run(
        args,
        check=True,
        stdin=subprocess.DEVNULL,
        capture_output=True,
        timeout=60,
    )


if __name__ == "__main__":
    main()
