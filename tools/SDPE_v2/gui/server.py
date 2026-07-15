#!/usr/bin/env python3
"""Local web GUI server for SDPE v2."""

from __future__ import annotations

import argparse
import json
import mimetypes
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sdpe_v2.generator import HeaderGenerator
from sdpe_v2.library import SDPELibrary
from sdpe_v2.model import SDPEError


STATIC_DIR = Path(__file__).resolve().parent / "static"


def read_json(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def write_json(path: Path, data) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")


def safe_name(name: str) -> str:
    allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-."
    value = "".join(ch for ch in name if ch in allowed).strip(".")
    if not value:
        raise SDPEError("empty or invalid name")
    return value


class SDPEGuiHandler(BaseHTTPRequestHandler):
    server_version = "SDPEV2GUI/0.1"

    def do_GET(self):
        try:
            parsed = urlparse(self.path)
            if parsed.path.startswith("/api/"):
                self.handle_api_get(parsed.path, parse_qs(parsed.query))
            else:
                self.serve_static(parsed.path)
        except Exception as exc:  # noqa: BLE001 - API boundary
            self.send_json({"ok": False, "error": str(exc)}, status=500)

    def do_POST(self):
        self.handle_write_request("POST")

    def do_PUT(self):
        self.handle_write_request("PUT")

    def handle_write_request(self, method: str):
        try:
            parsed = urlparse(self.path)
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length).decode("utf-8") if length else "{}"
            data = json.loads(raw or "{}")
            if not parsed.path.startswith("/api/"):
                self.send_json({"ok": False, "error": f"unsupported {method} {parsed.path}"}, status=404)
                return
            self.handle_api_write(parsed.path, data)
        except Exception as exc:  # noqa: BLE001 - API boundary
            self.send_json({"ok": False, "error": str(exc)}, status=500)

    @property
    def library_root(self) -> Path:
        return self.server.library_root  # type: ignore[attr-defined]

    def load_library(self) -> SDPELibrary:
        return SDPELibrary(self.library_root).load()

    def handle_api_get(self, path: str, query: dict[str, list[str]]) -> None:
        if path == "/api/state":
            self.send_json(self.api_state())
            return
        if path == "/api/validate":
            lib = self.load_library()
            warnings = lib.validate()
            self.send_json({"ok": True, "schemas": len(lib.schemas), "entities": len(lib.entity_files), "warnings": warnings})
            return
        if path == "/api/exports":
            entity_id = query.get("entity", [""])[0]
            self.send_json({"ok": True, "exports": self.entity_exports(entity_id)})
            return
        self.send_json({"ok": False, "error": f"unknown endpoint {path}"}, status=404)

    def handle_api_write(self, path: str, data) -> None:
        if path == "/api/schema":
            item_id = safe_name(data.get("id", ""))
            write_json(self.library_root / "schemas" / f"{item_id}.json", data)
            self.send_json({"ok": True, "path": f"schemas/{item_id}.json"})
            return
        if path == "/api/entity":
            item_id = safe_name(data.get("id", ""))
            write_json(self.library_root / "entities" / f"{item_id}.json", data)
            self.send_json({"ok": True, "path": f"entities/{item_id}.json"})
            return
        if path == "/api/project":
            item_id = safe_name(data.get("id", ""))
            write_json(self.library_root / "projects" / f"{item_id}.json", data)
            self.send_json({"ok": True, "path": f"projects/{item_id}.json"})
            return
        if path == "/api/generate":
            self.generate(data)
            return
        self.send_json({"ok": False, "error": f"unknown endpoint {path}"}, status=404)

    def api_state(self):
        lib = self.load_library()
        schemas = []
        for schema_id, schema in sorted(lib.schemas.items()):
            schemas.append(
                {
                    "id": schema_id,
                    "display_name": schema.display_name,
                    "description": schema.description,
                    "category": schema.category,
                    "data": read_json(lib.schemas_dir / f"{schema_id}.json"),
                }
            )

        entities = []
        for entity_id, path in sorted(lib.entity_files.items()):
            data = read_json(path)
            entities.append(
                {
                    "id": entity_id,
                    "schema": data.get("schema", ""),
                    "display_name": data.get("display_name", entity_id),
                    "data": data,
                }
            )

        projects = []
        projects_dir = self.library_root / "projects"
        if projects_dir.exists():
            for path in sorted(projects_dir.rglob("*.json")):
                data = read_json(path)
                projects.append(
                    {
                        "id": data.get("id", path.stem),
                        "suite": data.get("suite", ""),
                        "display_name": data.get("display_name", data.get("id", path.stem)),
                        "data": data,
                    }
                )

        return {
            "ok": True,
            "library_root": str(self.library_root),
            "schemas": schemas,
            "entities": entities,
            "projects": projects,
        }

    def entity_exports(self, entity_id: str):
        if not entity_id:
            return []
        lib = self.load_library()
        entity = lib.entity(entity_id)
        results = []

        def collect(prefix_path: str, node):
            schema = lib.schema(node.schema_id)
            for export_name in schema.exports:
                results.append(
                    {
                        "path": f"{prefix_path}.{export_name}",
                        "entity": node.id,
                        "schema": schema.id,
                        "export": export_name,
                    }
                )
            for slot, comp in node.components.items():
                collect(f"{prefix_path}.{slot}", comp.entity)

        collect(entity.id, entity)
        return results

    def generate(self, data) -> None:
        lib = self.load_library()
        out_dir = Path(data.get("out", ROOT / "build_gui"))
        include_prefix = data.get("include_prefix", "ctl/component")
        gen = HeaderGenerator(lib, out_dir, include_prefix)
        mode = data.get("mode")
        if mode == "entity":
            items = gen.generate_entity_tree(data["entity"])
        elif mode == "all":
            items = gen.generate_all_entities()
        elif mode == "project":
            project_id = data["project"]
            project_path = self.library_root / "projects" / f"{safe_name(project_id)}.json"
            items = gen.generate_project(project_path)
        else:
            raise SDPEError("generate mode must be entity, project, or all")
        self.send_json(
            {
                "ok": True,
                "files": [{"path": str(item.path), "changed": item.changed} for item in items],
            }
        )

    def serve_static(self, path: str) -> None:
        if path in ("", "/"):
            target = STATIC_DIR / "index.html"
        else:
            rel = path.lstrip("/")
            target = (STATIC_DIR / rel).resolve()
            if STATIC_DIR.resolve() not in target.parents and target != STATIC_DIR.resolve():
                self.send_error(403)
                return
        if not target.exists() or not target.is_file():
            self.send_error(404)
            return
        ctype = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
        data = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_json(self, data, status: int = 200) -> None:
        raw = json.dumps(data, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def log_message(self, fmt, *args):  # noqa: D401
        """Keep GUI server logs concise."""

        sys.stderr.write("[sdpe-gui] " + fmt % args + "\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run SDPE v2 local GUI server")
    parser.add_argument("--library", default=str(ROOT / "examples"), help="SDPE library root")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args(argv)

    server = ThreadingHTTPServer((args.host, args.port), SDPEGuiHandler)
    server.library_root = Path(args.library).resolve()  # type: ignore[attr-defined]
    print(f"SDPE v2 GUI: http://{args.host}:{args.port}")
    print(f"Library: {server.library_root}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping SDPE v2 GUI")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
