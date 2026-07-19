"""Generate C MEX S-Function sources and a MATLAB-consumable registry."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

from jinja2 import Environment, FileSystemLoader, StrictUndefined

from .model import ComponentDefinition


class ComponentGenerator:
    def __init__(self, tool_root: Path):
        self.tool_root = tool_root.resolve()
        self.environment = Environment(
            loader=FileSystemLoader(str(self.tool_root / "templates")),
            undefined=StrictUndefined,
            autoescape=False,
            keep_trailing_newline=True,
        )

    def generate(self, config_paths: Iterable[Path], output_root: Path) -> list[Path]:
        output_root = output_root.resolve()
        generated_root = output_root / "generated"
        generated_root.mkdir(parents=True, exist_ok=True)
        components = [ComponentDefinition.load(Path(path)) for path in config_paths]
        if not components:
            raise ValueError("no component definitions selected")

        outputs: list[Path] = []
        registry: list[dict] = []
        for component in components:
            component_dir = generated_root / component.symbol
            component_dir.mkdir(parents=True, exist_ok=True)
            target = component_dir / f"{component.sfunction_name}.cpp"
            target.write_text(self.preview(component), encoding="utf-8", newline="\n")
            outputs.append(target)
            registry.append(
                {
                    **component.data,
                    "config_file": component.path.as_posix(),
                    "symbol": component.symbol,
                    "sfunction_name": component.sfunction_name,
                    "generated_source": target.as_posix(),
                }
            )

        registry_path = output_root / "registry.json"
        registry_path.write_text(
            json.dumps({"schema_version": 1, "components": registry}, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
            newline="\n",
        )
        outputs.append(registry_path)
        return outputs

    def preview(self, component: ComponentDefinition) -> str:
        template = self.environment.get_template(f"{component.data['template']}.cpp.j2")
        return template.render(
            component=component.data,
            symbol=component.symbol,
            sfunction=component.sfunction_name,
            external_parameters=[item for item in component.data["parameters"] if item.get("externalizable", False)],
        )
