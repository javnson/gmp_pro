# -*- coding: utf-8 -*-
"""
SDPE Project Build Tool - Bugfix version (No empty line inflation)
"""

import os
import json
import sys
import re
from jinja2 import Environment, FileSystemLoader
from sdpe_compiler import SDPECompiler

class SDPEProjectBuilder:
    def __init__(self, project_path):
        self.lib_root = os.getenv("GMP_PRO_LOCATION")
        if not self.lib_root:
            print("[!] FATAL: Environment variable 'GMP_PRO_LOCATION' is not set.")
            sys.exit(1)
        
        self.script_base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.paradigms_dir = os.path.join(self.script_base, "paradigms")
        self.inst_dir = os.path.join(self.script_base, "inst")
        
        self.compiler = SDPECompiler(paradigms_dir=self.paradigms_dir, inst_dir=self.inst_dir)
        template_path = os.path.join(self.script_base, "bin_py", "templates")
        self.sys_env = Environment(loader=FileSystemLoader(template_path))
        self.sys_env.filters['ljust'] = lambda s, width: str(s).ljust(width)

        with open(project_path, 'r', encoding='utf-8') as f:
            self.proj_data = json.load(f)
            
        topo_name = self.proj_data["topology_ref"]
        topo_path = os.path.join(self.script_base, "proj_topology", f"{topo_name}.json")
        with open(topo_path, 'r', encoding='utf-8') as f:
            self.topo_data = json.load(f)

    def extract_user_code(self, filepath):
        user_code = {}
        if os.path.exists(filepath):
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
                pattern = r"/\*\s*USER CODE BEGIN\s+(.+?)\s*\*/(.*?)/\*\s*USER CODE END\s+\1\s*\*/"
                matches = re.finditer(pattern, content, re.DOTALL)
                for match in matches:
                    tag = match.group(1).strip()
                    code = match.group(2)
                    # Keep the content but remove leading/trailing structural newlines
                    user_code[tag] = code.strip('\r\n')
        return user_code

    def build(self, output_dir="gen"):
        os.makedirs(output_dir, exist_ok=True)
        build_ctx = {
            "overrides": self.proj_data.get("overrides", {}),
            "prefix_map": {},
            "preset_includes": [],
            "api_mappings": {},
            "input_blocks": [],
            "output_blocks": [],
            "user_code": {}
        }
        role_map = self.proj_data.get("role_mapping", {})
        hw_pool = self.proj_data.get("hardware_pool", {})
        custom_macros = self.proj_data.get("custom_macros", {})
        global_env = self.proj_data.get("global_environment", {})

        for role_name, hw_ref in role_map.items():
            parts = hw_ref.split('.')
            root_hw_name = parts[0]
            hw_config = hw_pool[root_hw_name]
            prefix = f"SDPE_{root_hw_name.upper()}"
            build_ctx["prefix_map"][root_hw_name] = prefix
            comp_context = global_env.copy()
            for key, val in custom_macros.items():
                if key.startswith(root_hw_name):
                    clean_key = key[len(root_hw_name)+1:] if '.' in key else key
                    comp_context[clean_key] = val
            blocks, _, paradigm = self.compiler.render_component(
                paradigm_name=hw_config["paradigm"],
                instance_name=hw_config["instance"],
                instance_data={"parameter_values": self.proj_data.get("overrides", {}).get(root_hw_name, {})},
                obj_name=f"SDPE_{root_hw_name.upper()}_OBJ",
                prefix=prefix,
                external_context=comp_context
            )
            if blocks["input"]: build_ctx["input_blocks"].append(blocks["input"])
            if blocks["output"]: build_ctx["output_blocks"].append(blocks["output"])
            inc_path = f"ctl/{paradigm['output_path']}/{hw_config['instance']}.h"
            build_ctx["preset_includes"].append(inc_path)

        for api_macro, source_path in self.topo_data.get("exported_api", {}).items():
            match = re.match(r"\{(.+?)\.(.+?)\}", source_path)
            if match:
                role, macro = match.groups()
                if role in role_map:
                    mapped_hw = role_map[role].split('.')[0]
                    build_ctx["api_mappings"][api_macro] = f"SDPE_{mapped_hw.upper()}_{macro.upper()}"

        self._render_sys_file("ctrl_settings.h.j2", os.path.join(output_dir, "ctrl_settings.h"), build_ctx)
        self._render_sys_file("ctl_interface.j2", os.path.join(output_dir, "xplt.ctl_interface.h"), build_ctx)
        print(f"[*] Build Complete for: {self.proj_data['project_name']}")

    def _render_sys_file(self, template_name, out_path, build_ctx):
        build_ctx["user_code"] = self.extract_user_code(out_path)
        template = self.sys_env.get_template(template_name)
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(template.render(**build_ctx))

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python sdpe_build.py <path_to_project.json>")
        sys.exit(1)
    SDPEProjectBuilder(sys.argv[1]).build()