# -*- coding: utf-8 -*-
"""
SDPE Project Build Tool (Project Engine)
Fuses static library headers with topology requirements and user-defined hardware code.
"""

import os
import json
import sys
import re
from jinja2 import Environment, FileSystemLoader
from sdpe_compiler import SDPECompiler

class SDPEProjectBuilder:
    def __init__(self, project_path):
        # 1. Environment and Path Initialization
        self.lib_root = os.getenv("GMP_PRO_LOCATION")
        if not self.lib_root:
            print("[!] FATAL: Environment variable 'GMP_PRO_LOCATION' is not set.")
            sys.exit(1)
        
        self.script_base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.paradigms_dir = os.path.join(self.script_base, "paradigms")
        self.inst_dir = os.path.join(self.script_base, "inst")
        self.templates_dir = os.path.join(self.script_base, "templates")
        
        # 2. Initialize Core Compiler
        self.compiler = SDPECompiler(
            paradigms_dir=self.paradigms_dir, 
            inst_dir=self.inst_dir,
            templates_dir=self.templates_dir
        )
        
        # 3. Initialize System Template Environment (bin_py/templates)
        sys_template_path = os.path.join(self.script_base, "bin_py", "templates")
        self.sys_env = Environment(
            loader=FileSystemLoader(sys_template_path),
            trim_blocks=True,
            lstrip_blocks=True
        )
        self.sys_env.filters['ljust'] = lambda s, width: str(s).ljust(width)

        # 4. Resolve Project File Path
        # Priority: Absolute -> PWD -> GMP_PRO_LOCATION -> Script Relative
        abs_project_path = self._resolve_path(project_path)
        if not abs_project_path:
            print(f"[X] FATAL: Project file '{project_path}' not found.")
            sys.exit(1)

        print(f"[*] Loading Project Configuration: {abs_project_path}")
        try:
            with open(abs_project_path, 'r', encoding='utf-8') as f:
                self.proj_data = json.load(f)
        except Exception as e:
            print(f"[X] Failed to load project JSON: {e}")
            sys.exit(1)
            
        # 5. Load Topology Definition
        topo_ref = self.proj_data["topology_ref"]
        # If it's a name like "pmsm_foc", append .json
        topo_filename = topo_ref if topo_ref.endswith(".json") else f"{topo_ref}.json"
        
        # Search topology file with the same priority
        topo_path = self._resolve_path(topo_filename, sub_dir="proj_topology")
        if not topo_path:
            print(f"[X] FATAL: Topology file '{topo_filename}' not found.")
            sys.exit(1)

        print(f"[*] Loading Topology: {topo_path}")
        with open(topo_path, 'r', encoding='utf-8') as f:
            self.topo_data = json.load(f)

    def _resolve_path(self, target_path, sub_dir=""):
        """
        Helper to resolve paths based on:
        1. Absolute Path
        2. PWD (Current Working Directory)
        3. GMP_PRO_LOCATION
        4. SDPE Tool Root (with optional sub_dir)
        """
        if os.path.isabs(target_path) and os.path.exists(target_path):
            return target_path

        candidates = [
            os.path.abspath(target_path), # CWD / PWD
            os.path.join(self.lib_root, target_path), # GMP_PRO_LOCATION
            os.path.join(self.script_base, sub_dir, target_path) # SDPE Internal
        ]

        for c in candidates:
            if os.path.exists(c):
                return c
        return None

    def extract_user_code(self, filepath):
        """Extracts content between USER CODE tags."""
        user_code = {}
        if os.path.exists(filepath):
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
                pattern = r"/\*\s*USER CODE BEGIN\s+(.+?)\s*\*/(.*?)/\*\s*USER CODE END\s+\1\s*\*/"
                matches = re.finditer(pattern, content, re.DOTALL)
                for match in matches:
                    tag = match.group(1).strip()
                    code = match.group(2)
                    user_code[tag] = code.strip('\r\n')
        return user_code

    def build(self, output_dir=None):
        # Default output to PWD if not specified
        final_output_dir = os.path.abspath(output_dir) if output_dir else os.getcwd()
        os.makedirs(final_output_dir, exist_ok=True)

        build_ctx = {
            "overrides": self.proj_data.get("overrides", {}),
            "prefix_map": {},
            "preset_includes": [],
            "api_mappings": {},
            "input_blocks": [],
            "output_blocks": [],
            "user_code": {},
            "project_name": self.proj_data.get("project_name", "SDPE_Project")
        }

        role_map = self.proj_data.get("role_mapping", {})
        hw_pool = self.proj_data.get("hardware_pool", {})
        custom_macros = self.proj_data.get("custom_macros", {})
        global_env = self.proj_data.get("global_environment", {})

        # --- Phase 1: Hardware Instantiation & Action Block Rendering ---
        for role_name, hw_ref in role_map.items():
            parts = hw_ref.split('.')
            root_hw_name = parts[0]
            if root_hw_name not in hw_pool: continue
                
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
                obj_name=f"g_{root_hw_name}_obj",
                prefix=prefix,
                external_context=comp_context
            )
            
            if blocks["input"]:  build_ctx["input_blocks"].append(blocks["input"])
            if blocks["output"]: build_ctx["output_blocks"].append(blocks["output"])
            
            inc_path = f"ctl/{paradigm['output_path']}/{hw_config['instance']}.h"
            build_ctx["preset_includes"].append(inc_path)

        # --- Phase 2: Topology API Mapping ---
        for api_macro, source_path in self.topo_data.get("exported_api", {}).items():
            match = re.match(r"\{(.+?)\.(.+?)\}", source_path)
            if match:
                role, macro = match.groups()
                if role in role_map:
                    mapped_hw = role_map[role].split('.')[0]
                    build_ctx["api_mappings"][api_macro] = f"SDPE_{mapped_hw.upper()}_{macro.upper()}"

        # --- Phase 3: System File Rendering ---
        self._render_sys_file("ctrl_settings.h.j2", os.path.join(final_output_dir, "ctrl_settings.h"), build_ctx)
        self._render_sys_file("ctl_interface.h.j2", os.path.join(final_output_dir, "xplt.ctl_interface.h"), build_ctx)
        
        print(f"[*] Build Successful for: {build_ctx['project_name']}")
        print(f"[*] Artifacts located in: {final_output_dir}")

    def _render_sys_file(self, template_name, out_path, build_ctx):
        build_ctx["user_code"] = self.extract_user_code(out_path)
        try:
            template = self.sys_env.get_template(template_name)
            with open(out_path, "w", encoding="utf-8") as f:
                f.write(template.render(**build_ctx))
        except Exception as e:
            print(f"[X] Render Error in {template_name}: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python sdpe_gen_xplt_src.py <project_json> [output_dir]")
        sys.exit(1)
    
    specified_out_dir = sys.argv[2] if len(sys.argv) > 2 else None
    builder = SDPEProjectBuilder(sys.argv[1])
    builder.build(specified_out_dir)