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
        
        # Base paths relative to the script location
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
        abs_project_path = project_path
        if not os.path.isabs(project_path):
            candidates = [
                os.path.join(self.lib_root, project_path),
                os.path.abspath(project_path),
                os.path.join(self.script_base, project_path)
            ]
            for c in candidates:
                if os.path.exists(c):
                    abs_project_path = c
                    break

        print(f"[*] Loading Project Configuration: {abs_project_path}")
        try:
            with open(abs_project_path, 'r', encoding='utf-8') as f:
                self.proj_data = json.load(f)
        except Exception as e:
            print(f"[X] Failed to load project JSON: {e}")
            sys.exit(1)
            
        # 5. Load Topology Definition
        topo_name = self.proj_data["topology_ref"]
        topo_path = os.path.join(self.script_base, "proj_topology", f"{topo_name}.json")
        print(f"[*] Loading Topology: {topo_path}")
        with open(topo_path, 'r', encoding='utf-8') as f:
            self.topo_data = json.load(f)

    def extract_user_code(self, filepath):
        """
        Extracts content between USER CODE BEGIN/END tags 
        to preserve manual implementation during re-generation.
        """
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
        # Determine absolute output directory. Default to PWD if not specified.
        if output_dir:
            if os.path.isabs(output_dir):
                final_output_dir = output_dir
            else:
                # If relative, anchor to lib_root or CWD based on preference.
                # Here we use CWD as standard PWD behavior.
                final_output_dir = os.path.abspath(output_dir)
        else:
            final_output_dir = os.getcwd()
            
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
            
            if root_hw_name not in hw_pool:
                continue
                
            hw_config = hw_pool[root_hw_name]
            prefix = f"SDPE_{root_hw_name.upper()}"
            build_ctx["prefix_map"][root_hw_name] = prefix
            
            comp_context = global_env.copy()
            for key, val in custom_macros.items():
                if key.startswith(root_hw_name):
                    clean_key = key[len(root_hw_name)+1:] if '.' in key else key
                    comp_context[clean_key] = val
            
            # Render Action Blocks
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
        """Internal helper to render system files with User Code protection."""
        build_ctx["user_code"] = self.extract_user_code(out_path)
        
        try:
            template = self.sys_env.get_template(template_name)
            content = template.render(**build_ctx)
            
            with open(out_path, "w", encoding="utf-8") as f:
                f.write(content)
        except Exception as e:
            print(f"[X] Render Error in {template_name}: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python sdpe_gen_xplt_src.py <path_to_project.json> [output_dir]")
        sys.exit(1)
    
    # Get output directory from sys.argv[2], otherwise None (will trigger PWD)
    specified_out_dir = sys.argv[2] if len(sys.argv) > 2 else None
    
    builder = SDPEProjectBuilder(sys.argv[1])
    builder.build(specified_out_dir)