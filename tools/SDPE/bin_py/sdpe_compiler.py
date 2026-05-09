# -*- coding: utf-8 -*-
"""
SDPE Core Compiler Engine
Integrated with ChoiceLoader for template/paradigm dual-path discovery.
"""

import os
import json
from jinja2 import Environment, FileSystemLoader, ChoiceLoader

class SDPECompiler:
    def __init__(self, paradigms_dir="paradigms", inst_dir="inst", templates_dir="templates"):
        self.paradigms_dir = paradigms_dir
        self.inst_dir = inst_dir
        self.templates_dir = templates_dir
        
        # Use ChoiceLoader to search both templates/ and paradigms/ folders
        loader = ChoiceLoader([
            FileSystemLoader(self.templates_dir),
            FileSystemLoader(self.paradigms_dir)
        ])
        
        self.env = Environment(loader=loader)
        # Standard filter for code alignment
        self.env.filters['ljust'] = lambda s, width: str(s).ljust(width)

    def load_json(self, filepath):
        """Safely load a JSON file with encoding fallback."""
        if not os.path.exists(filepath):
            return None
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            try:
                with open(filepath, 'r', encoding='gbk') as f:
                    return json.load(f)
            except Exception:
                return None

    def get_paradigm(self, paradigm_name):
        """Recursively search for paradigm definition."""
        target_filename = f"{paradigm_name}.json"
        for root, _, files in os.walk(self.paradigms_dir):
            if target_filename in files:
                return self.load_json(os.path.join(root, target_filename))
        return None

    def find_instance(self, instance_name, db_files):
        """Searches for an instance within the 'instances' key of DB files."""
        for db_file in db_files:
            full_path = os.path.join(self.inst_dir, db_file)
            db_data = self.load_json(full_path)
            if not db_data:
                continue
            # Logic: Check "instances" field first, then fallback to root
            instances_pool = db_data.get("instances", db_data)
            if isinstance(instances_pool, dict) and instance_name in instances_pool:
                return instances_pool[instance_name], db_file
        return None, None

    def get_all_instances(self, db_paths):
        """Fetch all instances from the provided database list."""
        all_inst = {}
        for db_file in db_paths:
            full_path = os.path.join(self.inst_dir, db_file)
            db_content = self.load_json(full_path)
            if db_content:
                pool = db_content.get("instances", db_content)
                if isinstance(pool, dict):
                    all_inst.update(pool)
        return all_inst

    def format_parameters(self, raw_params, paradigm_param_defs):
        """Format physical parameters into C-macros based on paradigm rules."""
        formatted_params = {}
        def_lookup = {p["name"]: p for p in paradigm_param_defs}
        
        for param_name, param_value in raw_params.items():
            if param_name in def_lookup:
                p_def = def_lookup[param_name]
                m_name = p_def["macro_name"]
                v_fmt = p_def.get("value_format", "(({}))")
                formatted_params[m_name] = {
                    "value": v_fmt.format(param_value),
                    "desc": p_def.get("description", ""),
                    "unit": p_def.get("unit", "")
                }
            else:
                formatted_params[param_name.upper()] = {
                    "value": f"(({param_value}))",
                    "desc": "User defined",
                    "unit": ""
                }
        return formatted_params

    def render_component(self, paradigm_name, instance_name, instance_data, obj_name=None, prefix=None, mode_override=None, external_context=None):
        """Main rendering engine for single or composite components."""
        paradigm = self.get_paradigm(paradigm_name)
        if not paradigm:
            raise ValueError(f"Paradigm '{paradigm_name}' not found.")

        # Determine mode
        modes = paradigm.get("supported_modes", {})
        mode_key = mode_override if mode_override in modes else (list(modes.keys())[0] if modes else "default")
        m_cfg = modes.get(mode_key, {})

        # Extract values
        raw_vals = instance_data.get("parameter_values", {})
        formatted_params = self.format_parameters(raw_vals, paradigm.get("parameters", []))

        # Base context
        ctx = {
            "display_name": paradigm.get("display_name", "Unknown"),
            "obj_name": obj_name or f"DBG_{instance_name.upper()}",
            "prefix": prefix or f"SDPE_{instance_name.upper()}",
            "channel_mode": mode_key,
            "c_type": m_cfg.get("c_type", ""),
            "channel_count": m_cfg.get("channel_count", 1),
            "init_func": m_cfg.get("init_func", ""),
            "step_func": m_cfg.get("step_func", ""),
            "data_injection": m_cfg.get("data_injection", "step"),
            "formatted_params": formatted_params,
        }
        
        # Inject raw values for direct access like {{ bias_v }}
        ctx.update(raw_vals)
        if external_context:
            ctx.update(external_context)

        # Template lookup
        j2_file = paradigm.get("template_file", f"{paradigm_name}.j2")
        template = self.env.get_template(j2_file)

        blocks = {"config": "", "decl": "", "init": "", "input": "", "output": "", "error": None}
        
        try:
            m = template.module
            blocks["config"] = m.render_config_macros(ctx).strip() if hasattr(m, 'render_config_macros') else ""
            blocks["decl"]   = m.render_decl(ctx).strip() if hasattr(m, 'render_decl') else ""
            blocks["init"]   = m.render_init(ctx).strip() if hasattr(m, 'render_init') else ""
            blocks["input"]  = m.render_input(ctx).strip() if hasattr(m, 'render_input') else ""
            blocks["output"] = m.render_output(ctx).strip() if hasattr(m, 'render_output') else ""
        except Exception as e:
            blocks["error"] = str(e)

        return blocks, paradigm.get("output_path", "GEN"), paradigm