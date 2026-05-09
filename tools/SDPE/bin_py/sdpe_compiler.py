# -*- coding: utf-8 -*-
"""
SDPE Core Compiler Engine
Contains the central logic for loading paradigms, formatting parameters, 
and rendering Jinja2 templates into C code blocks.
"""

import os
import json
from jinja2 import Environment, FileSystemLoader

class SDPECompiler:
    def __init__(self, paradigms_dir="paradigms", inst_dir="inst"):
        self.paradigms_dir = paradigms_dir
        self.inst_dir = inst_dir
        
        # Setup Jinja2 Environment globally for the compiler instance
        self.env = Environment(loader=FileSystemLoader(self.paradigms_dir))
        self.env.filters['ljust'] = lambda s, width: str(s).ljust(width)

    def load_json(self, filepath):
        """Safely load a JSON file."""
        if not os.path.exists(filepath):
            return None
        with open(filepath, 'r', encoding='utf-8') as f:
            return json.load(f)

    def find_instance(self, instance_name, db_paths):
        """Search for a specific instance across multiple databases."""
        for db_path in db_paths:
            actual_path = db_path.replace("instances/", f"{self.inst_dir}/")
            db_content = self.load_json(actual_path)
            if db_content and "instances" in db_content:
                if instance_name in db_content["instances"]:
                    return db_content["instances"][instance_name], actual_path
        return None, None

    def get_all_instances(self, db_paths):
        """Fetch all instances from the provided database paths."""
        all_inst = {}
        for db_path in db_paths:
            actual_path = db_path.replace("instances/", f"{self.inst_dir}/")
            db_content = self.load_json(actual_path)
            if db_content and "instances" in db_content:
                all_inst.update(db_content["instances"])
        return all_inst

    def format_parameters(self, raw_params, paradigm_param_defs):
        """Format parameters based on the paradigm value_format rules."""
        formatted_params = {}
        def_lookup = {p["name"]: p for p in paradigm_param_defs}
        
        for param_name, param_value in raw_params.items():
            if param_name in def_lookup:
                param_def = def_lookup[param_name]
                macro_name = param_def["macro_name"]
                value_format = param_def.get("value_format", "(({}))")
                
                formatted_params[macro_name] = {
                    "value": value_format.format(param_value),
                    "desc": param_def.get("description", "Component parameter"),
                    "unit": param_def.get("unit", "")
                }
            else:
                # User custom overrides
                macro_name = param_name.upper()
                formatted_params[macro_name] = {
                    "value": f"(({param_value}))",
                    "desc": "User custom defined parameter",
                    "unit": ""
                }
        return formatted_params

    def render_component(self, paradigm_name, instance_name, instance_data, obj_name=None, prefix=None, mode_override=None, external_context=None):
        """
        Core rendering function. 
        Returns a dictionary containing the 5 code blocks and the intended output path.
        """
        # Load Paradigm
        paradigm_path = os.path.join(self.paradigms_dir, f"{paradigm_name}.json")
        paradigm = self.load_json(paradigm_path)
        if not paradigm:
            raise ValueError(f"Paradigm '{paradigm_name}' not found.")

        # Determine Mode
        supported_modes = paradigm.get("supported_modes", {})
        if mode_override and mode_override in supported_modes:
            mode_key = mode_override
        else:
            mode_key = list(supported_modes.keys())[0] if supported_modes else "default"
        mode_config = supported_modes.get(mode_key, {})

        # Format Parameters
        formatted_params = self.format_parameters(
            instance_data.get("parameter_values", {}), 
            paradigm.get("parameters", [])
        )

        # Build Context
        ctx = {
            "display_name": paradigm.get("display_name", "Unknown"),
            "obj_name": obj_name if obj_name else f"DBG_{instance_name.upper()}",
            "prefix": prefix if prefix else f"SDPE_{instance_name.upper()}",
            "channel_mode": mode_key,
            "c_type": mode_config.get("c_type", ""),
            "channel_count": mode_config.get("channel_count", 1),
            "init_func": mode_config.get("init_func", ""),
            "step_func": mode_config.get("step_func", ""),
            "data_injection": mode_config.get("data_injection", "step"),
            "formatted_params": formatted_params,
        }

        # Inject external context (e.g., globals from project, or /* FROM_OUT */ from debug)
        if external_context:
            ctx.update(external_context)

        # Render
        j2_filename = paradigm.get("template_file", f"{paradigm_name}.j2")
        template = self.env.get_template(j2_filename)

        blocks = {"config": "", "decl": "", "init": "", "input": "", "output": "", "error": None}
        
        try:
            blocks["config"] = template.module.render_config_macros(ctx).strip()
            blocks["decl"] = template.module.render_decl(ctx).strip()
            blocks["init"] = template.module.render_init(ctx).strip()
            blocks["input"] = template.module.render_input(ctx).strip()
            blocks["output"] = template.module.render_output(ctx).strip()
        except AttributeError as e:
            blocks["error"] = f"Template rendering missing macro: {e}"

        intended_path = paradigm.get("output_path", "NOT_SPECIFIED")
        
        return blocks, intended_path, paradigm