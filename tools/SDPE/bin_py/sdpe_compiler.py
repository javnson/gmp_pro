# -*- coding: utf-8 -*-
"""
SDPE Core Compiler Engine
Supports single components and recursive composite boards with automatic dependency routing.
"""

import os
import json
from jinja2 import Environment, FileSystemLoader

class SDPECompiler:
    def __init__(self, paradigms_dir="paradigms", inst_dir="inst"):
        self.paradigms_dir = paradigms_dir
        self.inst_dir = inst_dir
        
        self.env = Environment(loader=FileSystemLoader(self.paradigms_dir))
        self.env.filters['ljust'] = lambda s, width: str(s).ljust(width)

    def load_json(self, filepath):
        """
        Safely load a JSON file with encoding fallback and strict error reporting.
        """
        if not os.path.exists(filepath):
            return None
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                return json.load(f)
        except UnicodeDecodeError:
            try:
                # Fallback to GBK for Windows environments
                with open(filepath, 'r', encoding='gbk') as f:
                    return json.load(f)
            except Exception:
                print(f"[X] Encoding Error: Could not read '{filepath}'. Please save it as UTF-8.")
                return None
        except json.JSONDecodeError as e:
            # Catch trailing commas or illegal comments (//)
            print(f"[X] JSON Syntax Error in '{filepath}': {e}")
            print(f"    (Note: Standard JSON does not support trailing commas or '//' comments)")
            return None
        except Exception as e:
            print(f"[X] Unexpected Error reading '{filepath}': {e}")
            return None

    def get_paradigm(self, paradigm_name):
        """
        Recursively search for paradigm definition by name across all subdirectories.
        """
        target_filename = f"{paradigm_name}.json"
        
        for root, dirs, files in os.walk(self.paradigms_dir):
            if target_filename in files:
                filepath = os.path.join(root, target_filename)
                data = self.load_json(filepath)
                if data: 
                    return data
                    
        return None

    def find_instance(self, instance_name, db_paths):
        """
        Search for a specific instance across multiple databases.
        """
        for db_path in db_paths:
            actual_path = db_path.replace("instances/", f"{self.inst_dir}/")
            db_content = self.load_json(actual_path)
            if db_content and "instances" in db_content:
                if instance_name in db_content["instances"]:
                    return db_content["instances"][instance_name], actual_path
        return None, None

    def get_all_instances(self, db_paths):
        """
        Fetch all instances from the provided database paths.
        """
        all_inst = {}
        for db_path in db_paths:
            actual_path = db_path.replace("instances/", f"{self.inst_dir}/")
            db_content = self.load_json(actual_path)
            if db_content and "instances" in db_content:
                all_inst.update(db_content["instances"])
        return all_inst

    def format_parameters(self, raw_params, paradigm_param_defs):
        """
        Format parameters based on the paradigm value_format rules.
        """
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
                formatted_params[param_name.upper()] = {
                    "value": f"(({param_value}))",
                    "desc": "User custom defined parameter",
                    "unit": ""
                }
        return formatted_params

    def render_component(self, paradigm_name, instance_name, instance_data, obj_name=None, prefix=None, mode_override=None, external_context=None, depth=0):
        """
        Core rendering function. Supports recursive unrolling for composite boards.
        """
        paradigm = self.get_paradigm(paradigm_name)
        if not paradigm:
            raise ValueError(f"Paradigm '{paradigm_name}' not found.")

        is_composite = paradigm.get("is_composite", False)

        supported_modes = paradigm.get("supported_modes", {})
        if mode_override and mode_override in supported_modes:
            mode_key = mode_override
        else:
            mode_key = list(supported_modes.keys())[0] if supported_modes else "default"
        mode_config = supported_modes.get(mode_key, {})

        formatted_params = self.format_parameters(
            instance_data.get("parameter_values", {}), 
            paradigm.get("parameters", [])
        )

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

        if external_context:
            ctx.update(external_context)

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
            blocks["error"] = f"Template rendering missing macro in {paradigm_name}: {e}"

        intended_path = paradigm.get("output_path", "NOT_SPECIFIED")

        # ==========================================================
        # Recursive Unrolling for Composites
        # ==========================================================
        if is_composite:
            expected_components = paradigm.get("expected_components", {})
            defined_components = instance_data.get("components", {})

            for role, exp_info in expected_components.items():
                if role not in defined_components:
                    continue # Skip optional components not defined in the instance
                
                sub_inst_info = defined_components[role]
                sub_instance_name = sub_inst_info.get("instance")
                sub_mode = sub_inst_info.get("mode")
                
                # Auto-Discovery for sub-components
                sub_paradigm_name = None
                sub_inst_data = None
                allowed_paradigms = exp_info.get("allowed_paradigms", [])
                
                for p_name in allowed_paradigms:
                    p_data = self.get_paradigm(p_name)
                    if not p_data: continue
                    db_paths = p_data.get("instance_databases", [])
                    found_data, _ = self.find_instance(sub_instance_name, db_paths)
                    if found_data:
                        sub_paradigm_name = p_name
                        sub_inst_data = found_data
                        break
                
                if not sub_inst_data:
                    err_msg = f" \n[!] Could not find instance '{sub_instance_name}' for role '{role}'."
                    blocks["error"] = (blocks["error"] or "") + err_msg
                    continue

                # Generate Sub-namespaces
                sub_obj = f"{ctx['obj_name']}_{role}"
                sub_prefix = f"{ctx['prefix']}_{role.upper()}"

                # Context Routing (Dependency Resolution & Bubbling)
                sub_context = {}
                default_map = exp_info.get("default_context_map", {})
                inst_map = sub_inst_info.get("context_map", {})
                override_map = sub_inst_info.get("context_override", {})

                sub_p_data = self.get_paradigm(sub_paradigm_name)
                for req in sub_p_data.get("required_context", []):
                    req_name = req["name"]
                    
                    if req_name in override_map:
                        sub_context[req_name] = override_map[req_name]
                    else:
                        mapped_name = inst_map.get(req_name, default_map.get(req_name, req_name))
                        if external_context and mapped_name in external_context:
                            sub_context[req_name] = external_context[mapped_name]
                        else:
                            sub_context[req_name] = f"/* FROM_OUT: {mapped_name} */"

                # Recursive Render
                sub_blocks, _, _ = self.render_component(
                    sub_paradigm_name, sub_instance_name, sub_inst_data,
                    obj_name=sub_obj, prefix=sub_prefix, mode_override=sub_mode, 
                    external_context=sub_context, depth=depth+1
                )

                # Aggregate
                separator = f"\n// {'-'*50}\n// Sub-component: {role} ({sub_paradigm_name})\n// {'-'*50}\n"
                
                for k in ["config", "decl", "init", "input", "output"]:
                    if sub_blocks[k]:
                        blocks[k] += separator + sub_blocks[k] + "\n"
                
                if sub_blocks["error"]:
                    blocks["error"] = (blocks["error"] or "") + sub_blocks["error"]

        return blocks, intended_path, paradigm