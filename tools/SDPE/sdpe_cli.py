# -*- coding: utf-8 -*-
"""
SDPE (Software-Defined Power Electronics) CLI Tool
Component Header Compiler
"""

import os
import json
import argparse
import sys
from jinja2 import Environment, FileSystemLoader

def load_json(filepath):
    """Load and return a JSON file as a dictionary."""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Error: Could not find '{filepath}'")
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)

def format_parameters(raw_params, paradigm_param_defs):
    """
    Format parameters and attach metadata (description, unit) for Doxygen generation.
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
            # Handle user custom parameters not defined in schema
            macro_name = param_name.upper()
            formatted_params[macro_name] = {
                "value": f"(({param_value}))",
                "desc": "User custom defined parameter",
                "unit": ""
            }
            
    return formatted_params

def generate_component_header(input_file, template_json_file, output_file):
    """Generate the C header file containing the hardware preset macros."""
    print(f"[*] Loading Instance Definition: {input_file}")
    print(f"[*] Loading Paradigm Schema:   {template_json_file}")
    
    # 1. Load Data
    instance = load_json(input_file)
    paradigm = load_json(template_json_file)
    
    # 2. Setup Jinja2 Environment
    template_dir = os.path.dirname(template_json_file) or "."
    env = Environment(loader=FileSystemLoader(template_dir))
    
    # Register custom filter for aligning C macros
    env.filters['ljust'] = lambda s, width: str(s).ljust(width)
    
    # 3. Format Parameters
    formatted_params = format_parameters(
        instance.get("parameter_values", {}), 
        paradigm.get("parameters", [])
    )
    
    # 4. Build Context Dictionary
    obj_name = instance.get("instance_name", "obj")
    prefix = instance.get("prefix", "PREFIX")
    
    ctx = {
        "display_name": paradigm.get("display_name", "Unknown Component"),
        "obj_name": obj_name,
        "prefix": prefix,
        "formatted_params": formatted_params,
    }
    
    # 5. Render Macros via Jinja2
    j2_filename = paradigm.get("template_file", "default.j2")
    print(f"[*] Rendering Template: {j2_filename}")
    template = env.get_template(j2_filename)
    rendered_macros = template.module.render_config_macros(ctx).strip()
    
    # 6. Construct Header File Content
    guard_name = f"_FILE_SDPE_{prefix.upper()}_{obj_name.upper()}_H_"
    gen_command = " ".join(sys.argv)
    
    header_content = [
        "/**",
        " * @file",
        " * @brief SDPE Auto-Generated Component Header",
        f" * @note Generated from: {os.path.basename(input_file)}",
        f" * @note Using Paradigm: {os.path.basename(template_json_file)}",
        " * @note WARNING: DO NOT MODIFY THIS FILE MANUALLY. EDIT THE SOURCE JSON INSTEAD.",
        f" * @note Command: python {gen_command}",
        " */",
        "",
        f"#ifndef {guard_name}",
        f"#define {guard_name}",
        "",
        "#ifdef __cplusplus",
        'extern "C"',
        "{",
        "#endif // __cplusplus",
        "",
        rendered_macros,
        "",
        "#ifdef __cplusplus",
        "}",
        "#endif // __cplusplus",
        "",
        f"#endif // {guard_name}",
        ""
    ]
    
    # 7. Write to File
    output_dir = os.path.dirname(output_file)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        
    with open(output_file, "w", encoding="utf-8") as f:
        f.write("\n".join(header_content))
        
    print(f"[+] Success! Component Preset Header Generated at: {output_file}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SDPE Component Header Compiler")
    parser.add_argument("-i", "--input", required=True, help="Input instance JSON file")
    parser.add_argument("-t", "--template", required=True, help="Paradigm JSON file")
    parser.add_argument("-o", "--output", help="Output .h file path (default: input_filename.h)")
    
    args = parser.parse_args()
    
    # Handle default output filename logic
    if not args.output:
        input_base = os.path.splitext(os.path.basename(args.input))[0]
        output_path = f"{input_base}.h"
    else:
        output_path = args.output
        
    generate_component_header(args.input, args.template, output_path)