import os
import json
import argparse
from jinja2 import Environment, FileSystemLoader

def load_json(filepath):
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Error: Could not find '{filepath}'")
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)

def format_parameters(raw_params, paradigm_param_defs):
    formatted_params = {}
    def_lookup = {p["name"]: p for p in paradigm_param_defs}
    
    for param_name, param_value in raw_params.items():
        if param_name in def_lookup:
            param_def = def_lookup[param_name]
            macro_name = param_def["macro_name"]
            value_format = param_def.get("value_format", "{}")
            formatted_params[macro_name] = value_format.format(param_value)
        else:
            macro_name = param_name.upper()
            formatted_params[macro_name] = f"({param_value})"
            
    return formatted_params

def generate_component(input_file, template_json_file, output_file):
    print(f"[*] Loading Definition: {input_file}")
    print(f"[*] Loading Paradigm:   {template_json_file}")
    
    # 1. Load JSONs
    instance = load_json(input_file)
    paradigm = load_json(template_json_file)
    
    # 2. Setup Jinja2 Environment (Pointing to the folder of the paradigm JSON)
    template_dir = os.path.dirname(template_json_file) or "."
    env = Environment(loader=FileSystemLoader(template_dir))
    env.filters['ljust'] = lambda s, width: str(s).ljust(width)
    
    # 3. Format parameters
    formatted_params = format_parameters(instance.get("parameter_values", {}), paradigm.get("parameters", []))
    
    # 4. Extract Mode Config
    mode_key = instance.get("channel_mode", "single_channel")
    mode_config = paradigm.get("supported_modes", {}).get(mode_key, {})
    
    # 5. Build Context Dictionary (ctx)
    ctx = {
        "display_name": paradigm.get("display_name", "Unknown Component"),
        "obj_name": instance.get("instance_name", "obj"),
        "prefix": instance.get("prefix", "PREFIX"),
        "channel_mode": mode_key,
        "c_type": mode_config.get("c_type", ""),
        "channel_count": mode_config.get("channel_count", 1),
        "init_func": mode_config.get("init_func", ""),
        "step_func": mode_config.get("step_func", ""),
        "formatted_params": formatted_params,
    }
    
    # Inject test context (ADC vref, etc.)
    if "test_context" in instance:
        ctx.update(instance["test_context"])
        
    # 6. Load Template and Render
    j2_filename = paradigm.get("template_file", "default.j2")
    print(f"[*] Rendering Template: {j2_filename}")
    template = env.get_template(j2_filename)
    
    # Generate the four sections
    rendered_macros = template.module.render_config_macros(ctx).strip()
    rendered_decl = template.module.render_decl(ctx).strip()
    rendered_init = template.module.render_init(ctx).strip()
    rendered_input = template.module.render_input(ctx).strip()
    
    # 7. Assemble final output
    output_content = [
        "// ============================================================================",
        f"// SDPE Auto-Generated Component: {ctx['display_name']}",
        "// ============================================================================\n",
        rendered_macros,
        "\n/* --- Declarations --- */",
        rendered_decl,
        "\n/* --- Initialization --- */",
        rendered_init,
        "\n/* --- IO Step --- */",
        rendered_input,
        "\n"
    ]
    
    # 8. Write to file
    os.makedirs(os.path.dirname(output_file) if os.path.dirname(output_file) else ".", exist_ok=True)
    with open(output_file, "w", encoding="utf-8") as f:
        f.write("\n".join(output_content))
        
    print(f"[+] Success! Code generated at: {output_file}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SDPE Single Component Generator")
    parser.add_argument("-i", "--input", required=True, help="Input instance JSON file")
    parser.add_argument("-t", "--template", required=True, help="Paradigm JSON file")
    parser.add_argument("-o", "--output", help="Output .h or .c file path (default: input_filename.h)")
    
    args = parser.parse_args()
    
    # Handle default output filename
    if not args.output:
        base_name = os.path.splitext(os.path.basename(args.input))[0]
        args.output = f"{base_name}.h"
        
    generate_component(args.input, args.template, args.output)