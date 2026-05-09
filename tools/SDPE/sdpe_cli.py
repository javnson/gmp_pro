# -*- coding: utf-8 -*-
"""
SDPE Production CLI
Generates formal C/H header and source files for a specific instance,
placing them in the Paradigm's specified output_path.
"""

import os
import sys
import argparse
from sdpe_compiler import SDPECompiler

def format_code_block(content):
    """Return content if valid, else empty string to avoid clutter."""
    return content.strip() if content and content.strip() else ""

def generate_production_files(paradigm_name, instance_name, mode_override=None):
    # Set paths relative to this script's location
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    paradigms_dir = os.path.join(base_dir, "paradigms")
    inst_dir = os.path.join(base_dir, "inst")
    
    compiler = SDPECompiler(paradigms_dir=paradigms_dir, inst_dir=inst_dir)

    paradigm = compiler.get_paradigm(paradigm_name)
    if not paradigm:
        return False, f"Paradigm '{paradigm_name}' not found."

    db_paths = paradigm.get("instance_databases", [])
    inst_data, _ = compiler.find_instance(instance_name, db_paths)
    
    if not inst_data:
        return False, f"Instance '{instance_name}' not found in databases."

    # Fake context for top-level requirements to allow successful compilation
    fake_context = {}
    for req in paradigm.get("required_context", []):
        ctx_name = req["name"]
        fake_context[ctx_name] = f"/* FROM_OUT: {ctx_name} */"

    # Render Component
    blocks, prod_path, _ = compiler.render_component(
        paradigm_name=paradigm_name,
        instance_name=instance_name,
        instance_data=inst_data,
        mode_override=mode_override,
        external_context=fake_context
    )

    if blocks["error"]:
        return False, f"Template Error: {blocks['error']}"

    if prod_path == "NOT_SPECIFIED":
        prod_path = "hardware_presets/uncategorized"

    # Create target directory
    output_dir = os.path.join(base_dir, prod_path)
    os.makedirs(output_dir, exist_ok=True)

    # Prefix generation for function wrappers
    prefix_lower = f"sdpe_{instance_name.lower()}"
    guard_name = f"_SDPE_{instance_name.upper()}_H_"

    # =========================================================
    # Generate Header File (.h)
    # =========================================================
    h_filepath = os.path.join(output_dir, f"{instance_name}.h")
    h_buffer = [
        "/**",
        f" * @file {instance_name}.h",
        f" * @brief SDPE Generated Hardware Preset for {instance_name}",
        f" * @paradigm {paradigm_name}",
        " */",
        f"#ifndef {guard_name}",
        f"#define {guard_name}\n",
        "#ifdef __cplusplus",
        'extern "C" {',
        "#endif\n",
        format_code_block(blocks["config"]),
        "\n/* --- External Declarations --- */",
        format_code_block(blocks["decl"]),
        "\n/* --- Preset Function Prototypes --- */",
        f"void {prefix_lower}_init(void);",
        f"void {prefix_lower}_step_in(void);",
        f"void {prefix_lower}_step_out(void);\n",
        "#ifdef __cplusplus",
        "}",
        "#endif",
        f"#endif // {guard_name}"
    ]

    with open(h_filepath, "w", encoding="utf-8") as f:
        f.write("\n".join(h_buffer))

    # =========================================================
    # Generate Source File (.c)
    # =========================================================
    c_filepath = os.path.join(output_dir, f"{instance_name}.c")
    c_buffer = [
        "/**",
        f" * @file {instance_name}.c",
        f" * @brief SDPE Generated Hardware Implementation for {instance_name}",
        " */",
        f'#include "{instance_name}.h"\n',
        f"void {prefix_lower}_init(void)\n{{",
        format_code_block(blocks["init"]),
        "}\n",
        f"void {prefix_lower}_step_in(void)\n{{",
        format_code_block(blocks["input"]),
        "}\n",
        f"void {prefix_lower}_step_out(void)\n{{",
        format_code_block(blocks["output"]),
        "}\n"
    ]

    with open(c_filepath, "w", encoding="utf-8") as f:
        f.write("\n".join(c_buffer))

    return True, f"Successfully generated {h_filepath} and {c_filepath}"

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SDPE Production CLI Generator")
    parser.add_argument("-p", "--paradigm", required=True, help="Paradigm name")
    parser.add_argument("-i", "--instance", required=True, help="Specific instance name")
    parser.add_argument("-m", "--mode", help="Specific mode (Optional)")
    
    args = parser.parse_args()
    success, msg = generate_production_files(args.paradigm, args.instance, args.mode)
    
    if not success:
        print(f"[FAIL] {args.instance}: {msg}", file=sys.stderr)
        sys.exit(1)
    else:
        print(f"[OK] {args.instance}: Generated successfully.")
        sys.exit(0)