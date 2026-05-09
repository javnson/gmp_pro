# -*- coding: utf-8 -*-
"""
SDPE Debug Compiler for Components
Location: $GMP_PRO_LOCATION/tools/SDPE/bin_py/debug_paradigms.py
Outputs formatted preview C files with Header Guards to $GMP_PRO_LOCATION/tools/SDPE/test/
"""

import os
import sys
import argparse
import json

# Ensure we can find sdpe_compiler.py in the same directory
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
from sdpe_compiler import SDPECompiler

def format_code_block(title, content):
    """Formats a specific logic block with comments for better readability."""
    buffer = [f"/* --- {title} --- */"]
    if content and content.strip():
        buffer.append(content.strip())
    else:
        buffer.append("/* --- NULL / EMPTY --- */")
    buffer.append("") 
    return "\n".join(buffer)

def debug_compile_main(paradigm_name, target_instance=None, mode_override=None):
    # 1. Setup Absolute Paths
    sdpe_root = os.path.dirname(current_dir)
    paradigms_dir = os.path.join(sdpe_root, "paradigms")
    inst_dir = os.path.join(sdpe_root, "inst")
    templates_dir = os.path.join(sdpe_root, "templates")
    output_dir = os.path.join(sdpe_root, "test")
    
    print("=" * 74)
    print(" SDPE BATCH DEBUG COMPILER (Enhanced Preview)")
    print(f" Root: {sdpe_root}")
    print("=" * 74)
    
    # 2. Initialize compiler
    compiler = SDPECompiler(
        paradigms_dir=paradigms_dir,
        inst_dir=inst_dir,
        templates_dir=templates_dir
    )

    paradigm = compiler.get_paradigm(paradigm_name)
    if not paradigm:
        print(f"[X] Error: Could not load paradigm '{paradigm_name}'")
        return

    # 3. Resolve Instances
    db_refs = paradigm.get("instance_databases", [])
    if target_instance:
        inst_data, _ = compiler.find_instance(target_instance, db_refs)
        if not inst_data:
            print(f"[X] Error: Instance '{target_instance}' not found in {db_refs}")
            return
        instances_to_compile = {target_instance: inst_data}
    else:
        instances_to_compile = compiler.get_all_instances(db_refs)
        if not instances_to_compile:
            print(f"[!] No instances found for paradigm '{paradigm_name}'.")
            return
        print(f"[*] Found {len(instances_to_compile)} instances. Generating previews...")

    # 4. Mock Context (Fake parameters for template resolution)
    fake_context = {}
    for req in paradigm.get("required_context", []):
        ctx_name = req["name"] if isinstance(req, dict) else req
        fake_context[ctx_name] = f"/* MOCK_REF: {ctx_name} */"

    os.makedirs(output_dir, exist_ok=True)

    # 5. Compile and Format Output
    for inst_name, instance_raw_data in instances_to_compile.items():
        print(f"[*] Rendering Instance: {inst_name}")
        
        # Unpack parameter_values if nested
        if isinstance(instance_raw_data, dict) and "parameter_values" in instance_raw_data:
            render_data = instance_raw_data["parameter_values"]
        else:
            render_data = instance_raw_data

        blocks, prod_path, _ = compiler.render_component(
            paradigm_name=paradigm_name,
            instance_name=inst_name,
            instance_data={"parameter_values": render_data}, 
            mode_override=mode_override,
            external_context=fake_context
        )
        
        guard_name = f"_FILE_SDPE_DBG_PREVIEW_{inst_name.upper()}_H_"
        
        # Build Output Buffer
        output_buffer = [
            "// " + "="*72,
            f"// SDPE DEBUG PREVIEW FOR INSTANCE: {inst_name}",
            f"// PARADIGM: {paradigm_name}",
            f"// INTENDED PRODUCTION PATH: {prod_path}",
            "// " + "="*72 + "\n"
        ]
        
        if blocks["error"]:
            output_buffer.append(f"// [!] RENDER ERROR: {blocks['error']}\n")
        else:
            # --- HEADER SECTION ---
            output_buffer.extend([
                "// " + "="*72,
                "// >>> HEADER FILE DOMAIN (.h / PRESETS) <<<",
                "// " + "="*72,
                f"#ifndef {guard_name}",
                f"#define {guard_name}\n",
                "#ifdef __cplusplus",
                'extern "C" {',
                "#endif // __cplusplus\n"
            ])
            
            output_buffer.append(format_code_block("CONFIG MACROS (Macros & Parameters)", blocks["config"]))
            output_buffer.append(format_code_block("DECLARATION (Object/Handle Decl)", blocks["decl"]))

            output_buffer.extend([
                "#ifdef __cplusplus",
                "}",
                "#endif // __cplusplus\n",
                f"#endif // {guard_name}\n\n",
                
                # --- SOURCE SECTION ---
                "// " + "="*72,
                "// >>> SOURCE FILE DOMAIN (.c / CALLBACKS) <<<",
                "// " + "="*72 + "\n"
            ])
            
            output_buffer.append(format_code_block("INITIALIZATION (Constructor Logic)", blocks["init"]))
            output_buffer.append(format_code_block("INPUT CALLBACK (Data Scaling)", blocks["input"]))
            output_buffer.append(format_code_block("OUTPUT CALLBACK (Modulation/Drive)", blocks["output"]))

        # 6. Write to File
        out_filepath = os.path.join(output_dir, f"{inst_name}_preview.c")
        with open(out_filepath, "w", encoding="utf-8") as f:
            f.write("\n".join(output_buffer))
            
    print(f"\n[OK] {len(instances_to_compile)} debug preview(s) saved to: {output_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SDPE Component Debug Compiler")
    parser.add_argument("-p", "--paradigm", required=True, help="Paradigm name")
    parser.add_argument("-i", "--instance", help="Specific instance name")
    parser.add_argument("-m", "--mode", help="Specific mode override")
    
    args = parser.parse_args()
    debug_compile_main(args.paradigm, args.instance, args.mode)