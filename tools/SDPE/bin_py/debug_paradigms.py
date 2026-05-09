# -*- coding: utf-8 -*-
"""
SDPE Debug Compiler for Components
Strict Absolute Path Version
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
    buffer = [f"/* --- {title} --- */"]
    if content and content.strip():
        buffer.append(content.strip())
    else:
        buffer.append("/* --- NULL --- */")
    buffer.append("") 
    return "\n".join(buffer)

def debug_compile_main(paradigm_name, target_instance=None, mode_override=None):
    # 1. Setup Absolute Paths
    # We assume bin_py is inside SDPE root
    sdpe_root = os.path.dirname(current_dir)
    paradigms_dir = os.path.join(sdpe_root, "paradigms")
    inst_dir = os.path.join(sdpe_root, "inst")
    output_dir = os.path.join(sdpe_root, "test")
    
    print("=" * 70)
    print(" SDPE COMPONENT DEBUGGER (Robust Path Mode)")
    print(f" SDPE Root: {sdpe_root}")
    print(f" Search P : {paradigms_dir}")
    print(f" Search I : {inst_dir}")
    print("=" * 70)
    
    # 2. Initialize compiler with Absolute Paths
    compiler = SDPECompiler(
        paradigms_dir=paradigms_dir,
        inst_dir=inst_dir,
        templates_dir=os.path.join(sdpe_root, "templates")
    )

    paradigm = compiler.get_paradigm(paradigm_name)
    if not paradigm:
        print(f"[X] Error: Could not load paradigm JSON: {paradigm_name}")
        return

    # 3. Resolve Databases
    db_refs = paradigm.get("instance_databases", [])
    print(f"[*] Referenced Databases: {db_refs}")
    
    if target_instance:
        # Pass the inst_dir to find_instance to ensure it looks in the right place
        inst_data, source_db = compiler.find_instance(target_instance, db_refs)
        if not inst_data:
            print(f"[X] Error: Instance '{target_instance}' not found in: {db_refs}")
            # Diagnostic: Print which databases were actually scanned
            return
        print(f"[OK] Found '{target_instance}' in {source_db}")
        instances_to_compile = {target_instance: inst_data}
    else:
        instances_to_compile = compiler.get_all_instances(db_refs)
        if not instances_to_compile:
            print(f"[!] No instances found in the databases for '{paradigm_name}'.")
            return
        print(f"[*] Found {len(instances_to_compile)} instances.")

    # 4. Mock Context
    fake_context = {}
    for req in paradigm.get("required_context", []):
        ctx_name = req["name"] if isinstance(req, dict) else req
        fake_context[ctx_name] = f"/* MOCK_{ctx_name.upper()} */"

    os.makedirs(output_dir, exist_ok=True)

    # 5. Render
    for inst_name, instance_raw_data in instances_to_compile.items():
        print(f"[*] Rendering: {inst_name}")
        
        # 兼容性处理：如果数据里嵌套了 "parameter_values"，则提取出来
        # 这是为了适配 db_hall_sensors.json 中的结构
        if isinstance(instance_raw_data, dict) and "parameter_values" in instance_raw_data:
            render_data = instance_raw_data["parameter_values"]
        else:
            render_data = instance_raw_data

        blocks, prod_path, _ = compiler.render_component(
            paradigm_name=paradigm_name,
            instance_name=inst_name,
            # 将提取后的物理参数传给编译器
            instance_data={"parameter_values": render_data}, 
            mode_override=mode_override,
            external_context=fake_context
        )
        
        out_content = [
            f"// PREVIEW FOR {inst_name}",
            f"// PRODUCTION PATH: {prod_path}\n"
        ]
        
        if blocks["error"]:
            out_content.append(f"/* ERROR: {blocks['error']} */")
        else:
            for b_name in ["config", "init", "input", "output"]:
                out_content.append(format_code_block(b_name.upper(), blocks[b_name]))

        out_filepath = os.path.join(output_dir, f"{inst_name}_debug.c")
        with open(out_filepath, "w", encoding="utf-8") as f:
            f.write("\n".join(out_content))
            
    print(f"\n[OK] Preview generated in: {output_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--paradigm", required=True)
    parser.add_argument("-i", "--instance")
    parser.add_argument("-m", "--mode")
    args = parser.parse_args()
    debug_compile_main(args.paradigm, args.instance, args.mode)