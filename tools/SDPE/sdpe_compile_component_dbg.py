# -*- coding: utf-8 -*-
"""
SDPE Debug Compiler for Components
Supports batch/single compiling, displays intended production paths, 
and outputs generated preview files to a designated test directory.

usage: python sdpe_compile_component_dbg.py -p current_shunt -i inst_shunt_2mohm -m tri_channel
       python sdpe_compile_component_dbg.py -p current_shunt

"""

# -*- coding: utf-8 -*-
"""
SDPE Debug Compiler CLI
Uses the core SDPECompiler class to generate heavily standardized preview files.
Clearly separates Header (.h) and Source (.c) domains.
"""

import os
import argparse
from bin_py.sdpe_compiler import SDPECompiler

def format_code_block(title, content):
    """Format a single code block. If empty, return a standardized NULL marker."""
    buffer = [f"/* --- {title} --- */"]
    if content and content.strip():
        buffer.append(content.strip())
    else:
        buffer.append("/* --- NULL --- */")
    buffer.append("") # Add a blank line for readability
    return "\n".join(buffer)

def debug_compile_main(paradigm_name, target_instance=None, mode_override=None, output_dir="test"):
    print("=" * 70)
    print(" SDPE BATCH DEBUG COMPILER (Powered by SDPECore)")
    print("=" * 70)
    
    # 1. Initialize the Compiler Engine
    compiler = SDPECompiler(paradigms_dir="paradigms", inst_dir="inst")

    # 2. Load Paradigm
    paradigm_path = os.path.join(compiler.paradigms_dir, f"{paradigm_name}.json")
    paradigm = compiler.load_json(paradigm_path)
    if not paradigm:
        print(f"[X] Error: Could not load paradigm '{paradigm_name}'")
        return

    # 3. Fetch instances
    db_paths = paradigm.get("instance_databases", [])
    if target_instance:
        inst_data, _ = compiler.find_instance(target_instance, db_paths)
        if not inst_data:
            print(f"[X] Error: Instance '{target_instance}' not found.")
            return
        instances_to_compile = {target_instance: inst_data}
    else:
        instances_to_compile = compiler.get_all_instances(db_paths)
        if not instances_to_compile:
            print(f"[!] No instances found in the databases for '{paradigm_name}'.")
            return
        print(f"[*] Found {len(instances_to_compile)} instances. Beginning batch compile...")

    # 4. Prepare Fake Context for Debugging
    fake_context = {}
    for req in paradigm.get("required_context", []):
        ctx_name = req["name"]
        fake_context[ctx_name] = f"/* FROM_OUT: {ctx_name} */"

    # 5. Compile and Output
    for inst_name, instance_data in instances_to_compile.items():
        print(f"\n{'=' * 70}")
        print(f" >>> COMPILING INSTANCE: {inst_name} ")
        
        # *** CALLING THE CORE ENGINE ***
        blocks, prod_path, _ = compiler.render_component(
            paradigm_name=paradigm_name,
            instance_name=inst_name,
            instance_data=instance_data,
            mode_override=mode_override,
            external_context=fake_context
        )
        
        print(f"[*] Intended Production Path: {prod_path}")
        
        guard_name = f"_FILE_SDPE_DBG_PREVIEW_{inst_name.upper()}_H_"
        
        output_buffer = [
            "// ============================================================================",
            f"// SDPE DEBUG PREVIEW FOR INSTANCE: {inst_name}",
            f"// PARADIGM: {paradigm.get('paradigm_name', 'Unknown')}",
            f"// INTENDED PRODUCTION PATH: {prod_path}",
            "// ============================================================================\n"
        ]
        
        if blocks["error"]:
            output_buffer.append(f"// [!] Error: {blocks['error']}\n")
            print(f"\n[!] Template Render Error: {blocks['error']}")
        else:
            # ---------------------------------------------------------
            # 域 1: HEADER FILE PART (.h)
            # ---------------------------------------------------------
            output_buffer.extend([
                "// " + "="*74,
                "// >>> HEADER FILE DOMAIN (.h) <<<",
                "// " + "="*74,
                "/**",
                " * @file",
                " * @brief SDPE Component Debug Preview Header",
                f" * @note Generated for instance '{inst_name}'",
                " */\n",
                f"#ifndef {guard_name}",
                f"#define {guard_name}\n",
                "#ifdef __cplusplus",
                'extern "C"',
                "{",
                "#endif // __cplusplus\n"
            ])
            
            output_buffer.append(format_code_block("CONFIG MACROS (sdpe_config.h)", blocks["config"]))
            output_buffer.append(format_code_block("DECLARATION (sdpe_instances.h)", blocks["decl"]))

            output_buffer.extend([
                "#ifdef __cplusplus",
                "}",
                "#endif // __cplusplus\n",
                f"#endif // {guard_name}\n\n"
            ])

            # ---------------------------------------------------------
            # 域 2: SOURCE FILE PART (.c)
            # ---------------------------------------------------------
            output_buffer.extend([
                "// " + "="*74,
                "// >>> SOURCE FILE DOMAIN (.c / inline step functions) <<<",
                "// " + "="*74,
                ""
            ])
            
            output_buffer.append(format_code_block("INITIALIZATION (sdpe_instances.c)", blocks["init"]))
            output_buffer.append(format_code_block("INPUT CALLBACK (sdpe_io_step.h)", blocks["input"]))
            output_buffer.append(format_code_block("OUTPUT CALLBACK (sdpe_io_step.h)", blocks["output"]))

        # Write to file (后缀使用 preview.c，方便编辑器高亮两者)
        os.makedirs(output_dir, exist_ok=True)
        out_filepath = os.path.join(output_dir, f"{inst_name}_preview.c")
        with open(out_filepath, "w", encoding="utf-8") as f:
            f.write("\n".join(output_buffer))
            
        print(f"[+] Debug preview saved to: {out_filepath}")

    print("\n" + "=" * 70)
    print(f" Debug Compile Completed. Check '{output_dir}/' for output files.")
    print("=" * 70 + "\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SDPE Component Debug Compiler (Powered by Engine)")
    parser.add_argument("-p", "--paradigm", required=True, help="Paradigm name (e.g., current_shunt)")
    parser.add_argument("-i", "--instance", help="(Optional) Specific instance name. If omitted, compiles ALL.")
    parser.add_argument("-m", "--mode", help="(Optional) Specific mode to test")
    parser.add_argument("-o", "--output", default="test", help="Output directory (default: 'test')")
    
    args = parser.parse_args()
    debug_compile_main(args.paradigm, args.instance, args.mode, args.output)