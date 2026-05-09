# -*- coding: utf-8 -*-
"""
SDPE Production Library Builder (Pure Parameter Mode)
Generates static hardware constants ONLY. No handles, no IO.
"""

import os
import sys
import argparse
import json
from sdpe_compiler import SDPECompiler

def smart_write(filepath, content):
    """Only writes if content has changed to preserve mtime."""
    if os.path.exists(filepath):
        with open(filepath, 'r', encoding='utf-8') as f:
            if f.read() == content:
                return False
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(content)
    return True

def build_library_main(paradigm_name, target_instance=None, manual_path=None):
    # 路径初始化
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sdpe_root = os.path.dirname(current_dir)
    lib_root = os.getenv("GMP_PRO_LOCATION", os.path.dirname(os.path.dirname(sdpe_root)))
    
    compiler = SDPECompiler(
        paradigms_dir=os.path.join(sdpe_root, "paradigms"),
        inst_dir=os.path.join(sdpe_root, "inst"),
        templates_dir=os.path.join(sdpe_root, "templates")
    )

    paradigm = compiler.get_paradigm(paradigm_name)
    if not paradigm:
        print(f"[X] Error: Paradigm '{paradigm_name}' not found.", file=sys.stderr)
        return

    # 确定输出基准
    prod_path_rel = manual_path if manual_path else paradigm.get("output_path", "ctl/hardware_presets")
    final_abs_dir = os.path.join(lib_root, prod_path_rel)

    # 获取实例
    db_refs = paradigm.get("instance_databases", [])
    if target_instance:
        inst_data, _ = compiler.find_instance(target_instance, db_refs)
        instances = {target_instance: inst_data} if inst_data else {}
    else:
        instances = compiler.get_all_instances(db_refs)

    if not instances:
        print(f"[X] Error: No instances found for {paradigm_name}.", file=sys.stderr)
        return

    print("=" * 74)
    print(f" SDPE PARAMETER BUILDER: {paradigm_name}")
    print(f" Mode: Static Constants Only (.h)")
    print("=" * 74)

    success_count = 0
    for inst_name, raw_data in instances.items():
        render_data = raw_data["parameter_values"] if "parameter_values" in raw_data else raw_data
        
        # 渲染组件 - 此时 blocks["decl"] 等内容将被丢弃
        blocks, _, _ = compiler.render_component(
            paradigm_name=paradigm_name,
            instance_name=inst_name,
            instance_data={"parameter_values": render_data}
        )

        if blocks["error"]:
            continue

        # --- 构造极简头文件 ---
        # 只保留 blocks["config"] 中的宏定义或常量
        guard_name = f"_SDPE_HW_CONST_{inst_name.upper()}_H_"
        h_buffer = [
            "/* Auto-generated static hardware constants. DO NOT EDIT. */",
            f"#ifndef {guard_name}",
            f"#define {guard_name}\n",
            blocks["config"].strip(),
            f"\n#endif // {guard_name}"
        ]

        out_file = os.path.join(final_abs_dir, f"{inst_name}.h")
        if smart_write(out_file, "\n".join(h_buffer)):
            print(f" [+] Built: {inst_name}.h")
        else:
            print(f" [.] Unchanged: {inst_name}.h")
        success_count += 1

    print(f"\n[OK] Library build completed. ({success_count} files)")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--paradigm", required=True)
    parser.add_argument("-i", "--instance")
    parser.add_argument("-o", "--output")
    args = parser.parse_args()
    build_library_main(args.paradigm, args.instance, args.output)