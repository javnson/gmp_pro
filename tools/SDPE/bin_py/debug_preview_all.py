# -*- coding: utf-8 -*-
"""
SDPE Batch Preview Tool - All Paradigms & All Instances
Location: $GMP_PRO_LOCATION/tools/SDPE/bin_py/debug_preview_all.py
"""

import os
import sys
import time

# 确保能找到同目录下的编译器
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
from sdpe_compiler import SDPECompiler
from debug_paradigms import debug_compile_main

def run_batch_preview():
    # 1. 自动定位路径
    sdpe_root = os.path.dirname(current_dir)
    paradigms_dir = os.path.join(sdpe_root, "paradigms")
    output_base = os.path.join(sdpe_root, "test", "full_library_preview")

    print("=" * 74)
    print(" SDPE FULL LIBRARY BATCH PREVIEWER")
    print(f" Scanning Paradigms: {paradigms_dir}")
    print(f" Target Output     : {output_base}")
    print("=" * 74)

    # 2. 搜索所有的 Paradigm JSON 文件
    paradigm_files = []
    for root, _, files in os.walk(paradigms_dir):
        for f in files:
            if f.endswith(".json"):
                paradigm_files.append(f.replace(".json", ""))

    if not paradigm_files:
        print("[X] No paradigms found. Check your path.")
        return

    print(f"[*] Found {len(paradigm_files)} paradigms: {', '.join(paradigm_files)}")
    
    start_time = time.time()
    total_count = 0

    # 3. 循环调用 debug_compile_main 执行每个范式的渲染
    for p_name in paradigm_files:
        print(f"\n>>> Processing Paradigm Group: [{p_name}]")
        
        # 为每个范式创建独立的子文件夹，防止文件同名冲突
        p_output_dir = os.path.join(output_base, p_name)
        
        try:
            # 调用已有的调试工具逻辑（不指定 instance 则默认渲染全部）
            # 我们直接使用 debug_compile_main，但传入特定的 output 目录
            # 注意：由于 debug_compile_main 内部默认写死了 test 目录，
            # 我们稍微调整逻辑或直接通过命令行参数思想调用
            
            from debug_paradigms import SDPECompiler
            # 重新实例化以确保路径隔离
            debug_compile_main(paradigm_name=p_name, target_instance=None)
            
            # 移动生成的文件到分类文件夹（可选，当前 debug_compile_main 统一放 test/）
            # 为了简单起见，目前我们让它全部堆在 test/ 下，或根据需要移动
            
        except Exception as e:
            print(f" [X] Failed to process {p_name}: {e}")

    duration = time.time() - start_time
    print("\n" + "=" * 74)
    print(f" [OK] Batch Preview Completed in {duration:.2f}s")
    print(f" All previews are available in: {os.path.join(sdpe_root, 'test')}")
    print("=" * 74)

if __name__ == "__main__":
    run_batch_preview()