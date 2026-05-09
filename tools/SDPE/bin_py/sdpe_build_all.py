# -*- coding: utf-8 -*-
"""
SDPE Full Library Build Automator
Scans all paradigms and generates all official header files.
"""

import os
import sys
import time

# 确保能找到同目录下的 build 核心逻辑
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
from sdpe_build_library import build_library_main

def build_all_official_headers():
    # 1. 定位路径
    sdpe_root = os.path.dirname(current_dir)
    paradigms_dir = os.path.join(sdpe_root, "paradigms")
    
    # 获取环境变量用于日志展示
    lib_root = os.getenv("GMP_PRO_LOCATION", "LOCAL_ROOT")

    print("=" * 74)
    print(" SDPE FULL LIBRARY BUILDER (Official Distribution)")
    print(f" Environment Root : {lib_root}")
    print(f" Scanning Path    : {paradigms_dir}")
    print("=" * 74)

    # 2. 递归扫描所有的 Paradigm JSON
    paradigm_list = []
    for root, _, files in os.walk(paradigms_dir):
        for f in files:
            if f.endswith(".json"):
                # 获取范式的相对名称
                p_name = f.replace(".json", "")
                paradigm_list.append(p_name)

    if not paradigm_list:
        print("[X] No paradigm definitions found. Build aborted.")
        return

    print(f"[*] Found {len(paradigm_list)} paradigms to process.")
    
    start_time = time.time()
    success_count = 0

    # 3. 逐个执行构建
    for p_name in paradigm_list:
        print(f"\n[>>>] Processing Paradigm: {p_name}")
        try:
            # 调用 Tool 3 的核心函数，不指定 instance 则默认构建该范式下的所有型号
            build_library_main(paradigm_name=p_name)
            success_count += 1
        except Exception as e:
            print(f" [!] Build failed for {p_name}: {e}", file=sys.stderr)

    # 4. 统计结果
    duration = time.time() - start_time
    print("\n" + "=" * 74)
    print(f" [FINISH] Build Task Completed.")
    print(f" - Paradigms Processed : {success_count}/{len(paradigm_list)}")
    print(f" - Total Time Taken    : {duration:.2f} seconds")
    print("=" * 74)

if __name__ == "__main__":
    build_all_official_headers()