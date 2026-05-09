# -*- coding: utf-8 -*-
"""
SDPE Batch Builder & Database Validator
Searches all instances in the 'inst/' directory and its subdirectories,
and compiles them using the production sdpe_cli.
"""

import os
import json
import subprocess
import time

def load_json(filepath):
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception:
        try:
            with open(filepath, 'r', encoding='gbk') as f:
                return json.load(f)
        except Exception:
            return None

def build_all_instances():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    inst_dir = os.path.join(base_dir, "inst")
    cli_path = os.path.join(base_dir, "bin_py", "sdpe_cli.py")
    
    tasks = []
    
    # 1. Traverse database files
    print("=" * 60)
    print(" SDPE BATCH BUILDER & VALIDATOR")
    print("=" * 60)
    print("[*] Scanning database files...")
    
    for root, _, files in os.walk(inst_dir):
        for file in files:
            if file.endswith(".json"):
                filepath = os.path.join(root, file)
                data = load_json(filepath)
                if data and "instances" in data and "paradigm_ref" in data:
                    paradigm_ref = data["paradigm_ref"]
                    for inst_name in data["instances"].keys():
                        tasks.append((paradigm_ref, inst_name, filepath))

    total_tasks = len(tasks)
    print(f"[*] Found {total_tasks} instances to build.\n")

    # 2. Execute CLI for each task
    success_count = 0
    fail_count = 0
    errors = []

    start_time = time.time()

    for idx, (paradigm, instance, filepath) in enumerate(tasks, 1):
        print(f"[{idx}/{total_tasks}] Building {instance} ({paradigm})...", end=" ", flush=True)
        
        cmd = ["python", cli_path, "-p", paradigm, "-i", instance]
        result = subprocess.run(cmd, capture_output=True, text=True, cwd=base_dir)

        if result.returncode == 0:
            print("OK")
            success_count += 1
        else:
            print("FAIL")
            fail_count += 1
            # Extract the error message from stderr
            err_msg = result.stderr.strip().split('\n')[-1]
            errors.append(f"  - {instance} ({os.path.basename(filepath)}): {err_msg}")

    # 3. Report Generation
    elapsed = time.time() - start_time
    print("\n" + "=" * 60)
    print(" BUILD SUMMARY")
    print("=" * 60)
    print(f" Total Built : {total_tasks}")
    print(f" Success     : {success_count}")
    print(f" Failed      : {fail_count}")
    print(f" Time Elapsed: {elapsed:.2f} seconds")
    
    if fail_count > 0:
        print("\n[!] DETAILED ERRORS:")
        for err in errors:
            print(err)
        print("\n[!] Build completed with errors. Please fix the database entries above.")
    else:
        print("\n[+] All instances compiled perfectly! Your hardware presets are ready.")

if __name__ == "__main__":
    build_all_instances()