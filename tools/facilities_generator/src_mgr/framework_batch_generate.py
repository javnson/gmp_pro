import os
import sys
import json
import subprocess
import glob
from pathlib import Path

def get_macros(dic_path, gmp_location):
    macros = {"GMP_PRO_LOCATION": Path(gmp_location).as_posix()}
    if dic_path.exists():
        try:
            with open(dic_path, 'r', encoding='utf-8') as f:
                dic_data = json.load(f)
                dic_macros = dic_data.get("macros", {})
                for k, v in dic_macros.items():
                    macros[k] = v
        except Exception as e:
            pass
    return sorted(macros.items(), key=lambda item: len(item[0]), reverse=True)

def resolve_search_roots(search_paths, sorted_macros):
    resolved_roots = set()
    for pat in search_paths:
        res_pat = pat
        for mac, val in sorted_macros:
            res_pat = res_pat.replace(f"${{{mac}}}", val)
        res_pat = res_pat.replace('\\', '/')
        if '*' in res_pat:
            matches = glob.glob(res_pat, recursive=True)
            for m in matches:
                p = Path(m)
                if p.is_dir(): resolved_roots.add(p.resolve())
        else:
            p = Path(res_pat)
            if p.is_dir(): resolved_roots.add(p.resolve())
    return list(resolved_roots)

def run_batch_generation():
    print("=" * 60)
    print("[START] [GMP Fleet] Starting Batch Generation Engine (Macro & Glob Mode)...")
    print("=" * 60)

    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("[ERROR] Environment variable GMP_PRO_LOCATION not found!")
        return False

    base_dir = Path(__file__).parent.resolve()
    target_json = base_dir / "deploy_targets.json"
    dic_path = base_dir / "gmp_framework_dic.json"

    if not target_json.exists():
        print(f"[ERROR] Target config not found: {target_json.name}")
        return False

    sorted_macros = get_macros(dic_path, gmp_location)

    with open(target_json, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    search_paths = config.get("search_paths", [])
    
    print("[INFO] Parsing search rules...")
    active_roots = resolve_search_roots(search_paths, sorted_macros)

    projects_found = set()

    print("[INFO] Deep scanning for target projects...")
    for root_path in active_roots:
        if root_path.name == "gmp_src_mgr":
            projects_found.add(root_path)
            continue
            
        for dirpath, dirnames, filenames in os.walk(root_path):
            current_dir = Path(dirpath)
            if current_dir.name == "gmp_src_mgr":
                projects_found.add(current_dir)

    if not projects_found:
        print("[WARNING] No matching 'gmp_src_mgr' folders found.")
        return True

    print(f"[INFO] Found {len(projects_found)} target projects. Initiating batch build...")
    print("-" * 60)

    stats = {"success": 0, "failed": 0}

    for proj_dir in sorted(list(projects_found)):
        print(f"\n>>> [BUILDING] Project: {proj_dir.parent.name} ({proj_dir})")
        
        script_src = proj_dir / "gmp_generate_src.bat"
        script_inc = proj_dir / "gmp_generate_inc.bat"
        
        has_error = False
        creationflags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0

        if script_src.exists():
            print("    -> Executing source file generation (Source)...")
            res_src = subprocess.run([str(script_src)], cwd=proj_dir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, creationflags=creationflags, encoding='utf-8', errors='replace')
            if res_src.returncode != 0:
                print(f"    [ERROR] Source file generation failed (Error Code: {res_src.returncode})")
                has_error = True
            else:
                print("    [OK] Source file generation successful.")
        else:
            print("    [SKIP] Source generation script not found.")

        if script_inc.exists() and not has_error:
            print("    -> Executing header file mirroring (Header)...")
            res_inc = subprocess.run([str(script_inc)], cwd=proj_dir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, creationflags=creationflags, encoding='utf-8', errors='replace')
            if res_inc.returncode != 0:
                print(f"    [ERROR] Header file mirroring failed (Error Code: {res_inc.returncode})")
                has_error = True
            else:
                print("    [OK] Header file mirroring successful.")
        elif not script_inc.exists():
            print("    [SKIP] Header generation script not found.")

        if has_error:
            stats["failed"] += 1
            print("    [ABORT] Project build aborted due to errors.")
        else:
            stats["success"] += 1
            print("    [SUCCESS] All generation tasks completed for this project.")

    print("\n" + "=" * 60)
    print("[FLEET SUMMARY] Fleet batch build completed!")
    print(f"    [SUCCESS] Projects successfully updated: {stats['success']}")
    print(f"    [FAILED] Projects with errors: {stats['failed']}")
    print("=" * 60)
    return True

if __name__ == "__main__":
    if not run_batch_generation():
        sys.exit(1)