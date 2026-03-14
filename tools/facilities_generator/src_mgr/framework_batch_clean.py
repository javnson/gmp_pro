import os
import sys
import json
import shutil
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
        except Exception:
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

def run_batch_clean():
    print("=" * 60)
    print("🧹 [GMP Fleet] Starting Batch Cleanup Engine (Clean Mode)...")
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

    print("[INFO] Scanning for target projects...")
    for root_path in active_roots:
        if root_path.name == "gmp_src_mgr":
            projects_found.add(root_path)
            continue
            
        for dirpath, dirnames, filenames in os.walk(root_path):
            current_dir = Path(dirpath)
            if current_dir.name == "gmp_src_mgr":
                projects_found.add(current_dir)

    if not projects_found:
        print("[WARNING] No matching projects found.")
        return True

    print(f"[INFO] Found {len(projects_found)} projects. Initiating cleansing protocol...")
    print("-" * 60)

    # Define STRICT targets to delete (Safety First)
    CLEAN_DIRS = ["gmp_src", "gmp_inc", "gmp_framework_src", "gmp_includes"] # Handle both old and new folder names
    CLEAN_FILES = ["gmp_compiler_includes.txt"]
    CLEAN_BATS = ["gmp_config.bat", "gmp_generate_src.bat", "gmp_generate_inc.bat"]

    stats = {"projects_cleaned": 0, "dirs_removed": 0, "files_removed": 0}

    for proj_dir in sorted(list(projects_found)):
        proj_root = proj_dir.parent # The actual user project root
        print(f"\n>>> [CLEANING] Project: {proj_root.name} ({proj_root})")
        
        cleaned_something = False

        # 1. Clean generated source/header directories in project root
        for d_name in CLEAN_DIRS:
            d_path = proj_root / d_name
            if d_path.exists() and d_path.is_dir():
                try:
                    shutil.rmtree(d_path)
                    print(f"    🗑️ [REMOVED DIR] {d_name}/")
                    stats["dirs_removed"] += 1
                    cleaned_something = True
                except Exception as e:
                    print(f"    ❌ [ERROR] Could not remove {d_name}: {e}")

        # 2. Clean generated text files in project root
        for f_name in CLEAN_FILES:
            f_path = proj_root / f_name
            if f_path.exists() and f_path.is_file():
                try:
                    f_path.unlink()
                    print(f"    🗑️ [REMOVED FILE] {f_name}")
                    stats["files_removed"] += 1
                    cleaned_something = True
                except Exception as e:
                    print(f"    ❌ [ERROR] Could not remove {f_name}: {e}")

        # 3. Clean distributed batch scripts inside gmp_src_mgr
        for b_name in CLEAN_BATS:
            b_path = proj_dir / b_name
            if b_path.exists() and b_path.is_file():
                try:
                    b_path.unlink()
                    print(f"    🗑️ [REMOVED SCRIPT] gmp_src_mgr/{b_name}")
                    stats["files_removed"] += 1
                    cleaned_something = True
                except Exception as e:
                    print(f"    ❌ [ERROR] Could not remove {b_name}: {e}")

        if cleaned_something:
            stats["projects_cleaned"] += 1
        else:
            print("    ✨ [CLEAN] Already clean. Nothing to remove.")

    print("\n" + "=" * 60)
    print("📊 [FLEET SUMMARY] Cleanup Protocol Completed!")
    print(f"    🧹 Projects Processed: {stats['projects_cleaned']}")
    print(f"    📁 Directories Freed: {stats['dirs_removed']}")
    print(f"    📄 Files/Scripts Freed: {stats['files_removed']}")
    print("=" * 60)
    return True

if __name__ == "__main__":
    if not run_batch_clean():
        sys.exit(1)