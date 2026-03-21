import os
import sys
import json
import shutil
import glob
from pathlib import Path

def get_macros(dic_path, gmp_location):
    """Load macros from the dictionary and inject core environment variables."""
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
    """Super Path Resolution Engine: Handles macro replacement and ** globstar expansion."""
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
                if p.is_dir():
                    resolved_roots.add(p.resolve())
        else:
            p = Path(res_pat)
            if p.is_dir():
                resolved_roots.add(p.resolve())
                
    return list(resolved_roots)

def run_distribution():
    print("=" * 60)
    print("[START] [GMP Fleet] Starting Distribution Engine (Safe Distribution Mode)...")
    print("=" * 60)

    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("[ERROR] Environment variable GMP_PRO_LOCATION not found!")
        return False

    base_dir = Path(__file__).parent.resolve()
    target_json = base_dir / "deploy_targets.json"
    template_dir = base_dir / "gmp_src_mgr"
    dic_path = base_dir / "gmp_framework_dic.json"

    if not target_json.exists():
        print(f"[ERROR] Target config not found: {target_json.name}")
        return False

    if not template_dir.exists() or not any(template_dir.iterdir()):
        print(f"[ERROR] Template directory not found or empty: {template_dir.name}")
        print("Please place the .bat scripts to distribute in this directory!")
        return False

    # ================= Core Defense Mechanism: Distribution Blacklist =================
    # These files are strictly forbidden from overwriting downstream projects, 
    # even if they accidentally exist in the template folder!
    EXCLUDE_FILES = {
        "gmp_framework_config.json", 
        "gmp_compiler_includes.txt", 
        "deploy_targets.json",
        ".gmpignore"
    }

    # Filter out blacklisted files when reading the template directory
    files_to_deploy = [
        f for f in template_dir.iterdir() 
        if f.is_file() and f.name not in EXCLUDE_FILES
    ]

    if not files_to_deploy:
        print("[WARNING] No valid files to distribute in the template folder! (Blacklisted files filtered)")
        return False
        
    sorted_macros = get_macros(dic_path, gmp_location)

    with open(target_json, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    search_paths = config.get("search_paths", [])
    if not search_paths:
        print("[WARNING] 'search_paths' list is empty. No directories to scan.")
        return True

    print("[INFO] Parsing search path rules...")
    active_roots = resolve_search_roots(search_paths, sorted_macros)
    
    if not active_roots:
        print("[WARNING] Failed to resolve any existing physical directories. Please check path configurations or macros!")
        return True

    print("-" * 60)
    print("[INFO] Deep scanning for target projects...\n")
    
    target_dirs = set()

    # Phase 1: Scan and print discovered projects in real-time
    for root_path in active_roots:
        if root_path.name == "gmp_src_mgr":
            print(f"  [FOUND] Target discovered: {root_path.parent.name} ({root_path})")
            target_dirs.add(root_path)
            continue
            
        for dirpath, dirnames, filenames in os.walk(root_path):
            current_dir = Path(dirpath)
            if current_dir.name == "gmp_src_mgr":
                print(f"  [FOUND] Target discovered: {current_dir.parent.name} ({current_dir})")
                target_dirs.add(current_dir)

    if not target_dirs:
        print("\n[WARNING] Scan completed, no target projects found.")
        return True

    # Phase 2: Execute safe physical overwrite
    print("\n" + "-" * 60)
    print("[INFO] Scan completed. Distributing/overwriting toolchain to target projects...\n")
    
    success_count = 0
    for current_dir in sorted(list(target_dirs)):
        print(f"  [DEPLOY] Deploying to project: {current_dir.parent.name}")
        try:
            for src_file in files_to_deploy:
                dest_file = current_dir / src_file.name
                shutil.copy2(src_file, dest_file)
                print(f"    -> [OVERWRITE] {src_file.name}")
            success_count += 1
        except Exception as e:
            print(f"    -> [ERROR] Overwrite failed: {e}")

    print("\n" + "=" * 60)
    print(f"[SUMMARY] Distribution complete! Successfully updated toolchain for {success_count} project(s).")
    print("=" * 60)
    return True

if __name__ == "__main__":
    if not run_distribution():
        sys.exit(1)