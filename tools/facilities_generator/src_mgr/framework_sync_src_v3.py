import os
import sys
import json
import shutil
from pathlib import Path

def run_src_sync():
    print("=" * 60)
    print("[START] [GMP Sync] Starting forward source file synchronization (Source Flatten Mode)...")
    print("=" * 60)

    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("[ERROR] Environment variable GMP_PRO_LOCATION not found!")
        return False

    gmp_base = Path(gmp_location).resolve()
    global_dic_path = gmp_base / "tools" / "facilities_generator" / "src_mgr" / "gmp_framework_dic.json"
    cwd = Path(os.getcwd())
    local_config_path = cwd / "gmp_framework_config.json"
    dest_src_dir = cwd / "gmp_src"

    if not global_dic_path.exists() or not local_config_path.exists():
        print("[WARNING] Global dictionary or local configuration not found. Skipping synchronization.")
        return True

    with open(global_dic_path, 'r', encoding='utf-8') as f:
        global_registry = json.load(f)
    with open(local_config_path, 'r', encoding='utf-8') as f:
        local_config = json.load(f)

    # Check sync mode
    sync_mode = local_config.get("sync_mode", "all")
    if sync_mode not in ("all", "src_only"):
        print("[SKIP] Current configuration mode does not require source file synchronization.")
        return True

    macros = global_registry.get("macros", {})
    macros["GMP_PRO_LOCATION"] = gmp_base.as_posix()
    sorted_macros = sorted(macros.items(), key=lambda item: len(item[1]), reverse=True)

    # 1. Collect source file patterns
    all_src_patterns = []
    for item in local_config.get("selected_modules", []):
        r, m = item.get("root"), item.get("module")
        if r in global_registry["modules"] and m in global_registry["modules"][r]:
            all_src_patterns.extend(global_registry["modules"][r][m].get("src_patterns", []))

    # 2. Parse into flattened mapping { "filename.c": absolute_path }
    src_map = {}
    for pat in all_src_patterns:
        resolved_pat = pat
        for mac, val in sorted_macros:
            resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)
            
        pat_obj = Path(resolved_pat)
        matched_files = []
        if pat_obj.is_absolute():
            anchor = pat_obj.anchor
            rest = str(pat_obj.relative_to(anchor))
            matched_files = [f.resolve() for f in Path(anchor).glob(rest) if f.is_file()]
        else:
            matched_files = [f.resolve() for f in gmp_base.glob(resolved_pat) if f.is_file()]
            
        for f_abs in matched_files:
            file_name = f_abs.name
            # Failsafe: Detect flattened naming conflicts
            if file_name in src_map and src_map[file_name] != f_abs:
                print(f"  [WARNING] Source file name conflict! {f_abs} will overwrite {src_map[file_name]}")
            src_map[file_name] = f_abs

    # 3. Execute synchronization and pruning
    dest_src_dir.mkdir(parents=True, exist_ok=True)
    
    valid_dest_files = { (dest_src_dir / filename).resolve() for filename in src_map.keys() }
    
    stats = {'CREATE': 0, 'UPDATE': 0, 'SKIP': 0, 'DELETE': 0}

    # -- Prune unused files --
    for file_path in dest_src_dir.iterdir():
        if file_path.is_file() and file_path.resolve() not in valid_dest_files:
            file_path.unlink()
            print(f"  [DELETE] {file_path.name}")
            stats['DELETE'] += 1

    # -- Incremental copy --
    for file_name, src_abs_path in src_map.items():
        dest_path = dest_src_dir / file_name
        
        if not dest_path.exists():
            shutil.copy2(src_abs_path, dest_path)
            print(f"  [CREATE] {file_name}")
            stats['CREATE'] += 1
        elif dest_path.stat().st_mtime < src_abs_path.stat().st_mtime or dest_path.stat().st_size != src_abs_path.stat().st_size:
            shutil.copy2(src_abs_path, dest_path)
            print(f"  [UPDATE] {file_name}")
            stats['UPDATE'] += 1
        else:
            print(f"  [SKIP]   {file_name}")
            stats['SKIP'] += 1

    print("\n[SUMMARY] Source file synchronization completed: " + " | ".join(f"{k}:{v}" for k, v in stats.items()))
    return True

if __name__ == "__main__":
    if not run_src_sync(): sys.exit(1)
    sys.exit(0)