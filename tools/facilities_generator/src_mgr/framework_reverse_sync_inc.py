import os
import sys
import json
import shutil
import filecmp
from pathlib import Path

def run_reverse_sync_inc():
    is_apply_mode = "--apply" in sys.argv
    
    if not is_apply_mode:
        print("=" * 60)
        print("[START] [GMP Reverse Sync] Header tree diff scanning (Header Diff Mode)...")
        print("=" * 60)

    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("[ERROR] Environment variable GMP_PRO_LOCATION not found!")
        return False

    gmp_base = Path(gmp_location).resolve()
    global_dic_path = gmp_base / "tools" / "facilities_generator" / "src_mgr" / "gmp_framework_dic.json"
    cwd = Path(os.getcwd())
    local_config_path = cwd / "gmp_framework_config.json"
    inc_dir = cwd / "gmp_inc"

    if not global_dic_path.exists() or not local_config_path.exists():
        print("[ERROR] Global dictionary or local config not found!")
        return False

    with open(global_dic_path, 'r', encoding='utf-8') as f:
        global_registry = json.load(f)
    with open(local_config_path, 'r', encoding='utf-8') as f:
        local_config = json.load(f)

    macros = global_registry.get("macros", {})
    macros["GMP_PRO_LOCATION"] = gmp_base.as_posix()
    sorted_macros = sorted(macros.items(), key=lambda item: len(item[1]), reverse=True)

    # 1. Rebuild the reverse mapping table for header files
    inc_map = {}
    for item in local_config.get("selected_modules", []):
        r, m = item.get("root"), item.get("module")
        if r in global_registry["modules"] and m in global_registry["modules"][r]:
            for pat in global_registry["modules"][r][m].get("inc_patterns", []):
                resolved_pat = pat
                for mac, val in sorted_macros:
                    resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)
                
                pat_obj = Path(resolved_pat)
                if pat_obj.is_absolute():
                    anchor = pat_obj.anchor
                    rest = str(pat_obj.relative_to(anchor))
                    matched = [f.resolve() for f in Path(anchor).glob(rest) if f.is_file()]
                else:
                    matched = [f.resolve() for f in gmp_base.glob(resolved_pat) if f.is_file()]
                    
                for f_abs in matched:
                    try:
                        rel = f_abs.relative_to(gmp_base)
                        inc_map[rel.as_posix()] = f_abs
                        continue
                    except ValueError:
                        pass
                        
                    matched_ext = False
                    for mac, val in sorted_macros:
                        if mac == "GMP_PRO_LOCATION": continue
                        try:
                            rel = f_abs.relative_to(Path(val).resolve())
                            inc_map[f"_ext_{mac}/{rel.as_posix()}"] = f_abs
                            matched_ext = True
                            break
                        except ValueError:
                            pass
                    if not matched_ext:
                        inc_map[f_abs.name] = f_abs

    if not inc_dir.exists():
        print("[ERROR] Local directory 'gmp_inc' does not exist. Cannot perform diff.")
        return False

    # 2. Execute Diff comparison
    modified_files = []
    untracked_files = []

    for root_dir, _, files in os.walk(inc_dir):
        root_path = Path(root_dir)
        for file in files:
            local_file = root_path / file
            rel_path = local_file.relative_to(inc_dir).as_posix()
            
            # Absolutely do not touch external dependencies starting with _ext_!
            if rel_path.startswith("_ext_"):
                continue
                
            if rel_path not in inc_map:
                untracked_files.append(rel_path)
                continue
                
            remote_file = inc_map[rel_path]
            
            if not remote_file.exists() or not filecmp.cmp(local_file, remote_file, shallow=False):
                modified_files.append((local_file, remote_file, rel_path))

    # 3. Output report or execute overwrite
    if not is_apply_mode:
        if not modified_files and not untracked_files:
            print("[SAFE] Local header tree perfectly matches the core library. No changes to push.")
            return True
            
        print("[REPORT] Scan complete. Check results below:\n")
        
        for loc, rem, rel in modified_files:
            print(f"  [MODIFIED] Detected change: {rel}")
            print(f"      -> Will overwrite to: {rem.relative_to(gmp_base) if rem.is_relative_to(gmp_base) else rem}")
            
        for rel in untracked_files:
            print(f"  [UNTRACKED] Unknown file: {rel} (Not registered in dictionary, will be ignored)")
            
        print(f"\n[SUMMARY] A total of {len(modified_files)} file(s) can be pushed back to the core library.")
        
    else:
        print("=" * 60)
        print("[APPLY] Physically overwriting modifications back to the core library...")
        for loc, rem, rel in modified_files:
            rem.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(loc, rem)
            print(f"  [OVERWRITE] Overwritten: {rel}")
        print("\n[SUCCESS] Reverse synchronization physical write complete!")

    return True

if __name__ == "__main__":
    run_reverse_sync_inc()