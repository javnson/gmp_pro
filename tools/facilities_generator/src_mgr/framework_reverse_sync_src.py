import os
import sys
import json
import shutil
import filecmp
from pathlib import Path

def run_reverse_sync_src():
    is_apply_mode = "--apply" in sys.argv
    
    if not is_apply_mode:
        print("=" * 60)
        print("[START] [GMP Reverse Sync] 源文件比对扫描 (Source Diff Mode)...")
        print("=" * 60)

    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("[ERROR] 未找到环境变量 GMP_PRO_LOCATION！")
        return False

    gmp_base = Path(gmp_location).resolve()
    global_dic_path = gmp_base / "tools" / "facilities_generator" / "src_mgr" / "gmp_framework_dic.json"
    cwd = Path(os.getcwd())
    local_config_path = cwd / "gmp_framework_config.json"
    src_dir = cwd / "gmp_src"

    if not global_dic_path.exists() or not local_config_path.exists():
        print("[ERROR] 找不到全局字典或本地配置！")
        return False

    with open(global_dic_path, 'r', encoding='utf-8') as f:
        global_registry = json.load(f)
    with open(local_config_path, 'r', encoding='utf-8') as f:
        local_config = json.load(f)

    macros = global_registry.get("macros", {})
    macros["GMP_PRO_LOCATION"] = gmp_base.as_posix()
    sorted_macros = sorted(macros.items(), key=lambda item: len(item[1]), reverse=True)

    # 1. 依据本地配置，重建源文件的反向映射表 { "filename.c": absolute_core_path }
    src_map = {}
    for item in local_config.get("selected_modules", []):
        r, m = item.get("root"), item.get("module")
        if r in global_registry["modules"] and m in global_registry["modules"][r]:
            for pat in global_registry["modules"][r][m].get("src_patterns", []):
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
                    src_map[f_abs.name] = f_abs

    if not src_dir.exists():
        print("[ERROR] 本地不存在 gmp_src 目录，无法比对。")
        return False

    # 2. 执行 Diff 比对
    modified_files = []   # (local_path, remote_path)
    untracked_files = []  # local_path

    for local_file in src_dir.iterdir():
        if not local_file.is_file(): continue
        
        fname = local_file.name
        if fname not in src_map:
            untracked_files.append(local_file)
            continue
            
        remote_file = src_map[fname]
        
        # 使用 filecmp 深度比对文件内容
        if not remote_file.exists() or not filecmp.cmp(local_file, remote_file, shallow=False):
            modified_files.append((local_file, remote_file))

    # 3. 输出报告或执行覆盖
    if not is_apply_mode:
        if not modified_files and not untracked_files:
            print("[SAFE] 本地源文件与核心库完全一致，没有需要提交的更改。")
            return True
            
        print("[REPORT] 扫描完毕。检查结果如下：\n")
        
        for loc, rem in modified_files:
            print(f"  [MODIFIED] 发现变更: {loc.name}")
            print(f"      -> 核心库映射: {rem.relative_to(gmp_base) if rem.is_relative_to(gmp_base) else rem}")
            
        for loc in untracked_files:
            print(f"  [UNTRACKED] 未知文件: {loc.name} (该文件未在字典中注册，将被忽略)")
            
        print(f"\n[SUMMARY] 共有 {len(modified_files)} 个文件可推回核心库。")
        
    else:
        print("=" * 60)
        print("[APPLY] 正在将修改物理覆盖回核心库...")
        for loc, rem in modified_files:
            rem.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(loc, rem)
            print(f"  [OVERWRITE] 已覆盖: {rem.name}")
        print("\n[SUCCESS] 反向同步物理写入完成！")

    return True

if __name__ == "__main__":
    run_reverse_sync_src()