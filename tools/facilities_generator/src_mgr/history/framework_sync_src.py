import os
import sys
import json
import shutil
from pathlib import Path

def run_src_sync():
    print("=" * 60)
    print("📦 [GMP Sync] 开始正向同步源文件 (Source Flatten Mode)...")
    print("=" * 60)

    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("❌ [错误] 未找到环境变量 GMP_PRO_LOCATION！")
        return False

    gmp_base = Path(gmp_location).resolve()
    global_dic_path = gmp_base / "tools" / "facilities_generator" / "src_mgr" / "gmp_framework_dic.json"
    cwd = Path(os.getcwd())
    local_config_path = cwd / "gmp_framework_config.json"
    dest_src_dir = cwd / "gmp_src"

    if not global_dic_path.exists() or not local_config_path.exists():
        print("⚠️ [跳过] 找不到全局字典或本地配置，跳过同步。")
        return True

    with open(global_dic_path, 'r', encoding='utf-8') as f:
        global_registry = json.load(f)
    with open(local_config_path, 'r', encoding='utf-8') as f:
        local_config = json.load(f)

    # 检查模式
    sync_mode = local_config.get("sync_mode", "all")
    if sync_mode not in ("all", "src_only"):
        print("⏭️ [跳过] 当前配置模式无需同步源文件。")
        return True

    macros = global_registry.get("macros", {})
    macros["GMP_PRO_LOCATION"] = gmp_base.as_posix()
    sorted_macros = sorted(macros.items(), key=lambda item: len(item[1]), reverse=True)

    # 1. 收集源文件模式
    all_src_patterns = []
    for item in local_config.get("selected_modules", []):
        r, m = item.get("root"), item.get("module")
        if r in global_registry["modules"] and m in global_registry["modules"][r]:
            all_src_patterns.extend(global_registry["modules"][r][m].get("src_patterns", []))

    # 2. 解析为扁平化映射表 { "filename.c": absolute_path }
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
            # 防呆：检测扁平化重名冲突
            if file_name in src_map and src_map[file_name] != f_abs:
                print(f"  [!] 警告: 源文件重名冲突！{f_abs} 将覆盖 {src_map[file_name]}")
            src_map[file_name] = f_abs

    # 3. 执行同步与裁剪
    dest_src_dir.mkdir(parents=True, exist_ok=True)
    valid_dest_files = { (dest_src_dir / fname).resolve() for filename in src_map.keys() }
    stats = {'CREATE': 0, 'UPDATE': 0, 'SKIP': 0, 'DELETE': 0}

    # -- 裁剪无用文件 --
    for file_path in dest_src_dir.iterdir():
        if file_path.is_file() and file_path.resolve() not in valid_dest_files:
            file_path.unlink()
            print(f"  [DELETE] {file_path.name}")
            stats['DELETE'] += 1

    # -- 增量拷贝 --
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

    print("\n📊 源文件同步完成: " + " | ".join(f"{k}:{v}" for k, v in stats.items()))
    return True

if __name__ == "__main__":
    if not run_src_sync(): sys.exit(1)
    sys.exit(0)