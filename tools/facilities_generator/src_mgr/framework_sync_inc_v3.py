import os
import sys
import json
import shutil
from pathlib import Path

def run_inc_sync():
    print("\n" + "=" * 60)
    print("[START] [GMP Sync] 开始正向同步头文件树 (Header Mirror Mode)...")
    print("=" * 60)

    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("[ERROR] 未找到环境变量 GMP_PRO_LOCATION！")
        return False

    gmp_base = Path(gmp_location).resolve()
    global_dic_path = gmp_base / "tools" / "facilities_generator" / "src_mgr" / "gmp_framework_dic.json"
    cwd = Path(os.getcwd())
    local_config_path = cwd / "gmp_framework_config.json"
    dest_inc_dir = cwd / "gmp_inc"
    dest_inc_txt = cwd / "gmp_compiler_includes.txt"

    if not global_dic_path.exists() or not local_config_path.exists():
        print("[WARNING] 找不到全局字典或本地配置，跳过同步。")
        return True

    with open(global_dic_path, 'r', encoding='utf-8') as f:
        global_registry = json.load(f)
    with open(local_config_path, 'r', encoding='utf-8') as f:
        local_config = json.load(f)

    sync_mode = local_config.get("sync_mode", "all")
    if sync_mode not in ("all", "inc_only"):
        print("[SKIP] 当前配置模式无需同步头文件。")
        return True

    macros = global_registry.get("macros", {})
    macros["GMP_PRO_LOCATION"] = gmp_base.as_posix()
    sorted_macros = sorted(macros.items(), key=lambda item: len(item[1]), reverse=True)

    # 1. 收集规则
    all_inc_patterns, all_inc_dirs_patterns = [], []
    for item in local_config.get("selected_modules", []):
        r, m = item.get("root"), item.get("module")
        if r in global_registry["modules"] and m in global_registry["modules"][r]:
            all_inc_patterns.extend(global_registry["modules"][r][m].get("inc_patterns", []))
            all_inc_dirs_patterns.extend(global_registry["modules"][r][m].get("inc_dirs", []))

    # 2. 智能镜像树推导 { "relative/path/to/file.h": absolute_path }
    inc_map = {}
    for pat in all_inc_patterns:
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

    # 3. 执行同步与裁剪
    dest_inc_dir.mkdir(parents=True, exist_ok=True)
    valid_dest_paths = { (dest_inc_dir / rel).resolve() for rel in inc_map.keys() }
    stats = {'CREATE': 0, 'UPDATE': 0, 'SKIP': 0, 'DELETE': 0}

    # -- 裁剪无用文件和空目录 --
    for root_dir, dirs, files in os.walk(dest_inc_dir, topdown=False):
        root_path = Path(root_dir).resolve()
        for file in files:
            file_path = root_path / file
            if file_path not in valid_dest_paths:
                file_path.unlink()
                print(f"  [DELETE] {file_path.relative_to(dest_inc_dir).as_posix()}")
                stats['DELETE'] += 1
        
        if not os.listdir(root_path) and root_path != dest_inc_dir.resolve():
            root_path.rmdir()

    # -- 增量拷贝 --
    for rel_path, src_abs_path in inc_map.items():
        dest_path = dest_inc_dir / rel_path
        dest_path.parent.mkdir(parents=True, exist_ok=True)
        
        if not dest_path.exists():
            shutil.copy2(src_abs_path, dest_path)
            print(f"  [CREATE] {rel_path}")
            stats['CREATE'] += 1
        elif dest_path.stat().st_mtime < src_abs_path.stat().st_mtime or dest_path.stat().st_size != src_abs_path.stat().st_size:
            shutil.copy2(src_abs_path, dest_path)
            print(f"  [UPDATE] {rel_path}")
            stats['UPDATE'] += 1
        else:
            print(f"  [SKIP]   {rel_path}")
            stats['SKIP'] += 1

    # 4. 生成 Include 指南
    inc_dirs_list = set()
    for pat in all_inc_dirs_patterns:
        resolved_pat = pat
        for mac, val in sorted_macros:
            resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)
        pat_obj = Path(resolved_pat)
        if pat_obj.is_absolute() and pat_obj.is_dir():
            inc_dirs_list.add(pat_obj.as_posix())
        else:
            target = gmp_base / resolved_pat
            if target.is_dir(): inc_dirs_list.add(target.as_posix())

    try:
        with open(dest_inc_txt, 'w', encoding='utf-8') as f:
            f.write("========== GMP 编译器 Include Paths 汇总 ==========\n")
            f.write("1. 本地镜像基础路径:\n")
            f.write(f".\\{dest_inc_dir.name}\n\n")
            if inc_dirs_list:
                f.write("2. 外部宏依赖路径 (-I):\n")
                for d in sorted(list(inc_dirs_list)):
                    f.write(f"{d}\n")
    except Exception as e:
        print(f"  [ERROR] 生成 Include 列表文件失败: {e}")

    print("\n[SUMMARY] 头文件同步完成: " + " | ".join(f"{k}:{v}" for k, v in stats.items()))
    print("=" * 60)
    return True

if __name__ == "__main__":
    if not run_inc_sync(): sys.exit(1)
    sys.exit(0)