import os
import sys
import json
import shutil
from pathlib import Path

def run_forward_sync():
    print("=" * 60)
    print("🚀 [GMP Framework Sync] 开始执行框架正向同步引擎...")
    print("=" * 60)

    # ================= 1. 基础路径与环境检查 =================
    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("❌ [错误] 未找到环境变量 GMP_PRO_LOCATION！请先配置系统环境变量。")
        return False

    gmp_base = Path(gmp_location).resolve()
    global_dic_path = gmp_base / "tools" / "facilities_generator" / "src_mgr" / "gmp_framework_dic.json"
    
    cwd = Path(os.getcwd())
    local_config_path = cwd / "gmp_framework_config.json"
    
    dest_src_dir = cwd / "gmp_framework_src"
    dest_inc_dir = cwd / "gmp_includes"
    dest_inc_txt = cwd / "gmp_compiler_includes.txt"

    if not global_dic_path.exists():
        print(f"❌ [错误] 找不到全局字典: {global_dic_path}")
        return False
        
    if not local_config_path.exists():
        print(f"⚠️ [跳过] 当前目录未检测到 {local_config_path.name}，无需同步。")
        return True

    # ================= 2. 加载配置与宏 =================
    try:
        with open(global_dic_path, 'r', encoding='utf-8') as f:
            global_registry = json.load(f)
        with open(local_config_path, 'r', encoding='utf-8') as f:
            local_config = json.load(f)
    except Exception as e:
        print(f"❌ [错误] 读取 JSON 失败: {e}")
        return False

    sync_mode = local_config.get("sync_mode", "all")
    print(f"📌 检测到同步模式: [{sync_mode.upper()}]")

    macros = global_registry.get("macros", {})
    macros["GMP_PRO_LOCATION"] = gmp_base.as_posix()
    # 按照路径长度降序排列，确保长路径优先替换
    sorted_macros = sorted(macros.items(), key=lambda item: len(item[1]), reverse=True)

    # ================= 3. 提取选定模块的规则 =================
    all_src_patterns, all_inc_patterns, all_inc_dirs_patterns = [], [], []

    for item in local_config.get("selected_modules", []):
        r = item.get("root")
        m = item.get("module")
        try:
            mod_data = global_registry["modules"][r][m]
            all_src_patterns.extend(mod_data.get("src_patterns", []))
            all_inc_patterns.extend(mod_data.get("inc_patterns", []))
            all_inc_dirs_patterns.extend(mod_data.get("inc_dirs", []))
        except KeyError:
            print(f"  [!] 警告: 本地配置中的模块 '{r}|{m}' 在全局库中已丢失，已跳过。")

    # ================= 4. 智能镜像解析引擎 =================
    def resolve_and_mirror(patterns):
        """解析通配符，并计算出 1:1 镜像的相对目标路径"""
        results = {} # target_rel_path : absolute_source_path
        for pat in patterns:
            resolved_pat = pat
            for mac, val in sorted_macros:
                resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)
            
            pat_obj = Path(resolved_pat)
            matched_files = []
            
            if pat_obj.is_absolute():
                anchor = pat_obj.anchor
                rest = str(pat_obj.relative_to(anchor))
                for f in Path(anchor).glob(rest):
                    if f.is_file(): matched_files.append(f.resolve())
            else:
                for f in gmp_base.glob(resolved_pat):
                    if f.is_file(): matched_files.append(f.resolve())
                    
            for f_abs in matched_files:
                # 情况A: 在 GMP 内部 -> 直接复刻相对路径
                try:
                    rel = f_abs.relative_to(gmp_base)
                    results[rel.as_posix()] = f_abs
                    continue
                except ValueError:
                    pass
                
                # 情况B: 在其他宏内 -> 建专属 _ext_ 文件夹
                matched_ext = False
                for mac, val in sorted_macros:
                    if mac == "GMP_PRO_LOCATION": continue
                    try:
                        rel = f_abs.relative_to(Path(val).resolve())
                        results[f"_ext_{mac}/{rel.as_posix()}"] = f_abs
                        matched_ext = True
                        break
                    except ValueError:
                        pass
                
                # 情况C: 游离绝对路径 -> 扁平化兜底
                if not matched_ext:
                    results[f_abs.name] = f_abs
                    
        return results

    def resolve_inc_dirs(patterns):
        results = set()
        for pat in patterns:
            resolved_pat = pat
            for mac, val in sorted_macros:
                resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)
            pat_obj = Path(resolved_pat)
            if pat_obj.is_absolute() and pat_obj.is_dir():
                results.add(pat_obj.as_posix())
            else:
                target = gmp_base / resolved_pat
                if target.is_dir(): results.add(target.as_posix())
        return sorted(list(results))

    # ================= 5. 增量同步与裁剪功能 =================
    stats = {'copied': 0, 'skipped': 0, 'deleted': 0}

    def sync_and_prune(file_map, dest_root):
        dest_root = Path(dest_root)
        dest_root.mkdir(parents=True, exist_ok=True)
        valid_dest_paths = { (dest_root / rel).resolve() for rel in file_map.keys() }
        
        # 1. 废弃文件与空文件夹清理 (自底向上扫描)
        for root_dir, dirs, files in os.walk(dest_root, topdown=False):
            root_path = Path(root_dir).resolve()
            for file in files:
                file_path = root_path / file
                if file_path not in valid_dest_paths:
                    file_path.unlink()
                    stats['deleted'] += 1
            
            # 删除空文件夹
            if not os.listdir(root_path) and root_path != dest_root.resolve():
                root_path.rmdir()

        # 2. 增量复制
        for rel_path, src_abs_path in file_map.items():
            dest_path = dest_root / rel_path
            dest_path.parent.mkdir(parents=True, exist_ok=True)
            
            need_copy = True
            if dest_path.exists() and src_abs_path.stat().st_mtime <= dest_path.stat().st_mtime:
                need_copy = False
                    
            if need_copy:
                shutil.copy2(src_abs_path, dest_path)
                stats['copied'] += 1
            else:
                stats['skipped'] += 1

    # 执行核心逻辑：根据 sync_mode 分离部署
    if sync_mode in ("all", "src_only"):
        print(">> 正在同步 Source 源文件...")
        src_map = resolve_and_mirror(all_src_patterns)
        sync_and_prune(src_map, dest_src_dir)
        
    if sync_mode in ("all", "inc_only"):
        print(">> 正在同步 Header 头文件树...")
        inc_map = resolve_and_mirror(all_inc_patterns)
        sync_and_prune(inc_map, dest_inc_dir)
        
        print(">> 正在生成编译器 Include Paths 指南...")
        inc_dirs_list = resolve_inc_dirs(all_inc_dirs_patterns)
        try:
            with open(dest_inc_txt, 'w', encoding='utf-8') as f:
                f.write("========== GMP 编译器 Include Paths 汇总 ==========\n")
                f.write("请将以下路径加入到您的 IDE (Keil/IAR/GCC) 的 C/C++ 包含目录中:\n\n")
                f.write("1. 本地镜像基础路径 (必填):\n")
                f.write(f".\\{dest_inc_dir.name}\n\n")
                if inc_dirs_list:
                    f.write("2. 外部/底层依赖绝对路径 (-I):\n")
                    for d in inc_dirs_list:
                        f.write(f"{d}\n")
        except Exception as e:
            print(f"  [!] 生成 Include 列表文件失败: {e}")

    print("\n✅ [同步完成] 统计信息:")
    print(f"    - 新增/更新: {stats['copied']} 个文件")
    print(f"    - 已存在跳过: {stats['skipped']} 个文件")
    print(f"    - 废弃清理:   {stats['deleted']} 个文件")
    print("=" * 60)
    return True

if __name__ == "__main__":
    if not run_forward_sync():
        sys.exit(1)
    sys.exit(0)