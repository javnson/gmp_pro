import os
import sys
import json
import shutil
import glob
from pathlib import Path

def get_macros(dic_path, gmp_location):
    """从字典中加载宏，并注入核心环境变量"""
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
    """超级路径解析引擎：处理宏替换与 ** 通配符展开"""
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
    print("🚀 [GMP Fleet] 启动分布式部署工具 (Safe Distribution Mode)...")
    print("=" * 60)

    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("[ERROR] 未找到环境变量 GMP_PRO_LOCATION！")
        return False

    base_dir = Path(__file__).parent.resolve()
    target_json = base_dir / "deploy_targets.json"
    template_dir = base_dir / "gmp_src_mgr"
    dic_path = base_dir / "gmp_framework_dic.json"

    if not target_json.exists():
        print(f"[ERROR] 找不到目标配置文件: {target_json.name}")
        return False

    if not template_dir.exists() or not any(template_dir.iterdir()):
        print(f"[ERROR] 模板文件夹不存在或为空: {template_dir.name}")
        print("请将待分发的 .bat 脚本放入该目录中！")
        return False

    # ================= 核心防御机制：分发黑名单 =================
    # 这些文件即使在母版文件夹中存在，也绝对不允许去覆盖下游工程！
    EXCLUDE_FILES = {
        "gmp_framework_config.json", 
        "gmp_compiler_includes.txt", 
        "deploy_targets.json",
        ".gmpignore"
    }

    # 读取要分发的文件时，直接过滤掉黑名单文件
    files_to_deploy = [
        f for f in template_dir.iterdir() 
        if f.is_file() and f.name not in EXCLUDE_FILES
    ]

    if not files_to_deploy:
        print("[WARNING] 模板文件夹中没有可合法分发的文件！(黑名单文件已被过滤)")
        return False
        
    sorted_macros = get_macros(dic_path, gmp_location)

    with open(target_json, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    search_paths = config.get("search_paths", [])
    if not search_paths:
        print("[WARNING] search_paths 列表为空，没有需要扫描的目录。")
        return True

    print("[INFO] 正在解析搜索路径规则...")
    active_roots = resolve_search_roots(search_paths, sorted_macros)
    
    if not active_roots:
        print("[WARNING] 未能解析到任何真实存在的物理目录，请检查路径配置或宏变量！")
        return True

    print("-" * 60)
    print("[INFO] 开始深度扫描目标工程...\n")
    
    target_dirs = set()

    # 第一阶段：扫描并实时打印发现的工程
    for root_path in active_roots:
        if root_path.name == "gmp_src_mgr":
            print(f"  🔍 [FOUND] 发现目标: {root_path.parent.name} ({root_path})")
            target_dirs.add(root_path)
            continue
            
        for dirpath, dirnames, filenames in os.walk(root_path):
            current_dir = Path(dirpath)
            if current_dir.name == "gmp_src_mgr":
                print(f"  🔍 [FOUND] 发现目标: {current_dir.parent.name} ({current_dir})")
                target_dirs.add(current_dir)

    if not target_dirs:
        print("\n[WARNING] 扫描结束，未找到任何工程目标。")
        return True

    # 第二阶段：执行安全的物理覆盖
    print("\n" + "-" * 60)
    print("[INFO] 扫描完毕，开始向目标工程分发/覆盖工具链...\n")
    
    success_count = 0
    for current_dir in sorted(list(target_dirs)):
        print(f"  [DEPLOY] 正在部署至工程: {current_dir.parent.name}")
        try:
            for src_file in files_to_deploy:
                dest_file = current_dir / src_file.name
                shutil.copy2(src_file, dest_file)
                print(f"    -> [OVERWRITE] {src_file.name}")
            success_count += 1
        except Exception as e:
            print(f"    -> [ERROR] 覆盖失败: {e}")

    print("\n" + "=" * 60)
    print(f"🎉 [SUMMARY] 分发完成！成功更新了 {success_count} 个工程的工具链。")
    print("=" * 60)
    return True

if __name__ == "__main__":
    if not run_distribution():
        sys.exit(1)