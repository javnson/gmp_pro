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
        except Exception as e:
            print(f"[WARNING] 无法解析字典中的宏变量: {e}")
    # 按宏名称长度降序排列，防止子串替换冲突 (如 ${C2000} 和 ${C2000WARE})
    return sorted(macros.items(), key=lambda item: len(item[0]), reverse=True)

def resolve_search_roots(search_paths, sorted_macros):
    """超级路径解析引擎：处理宏替换与 ** 通配符展开"""
    resolved_roots = set()
    for pat in search_paths:
        # 1. 宏替换
        res_pat = pat
        for mac, val in sorted_macros:
            res_pat = res_pat.replace(f"${{{mac}}}", val)
        
        res_pat = res_pat.replace('\\', '/')
        
        # 2. 通配符展开
        if '*' in res_pat:
            # 开启 recursive=True 支持 ** 跨级目录匹配
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
    print("🚀 [GMP Fleet] 启动分布式部署工具 (Macro & Glob Mode)...")
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

    # 读取宏变量
    sorted_macros = get_macros(dic_path, gmp_location)

    # 读取要分发的文件
    files_to_deploy = [f for f in template_dir.iterdir() if f.is_file()]
    
    with open(target_json, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    search_paths = config.get("search_paths", [])
    if not search_paths:
        print("[WARNING] search_paths 列表为空，没有需要扫描的目录。")
        return True

    # 解析宏与通配符，得到绝对路径的搜索起点集
    print("[INFO] 正在解析搜索路径规则...")
    active_roots = resolve_search_roots(search_paths, sorted_macros)
    
    if not active_roots:
        print("[WARNING] 未能解析到任何真实存在的物理目录，请检查路径配置或宏变量！")
        return True

    success_count = 0
    print("[INFO] 开始深度扫描并分发...")
    
    target_dirs = set()

    for root_path in active_roots:
        # 如果用户直接通过 glob 匹配到了 gmp_src_mgr 文件夹本身
        if root_path.name == "gmp_src_mgr":
            target_dirs.add(root_path)
            continue
            
        # 深度遍历寻找名为 gmp_src_mgr 的目标文件夹
        for dirpath, dirnames, filenames in os.walk(root_path):
            current_dir = Path(dirpath)
            if current_dir.name == "gmp_src_mgr":
                target_dirs.add(current_dir)

    for current_dir in target_dirs:
        print(f"  [DEPLOY] 找到目标工程: {current_dir.parent.name}")
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