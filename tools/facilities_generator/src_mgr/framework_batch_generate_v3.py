import os
import sys
import json
import subprocess
import re
from pathlib import Path

def get_macros(dic_path):
    """
    只负责从 json 配置文件中读取自定义宏，不再混入系统环境变量。
    """
    macros = {}
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

def find_target_projects(search_paths, sorted_macros, target_dir_name="gmp_src_mgr"):
    projects_found = set()
    
    # 预编译正则表达式，用于动态匹配 ${VAR_NAME} 格式的环境变量
    env_pattern = re.compile(r'\$\{([^}]+)\}')
    
    def env_replacer(match):
        var_name = match.group(1)
        # 实时从系统中获取环境变量的值
        val = os.environ.get(var_name)
        # 如果系统中存在该环境变量则替换；如果不存在，则保持原样（可能留给自定义宏替换）
        return val if val is not None else match.group(0)

    for pat in search_paths:
        # 1. 动态获取并展开真正的环境变量 (例如 ${GMP_PRO_LOCATION} 将在这里被替换为实际值)
        res_pat = env_pattern.sub(env_replacer, pat)
        
        # 2. 接着替换 json 文件中读取出的自定义宏
        for mac, val in sorted_macros:
            res_pat = res_pat.replace(f"${{{mac}}}", val)
            
        # 3. 统一斜杠，避免 Windows/Linux 混用导致路径解析错误
        res_pat = res_pat.replace('\\', '/')
        
        # 4. 智能解析通配符 (剔除 /**，转换为 Path 对象)
        is_recursive = False
        if res_pat.endswith('/**'):
            is_recursive = True
            base_dir_str = res_pat[:-3]
        elif res_pat.endswith('/*'):
            base_dir_str = res_pat[:-2]
        else:
            base_dir_str = res_pat

        base_path = Path(base_dir_str).resolve()

        if not base_path.exists() or not base_path.is_dir():
            print(f"[DEBUG] 路径无效或不存在，跳过: {base_path}")
            continue

        print(f"[INFO] 正在扫描基准目录: {base_path} (递归模式: {is_recursive})")

        # 5. 使用健壮的 rglob() 递归搜索名为 gmp_src_mgr 的目标文件夹
        if is_recursive:
            for match in base_path.rglob(target_dir_name):
                if match.is_dir():
                    projects_found.add(match.resolve())
        else:
            target = base_path / target_dir_name
            if target.is_dir():
                projects_found.add(target.resolve())
                
    return list(projects_found)

def run_batch_generation():
    print("=" * 60)
    print("[START] [GMP Fleet] Starting Batch Generation Engine (Dynamic Env Mode)...")
    print("=" * 60)

    # 这里的检查仅用作预警提示，如果用户没有配置该变量，及时提醒
    if not os.environ.get('GMP_PRO_LOCATION'):
        print("[WARNING] Environment variable GMP_PRO_LOCATION not found! Paths depending on it may fail.")

    base_dir = Path(__file__).parent.resolve()
    target_json = base_dir / "deploy_targets.json"
    dic_path = base_dir / "gmp_framework_dic.json"

    if not target_json.exists():
        print(f"[ERROR] Target config not found: {target_json.name}")
        return False

    # 读取单纯的自定义宏（不再强制传入 GMP_PRO_LOCATION）
    sorted_macros = get_macros(dic_path)

    with open(target_json, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    search_paths = config.get("search_paths", [])
    
    print("[INFO] Parsing search rules and scanning for projects...")
    projects_found_list = find_target_projects(search_paths, sorted_macros)

    if not projects_found_list:
        print("[WARNING] No matching 'gmp_src_mgr' folders found.")
        return True

    print(f"[INFO] Found {len(projects_found_list)} target projects. Initiating batch build...")
    print("-" * 60)

    stats = {"success": 0, "failed": 0}

    # 为了防止 Windows 执行 .bat 脚本时出现中文乱码，自动切换子进程编码
    subprocess_encoding = 'mbcs' if os.name == 'nt' else 'utf-8'

    for proj_dir in sorted(projects_found_list):
        print(f"\n>>> [BUILDING] Project: {proj_dir.parent.name} ({proj_dir})")
        
        script_src = proj_dir / "gmp_generate_src.bat"
        script_inc = proj_dir / "gmp_generate_inc.bat"
        
        has_error = False
        creationflags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0

        if script_src.exists():
            print("    -> Executing source file generation (Source)...")
            res_src = subprocess.run([str(script_src)], cwd=proj_dir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, creationflags=creationflags, encoding=subprocess_encoding, errors='replace')
            if res_src.returncode != 0:
                print(f"    [ERROR] Source file generation failed (Error Code: {res_src.returncode})")
                has_error = True
            else:
                print("    [OK] Source file generation successful.")
        else:
            print("    [SKIP] Source generation script not found.")

        if script_inc.exists() and not has_error:
            print("    -> Executing header file mirroring (Header)...")
            res_inc = subprocess.run([str(script_inc)], cwd=proj_dir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, creationflags=creationflags, encoding=subprocess_encoding, errors='replace')
            if res_inc.returncode != 0:
                print(f"    [ERROR] Header file mirroring failed (Error Code: {res_inc.returncode})")
                has_error = True
            else:
                print("    [OK] Header file mirroring successful.")
        elif not script_inc.exists():
            print("    [SKIP] Header generation script not found.")

        if has_error:
            stats["failed"] += 1
            print("    [ABORT] Project build aborted due to errors.")
        else:
            stats["success"] += 1
            print("    [SUCCESS] All generation tasks completed for this project.")

    print("\n" + "=" * 60)
    print("[FLEET SUMMARY] Fleet batch build completed!")
    print(f"    [SUCCESS] Projects successfully updated: {stats['success']}")
    print(f"    [FAILED] Projects with errors: {stats['failed']}")
    print("=" * 60)
    return True

if __name__ == "__main__":
    if not run_batch_generation():
        sys.exit(1)