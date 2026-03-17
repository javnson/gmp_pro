import os
import sys
import json
import subprocess
from pathlib import Path

def get_macros(dic_path, gmp_location):
    # ... (保持原样不变) ...
    macros = {"GMP_PRO_LOCATION": Path(gmp_location).as_posix()}
    if dic_path.exists():
        try:
            with open(dic_path, 'r', encoding='utf-8') as f:
                dic_data = json.load(f)
                dic_macros = dic_data.get("macros", {})
                for k, v in dic_macros.items():
                    macros[k] = v
        except Exception as e:
            pass
    return sorted(macros.items(), key=lambda item: len(item[0]), reverse=True)

def find_target_projects(search_paths, sorted_macros, target_dir_name="gmp_src_mgr"):
    projects_found = set()
    
    for pat in search_paths:
        res_pat = pat
        # 1. 替换宏
        for mac, val in sorted_macros:
            res_pat = res_pat.replace(f"${{{mac}}}", val)
            
        # 2. 统一斜杠，方便处理
        res_pat = res_pat.replace('\\', '/')
        
        # 3. 智能解析通配符
        is_recursive = False
        if res_pat.endswith('/**'):
            is_recursive = True
            base_dir_str = res_pat[:-3] # 截掉 '/**' 获取实际要搜索的基准目录
        elif res_pat.endswith('/*'):
            base_dir_str = res_pat[:-2]
        else:
            base_dir_str = res_pat

        base_path = Path(base_dir_str).resolve()

        if not base_path.exists() or not base_path.is_dir():
            print(f"[DEBUG] 路径无效或不存在，跳过: {base_path}")
            continue

        print(f"[INFO] 正在扫描基准目录: {base_path} (递归模式: {is_recursive})")

        # 4. 使用更健壮的 rglob 直接搜索目标文件夹
        if is_recursive:
            # rglob 会递归查找所有层级下名为 gmp_src_mgr 的文件夹
            for match in base_path.rglob(target_dir_name):
                if match.is_dir():
                    projects_found.add(match.resolve())
        else:
            # 非递归模式，只检查当前目录下是否有目标文件夹
            target = base_path / target_dir_name
            if target.is_dir():
                projects_found.add(target.resolve())
                
    return list(projects_found)

def run_batch_generation():
    print("=" * 60)
    print("[START] [GMP Fleet] Starting Batch Generation Engine (Pathlib Mode)...")
    print("=" * 60)

    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("[ERROR] Environment variable GMP_PRO_LOCATION not found!")
        return False

    base_dir = Path(__file__).parent.resolve()
    target_json = base_dir / "deploy_targets.json"
    dic_path = base_dir / "gmp_framework_dic.json"

    if not target_json.exists():
        print(f"[ERROR] Target config not found: {target_json.name}")
        return False

    sorted_macros = get_macros(dic_path, gmp_location)

    with open(target_json, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    search_paths = config.get("search_paths", [])
    
    print("[INFO] Parsing search rules and scanning for projects...")
    
    # 【改动核心】：用新函数一步到位拿到所有 gmp_src_mgr 的路径
    projects_found_list = find_target_projects(search_paths, sorted_macros)

    if not projects_found_list:
        print("[WARNING] No matching 'gmp_src_mgr' folders found.")
        return True

    print(f"[INFO] Found {len(projects_found_list)} target projects. Initiating batch build...")
    print("-" * 60)

    stats = {"success": 0, "failed": 0}

    # 为了防止 Windows 的 bat 脚本输出中文时 utf-8 解码报错
    # 推荐使用 'mbcs' (Windows ANSI 默认编码) 或者直接不强制设定编码
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