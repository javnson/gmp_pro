import os
import sys
import json
import subprocess
import glob
from pathlib import Path

def get_macros(dic_path, gmp_location):
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

def resolve_search_roots(search_paths, sorted_macros):
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
                if p.is_dir(): resolved_roots.add(p.resolve())
        else:
            p = Path(res_pat)
            if p.is_dir(): resolved_roots.add(p.resolve())
    return list(resolved_roots)

def run_batch_generation():
    print("=" * 60)
    print("⚡ [GMP Fleet] 启动批量代码生成引擎 (Macro & Glob Mode)...")
    print("=" * 60)

    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("[ERROR] 未找到环境变量 GMP_PRO_LOCATION！")
        return False

    base_dir = Path(__file__).parent.resolve()
    target_json = base_dir / "deploy_targets.json"
    dic_path = base_dir / "gmp_framework_dic.json"

    if not target_json.exists():
        print(f"[ERROR] 找不到目标配置文件: {target_json.name}")
        return False

    sorted_macros = get_macros(dic_path, gmp_location)

    with open(target_json, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    search_paths = config.get("search_paths", [])
    
    print("[INFO] 正在解析搜索路径规则...")
    active_roots = resolve_search_roots(search_paths, sorted_macros)

    projects_found = set()

    print("[INFO] 正在深度扫描工程目标...")
    for root_path in active_roots:
        if root_path.name == "gmp_src_mgr":
            projects_found.add(root_path)
            continue
            
        for dirpath, dirnames, filenames in os.walk(root_path):
            current_dir = Path(dirpath)
            if current_dir.name == "gmp_src_mgr":
                projects_found.add(current_dir)

    if not projects_found:
        print("[WARNING] 未找到任何符合条件的 gmp_src_mgr 文件夹。")
        return True

    print(f"[INFO] 发现 {len(projects_found)} 个目标工程，开始执行构建列阵...")
    print("-" * 60)

    stats = {"success": 0, "failed": 0}

    for proj_dir in sorted(list(projects_found)):
        print(f"\n>>> [BUILDING] 工程路径: {proj_dir.parent.name} ({proj_dir})")
        
        script_src = proj_dir / "gmp_generate_src.bat"
        script_inc = proj_dir / "gmp_generate_inc.bat"
        
        has_error = False
        creationflags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0

        if script_src.exists():
            print("    -> 正在执行源文件生成 (Source)...")
            res_src = subprocess.run([str(script_src)], cwd=proj_dir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, creationflags=creationflags, encoding='utf-8', errors='replace')
            if res_src.returncode != 0:
                print(f"    ❌ [ERROR] 源文件生成失败 (Error Code: {res_src.returncode})")
                has_error = True
            else:
                print("    ✅ [OK] 源文件生成成功。")
        else:
            print("    ⚠️ [SKIP] 找不到源文件生成脚本。")

        if script_inc.exists() and not has_error:
            print("    -> 正在执行头文件镜像 (Header)...")
            res_inc = subprocess.run([str(script_inc)], cwd=proj_dir, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, creationflags=creationflags, encoding='utf-8', errors='replace')
            if res_inc.returncode != 0:
                print(f"    ❌ [ERROR] 头文件生成失败 (Error Code: {res_inc.returncode})")
                has_error = True
            else:
                print("    ✅ [OK] 头文件镜像成功。")
        elif not script_inc.exists():
            print("    ⚠️ [SKIP] 找不到头文件生成脚本。")

        if has_error:
            stats["failed"] += 1
            print("    🛑 该工程构建异常中止。")
        else:
            stats["success"] += 1
            print("    🎉 该工程全部生成任务完成。")

    print("\n" + "=" * 60)
    print("📊 [FLEET SUMMARY] 舰队构建任务执行完毕！")
    print(f"    🟢 成功更新工程: {stats['success']} 个")
    print(f"    🔴 发生错误工程: {stats['failed']} 个")
    print("=" * 60)
    return True

if __name__ == "__main__":
    if not run_batch_generation():
        sys.exit(1)