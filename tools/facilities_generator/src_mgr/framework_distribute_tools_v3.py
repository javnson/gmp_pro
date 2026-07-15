import os
import sys
import json
import shutil
import subprocess
import re
from pathlib import Path

from framework_project_discovery import exclude_git_ignored

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

def find_target_projects(
    search_paths, sorted_macros, repository_root, target_dir_name="gmp_src_mgr"
):
    """
    Super Path Resolution Engine V2: 
    支持正则动态解析任何系统环境变量，并使用 pathlib 极速递归。
    """
    projects_found = set()
    env_pattern = re.compile(r'\$\{([^}]+)\}')
    
    def env_replacer(match):
        var_name = match.group(1)
        val = os.environ.get(var_name)
        return val if val is not None else match.group(0)

    for pat in search_paths:
        # 1. 动态获取并展开真正的系统环境变量
        res_pat = env_pattern.sub(env_replacer, pat)
        
        # 2. 替换 json 文件中读取出的自定义宏
        for mac, val in sorted_macros:
            res_pat = res_pat.replace(f"${{{mac}}}", val)
            
        # 3. 统一斜杠，避免平台差异
        res_pat = res_pat.replace('\\', '/')
        
        # 4. 智能解析通配符
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
            print(f"  [SKIP] 路径无效或不存在: {base_path}")
            continue

        # 5. 使用健壮的 rglob() 递归搜索目标文件夹
        if is_recursive:
            for match in base_path.rglob(target_dir_name):
                if match.is_dir():
                    projects_found.add(match.resolve())
        else:
            target = base_path / target_dir_name
            if target.is_dir():
                projects_found.add(target.resolve())
                
    visible, ignored = exclude_git_ignored(projects_found, repository_root)
    for path in ignored:
        print(f"  [IGNORE] Git-ignored source-manager copy: {path}")
    return visible


def execute_generation_script(script_path):
    """Run one distributed BAT without hiding its output or exit code."""
    if os.name != "nt":
        print(f"    -> [ERROR] BAT generation is only supported on Windows: {script_path.name}")
        return False
    command = [os.environ.get("COMSPEC", "cmd.exe"), "/d", "/c", "call", str(script_path)]
    result = subprocess.run(command, cwd=script_path.parent)
    if result.returncode:
        print(f"    -> [ERROR] {script_path.name} failed with exit code {result.returncode}")
        return False
    print(f"    -> [OK] {script_path.name}")
    return True

def run_distribution():
    print("=" * 60)
    print("[START] [GMP Fleet] Starting Distribution Engine (Dynamic Env Mode)...")
    print("=" * 60)

    root_value = os.environ.get('GMP_PRO_LOCATION')
    if not root_value:
        print("[ERROR] Environment variable GMP_PRO_LOCATION not found!")
        return False
    repository_root = Path(root_value).resolve()

    base_dir = Path(__file__).parent.resolve()
    target_json = base_dir / "deploy_targets.json"
    template_dir = base_dir / "gmp_src_mgr"
    dic_path = base_dir / "gmp_framework_dic.json"

    if not target_json.exists():
        print(f"[ERROR] Target config not found: {target_json.name}")
        return False

    if not template_dir.exists() or not any(template_dir.iterdir()):
        print(f"[ERROR] Template directory not found or empty: {template_dir.name}")
        print("Please place the .bat scripts to distribute in this directory!")
        return False

    # ================= Core Defense Mechanism: Distribution Blacklist =================
    EXCLUDE_FILES = {
        "gmp_framework_config.json", 
        "gmp_compiler_includes.txt", 
        "deploy_targets.json",
        ".gmpignore"
    }

    files_to_deploy = [
        f for f in template_dir.iterdir() 
        if f.is_file() and f.name not in EXCLUDE_FILES
    ]

    if not files_to_deploy:
        print("[WARNING] No valid files to distribute in the template folder! (Blacklisted files filtered)")
        return False
        
    sorted_macros = get_macros(dic_path)

    with open(target_json, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    search_paths = config.get("search_paths", [])
    if not search_paths:
        print("[WARNING] 'search_paths' list is empty. No directories to scan.")
        return True

    print("[INFO] Parsing search rules and deep scanning for targets...\n")
    
    # 获取所有目标项目路径
    try:
        target_dirs = find_target_projects(search_paths, sorted_macros, repository_root)
    except RuntimeError as error:
        print(f"[ERROR] Project discovery failed: {error}")
        return False

    if not target_dirs:
        print("\n[WARNING] Scan completed, no target projects found.")
        return True

    # 打印发现的目标，保持原脚本的 Phase 1 输出风格
    for target in sorted(target_dirs):
        print(f"  [FOUND] Target discovered: {target.parent.name} ({target})")

    # Phase 2: Execute safe physical overwrite
    print("\n" + "-" * 60)
    print("[INFO] Scan completed. Distributing/overwriting toolchain to target projects...\n")
    
    deployed_targets = []
    deployment_failures = 0
    for current_dir in sorted(target_dirs):
        print(f"  [DEPLOY] Deploying to project: {current_dir.parent.name}")
        try:
            for src_file in files_to_deploy:
                dest_file = current_dir / src_file.name
                shutil.copy2(src_file, dest_file)
                print(f"    -> [OVERWRITE] {src_file.name}")
            deployed_targets.append(current_dir)
        except Exception as e:
            print(f"    -> [ERROR] Overwrite failed: {e}")
            deployment_failures += 1

    # Phase 3: Generate headers and sources only after the tool scripts have
    # been refreshed. Header generation runs first by installer contract.
    print("\n" + "-" * 60)
    print("[INFO] Tool distribution completed. Generating project files...\n")

    generation_successes = 0
    generation_failures = 0
    for current_dir in deployed_targets:
        print(f"  [GENERATE] Project: {current_dir.parent.name} ({current_dir})")
        project_ok = True
        for script_name in ("gmp_generate_inc.bat", "gmp_generate_src.bat"):
            script_path = current_dir / script_name
            if not script_path.is_file():
                print(f"    -> [ERROR] Required generation script is missing: {script_name}")
                project_ok = False
                continue
            if not execute_generation_script(script_path):
                project_ok = False
        if project_ok:
            generation_successes += 1
        else:
            generation_failures += 1

    print("\n" + "=" * 60)
    print(f"[SUMMARY] Tool scripts deployed: {len(deployed_targets)} project(s).")
    print(f"[SUMMARY] Generation succeeded: {generation_successes} project(s).")
    print(f"[SUMMARY] Deployment failures: {deployment_failures} project(s).")
    print(f"[SUMMARY] Generation failures: {generation_failures} project(s).")
    print("=" * 60)
    return deployment_failures == 0 and generation_failures == 0

if __name__ == "__main__":
    if not run_distribution():
        sys.exit(1)
