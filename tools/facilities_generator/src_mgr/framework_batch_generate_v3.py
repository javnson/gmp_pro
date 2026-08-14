import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

from framework_project_discovery import exclude_git_ignored
from framework_registry import RegistrySelectionError, resolve_selected_modules

def get_macros(dic_path):
    """Load custom macros without mixing in process environment variables."""
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
    projects_found = set()
    
    # Match environment-variable references in the ${VAR_NAME} form.
    env_pattern = re.compile(r'\$\{([^}]+)\}')
    
    def env_replacer(match):
        var_name = match.group(1)
        val = os.environ.get(var_name)
        return val if val is not None else match.group(0)

    for pat in search_paths:
        # Expand process environment variables first.
        res_pat = env_pattern.sub(env_replacer, pat)
        
        # Expand registry macros second.
        for mac, val in sorted_macros:
            res_pat = res_pat.replace(f"${{{mac}}}", val)
            
        # Normalize separators before interpreting the search suffix.
        res_pat = res_pat.replace('\\', '/')
        
        # Interpret the supported recursive and direct-child suffixes.
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
            print(f"[DEBUG] Search root does not exist; skipping: {base_path}")
            continue

        print(f"[INFO] Scanning: {base_path} (recursive: {is_recursive})")

        # Locate project source-manager directories.
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
        print(f"[IGNORE] Git-ignored source-manager copy: {path}")
    return visible

def generate_project(proj_dir, dry_run=False, runner=subprocess.run):
    """Validate or generate one project in the canonical header/source order."""
    script_inc = proj_dir / "gmp_generate_inc.bat"
    script_src = proj_dir / "gmp_generate_src.bat"
    project_config = proj_dir / "gmp_framework_config.json"
    required = (project_config, script_inc, script_src)
    missing = [path.name for path in required if not path.is_file()]
    if missing:
        print(f"    [ERROR] Missing required project file(s): {', '.join(missing)}")
        return False

    registry_path = Path(__file__).with_name("gmp_framework_dic.json")
    try:
        with registry_path.open("r", encoding="utf-8") as stream:
            registry = json.load(stream)
        with project_config.open("r", encoding="utf-8") as stream:
            local_config = json.load(stream)
        resolve_selected_modules(registry, local_config)
    except (OSError, json.JSONDecodeError, RegistrySelectionError) as error:
        print(f"    [ERROR] Invalid Facility selection: {error}")
        return False

    if dry_run:
        print("    [VALID] Entry points and the complete Facility dependency graph are valid.")
        return True

    creationflags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
    subprocess_encoding = 'mbcs' if os.name == 'nt' else 'utf-8'
    for label, script in (("header", script_inc), ("source", script_src)):
        print(f"    -> Executing {label} generation: {script.name}")
        result = runner(
            [str(script)],
            cwd=proj_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            creationflags=creationflags,
            encoding=subprocess_encoding,
            errors='replace',
        )
        if result.returncode != 0:
            print(f"    [ERROR] {label.capitalize()} generation failed (error code {result.returncode}).")
            if result.stdout:
                print(result.stdout.rstrip())
            return False
        print(f"    [OK] {label.capitalize()} generation successful.")
    return True


def run_batch_generation(dry_run=False):
    print("=" * 60)
    print("[START] [GMP Fleet] Starting Batch Generation Engine (Dynamic Env Mode)...")
    print("=" * 60)

    root_value = os.environ.get('GMP_PRO_LOCATION')
    if not root_value:
        print("[ERROR] Environment variable GMP_PRO_LOCATION not found!")
        return False
    repository_root = Path(root_value).resolve()

    base_dir = Path(__file__).parent.resolve()
    target_json = base_dir / "deploy_targets.json"
    dic_path = base_dir / "gmp_framework_dic.json"

    if not target_json.exists():
        print(f"[ERROR] Target config not found: {target_json.name}")
        return False

    # Load registry macros without overriding the process environment.
    sorted_macros = get_macros(dic_path)

    with open(target_json, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    search_paths = config.get("search_paths", [])
    
    print("[INFO] Parsing search rules and scanning for projects...")
    try:
        projects_found_list = find_target_projects(
            search_paths, sorted_macros, repository_root
        )
    except RuntimeError as error:
        print(f"[ERROR] Project discovery failed: {error}")
        return False

    if not projects_found_list:
        print("[WARNING] No matching 'gmp_src_mgr' folders found.")
        return True

    action = "validation" if dry_run else "generation"
    print(f"[INFO] Found {len(projects_found_list)} target projects. Initiating batch {action}...")
    print("-" * 60)

    stats = {"success": 0, "failed": 0}

    for proj_dir in sorted(projects_found_list):
        print(f"\n>>> [PROJECT] {proj_dir.parent.name} ({proj_dir})")
        if not generate_project(proj_dir, dry_run=dry_run):
            stats["failed"] += 1
            print("    [FAILED] Project generation contract is not satisfied.")
        else:
            stats["success"] += 1
            print("    [SUCCESS] Project generation contract passed.")

    print("\n" + "=" * 60)
    print("[FLEET SUMMARY] Fleet batch build completed!")
    print(f"    [SUCCESS] Projects successfully updated: {stats['success']}")
    print(f"    [FAILED] Projects with errors: {stats['failed']}")
    print("=" * 60)
    return stats["failed"] == 0


def parse_args(argv=None):
    """Parse the batch generator command line without side effects."""
    parser = argparse.ArgumentParser(
        description="Validate or regenerate every discovered GMP source-manager project."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="discover projects and validate required entry points without generating files",
    )
    return parser.parse_args(argv)

if __name__ == "__main__":
    arguments = parse_args()
    if not run_batch_generation(dry_run=arguments.dry_run):
        sys.exit(1)
