import json
import copy
from pathlib import Path

class FrameworkDataModel:
    def __init__(self, gmp_location, json_path):
        self.gmp_location = gmp_location
        self.json_path = Path(json_path)
        self.registry = {"roots": ["core", "ctl", "cctl", "csp", "vctl"], "modules": {}, "macros": {}}

    def get_default_tasks(self):
        """Define the 3 mandatory system-level verification tasks"""
        return [
            {"id": "sys_compile", "summary": "Compilation Passed", "completed": False, "date": "", "reply": ""},
            {"id": "sys_sim", "summary": "Simulation / Principle Verified", "completed": False, "date": "", "reply": ""},
            {"id": "sys_hw", "summary": "Physical Hardware Verified", "completed": False, "date": "", "reply": ""}
        ]

    def inject_default_tasks(self, mod_key, mod_data):
        """Ensure tasks exist and contain mandatory system tasks, but ignore _internal"""
        # [核心改动]：识别并屏蔽 _internal 的任务注入，并清理历史遗留的无效任务
        if mod_key.endswith("_internal"):
            if "tasks" in mod_data:
                del mod_data["tasks"] 
            return

        if "tasks" not in mod_data:
            mod_data["tasks"] = copy.deepcopy(self.get_default_tasks())
        else:
            existing_ids = [t.get("id") for t in mod_data["tasks"]]
            for dt in reversed(self.get_default_tasks()): 
                if dt["id"] not in existing_ids:
                    mod_data["tasks"].insert(0, copy.deepcopy(dt))

    def get_module_status(self, mod_key, mod_data):
        """
        Calculate module maturity level.
        Returns: (Icon_String, Status_Description)
        """
        # [核心改动]：接收 mod_key，遇到内部包直接给予最高绿灯通行
        if mod_key.endswith("_internal"):
            return "🟢", "Implicitly Verified (Internal package)"

        tasks = mod_data.get("tasks", [])
        task_map = {t.get("id"): t.get("completed", False) for t in tasks}

        if task_map.get("sys_hw", False):
            return "🟢", "Ready for Production (HW Verified)"
        elif task_map.get("sys_sim", False):
            return "🔷", "Sim/Principle Verified"  
        elif task_map.get("sys_compile", False):
            return "🟨", "Compilation Passed"
        else:
            return "❌", "In Development (Unverified)"

    def load(self):
        if not self.json_path.exists():
            self.registry.clear()
            self.registry.update({"roots": ["core", "ctl", "cctl", "csp", "vctl"], "modules": {}, "macros": {}})
        else:
            with open(self.json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.registry.clear()
                self.registry.update(data)
                
        if "macros" not in self.registry:
            self.registry["macros"] = {}
        self.registry["macros"]["GMP_PRO_LOCATION"] = Path(self.gmp_location).as_posix()
            
        for root_name, modules in self.registry.get("modules", {}).items():
            for mod_key, mod_data in modules.items():
                mod_data.setdefault("description", "")
                mod_data.setdefault("src_patterns", [])
                mod_data.setdefault("inc_patterns", [])
                mod_data.setdefault("inc_dirs", [])
                mod_data.setdefault("help_docs", [])
                mod_data.setdefault("depends_on", [])
                # 传入 mod_key
                self.inject_default_tasks(mod_key, mod_data)

        self.ensure_internals()

    def save(self):
        self.json_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.json_path, 'w', encoding='utf-8') as f:
            json.dump(self.registry, f, indent=4, ensure_ascii=False)

    def ensure_internals(self):
        changed = False
        for root_name, modules in self.registry.get("modules", {}).items():
            folders = set()
            for mod_key in modules.keys():
                parts = mod_key.split('|')
                for i in range(1, len(parts)):
                    folders.add('|'.join(parts[:i]))
                    
            for folder in folders:
                internal_key = f"{folder}|_internal"
                if internal_key not in modules:
                    modules[internal_key] = {
                        "type": "module", "description": "Auto-generated internal config",
                        "src_patterns": [], "inc_patterns": [], "inc_dirs": [], 
                        "help_docs": [], "depends_on": []
                    }
                    changed = True
        return changed

    def check_and_rollback_parent(self, root_name, deleted_path):
        parts = deleted_path.split('|')
        if len(parts) > 1:
            parent_folder = '|'.join(parts[:-1])
            remaining = [k for k in self.registry["modules"].get(root_name, {}).keys() if k.startswith(parent_folder + '|')]
            if len(remaining) == 1 and remaining[0] == f"{parent_folder}|_internal":
                internal_data = self.registry["modules"][root_name].pop(remaining[0])
                self.registry["modules"][root_name][parent_folder] = internal_data

    def resolve_paths(self, patterns, is_dir_mode=False, return_absolute=False):
        matched = set()
        base_dir = Path(self.gmp_location)
        
        for pat in patterns:
            resolved_pat = pat
            for mac, val in self.registry.get("macros", {}).items():
                resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)
            
            pat_obj = Path(resolved_pat)
            
            if pat_obj.is_absolute():
                search_base = Path(pat_obj.anchor)
                search_pattern = str(pat_obj.relative_to(pat_obj.anchor))
            else:
                search_base = base_dir
                search_pattern = str(pat_obj)
                
            if is_dir_mode:
                target = search_base / search_pattern
                if target.exists() and target.is_dir():
                    matched.add(target.resolve().as_posix() if return_absolute else target.as_posix())
            else:
                for f in search_base.glob(search_pattern):
                    if f.is_file():
                        matched.add(f.resolve().as_posix() if return_absolute else f.as_posix())
                        
        return sorted(list(matched))