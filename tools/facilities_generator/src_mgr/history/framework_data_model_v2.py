import json
from pathlib import Path

class FrameworkDataModel:
    def __init__(self, gmp_location, json_path):
        self.gmp_location = gmp_location
        self.json_path = Path(json_path)
        self.registry = {"roots": ["core", "ctl", "cctl", "csp", "vctl"], "modules": {}, "macros": {}}

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
            
        # 自动防呆与补全 (新增 help_docs 字段)
        for root_name, modules in self.registry.get("modules", {}).items():
            for mod_data in modules.values():
                mod_data.setdefault("description", "")
                mod_data.setdefault("src_patterns", [])
                mod_data.setdefault("inc_patterns", [])
                mod_data.setdefault("inc_dirs", [])
                mod_data.setdefault("help_docs", []) # 新增：帮助文档
                mod_data.setdefault("depends_on", [])

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
                        "type": "module", "description": "自动生成的包基础配置",
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
        """
        超强路径解析引擎：支持相对路径基准推导与绝对路径返回
        """
        matched = set()
        base_dir = Path(self.gmp_location)
        
        for pat in patterns:
            resolved_pat = pat
            for mac, val in self.registry.get("macros", {}).items():
                resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)
            
            pat_obj = Path(resolved_pat)
            
            # 判断宏替换后是否为绝对路径
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
