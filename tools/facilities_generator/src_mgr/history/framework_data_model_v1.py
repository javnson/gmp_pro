import json
from pathlib import Path

class FrameworkDataModel:
    def __init__(self, gmp_location, json_path):
        self.gmp_location = gmp_location
        self.json_path = Path(json_path)
        # 核心数据字典
        self.registry = {"roots": ["core", "ctl", "cctl", "csp", "vctl"], "modules": {}, "macros": {}}

    def load(self):
        """加载数据并注入防呆默认值，使用 update 保证内存引用不丢失"""
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
        # 强行注入基准宏
        self.registry["macros"]["GMP_PRO_LOCATION"] = Path(self.gmp_location).as_posix()
            
        for root_name, modules in self.registry.get("modules", {}).items():
            for mod_data in modules.values():
                mod_data.setdefault("description", "")
                mod_data.setdefault("src_patterns", [])
                mod_data.setdefault("inc_patterns", [])
                mod_data.setdefault("inc_dirs", [])
                mod_data.setdefault("depends_on", [])

        self.ensure_internals()

    def save(self):
        """将当前内存中的 registry 写入磁盘"""
        self.json_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.json_path, 'w', encoding='utf-8') as f:
            json.dump(self.registry, f, indent=4, ensure_ascii=False)

    def ensure_internals(self):
        """确保所有文件夹包都有 _internal 节点"""
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
                        "src_patterns": [], "inc_patterns": [], "inc_dirs": [], "depends_on": []
                    }
                    changed = True
        return changed

    def check_and_rollback_parent(self, root_name, deleted_path):
        """当子节点删空时，自动将 _internal 退回为独立模块"""
        parts = deleted_path.split('|')
        if len(parts) > 1:
            parent_folder = '|'.join(parts[:-1])
            remaining = [k for k in self.registry["modules"].get(root_name, {}).keys() if k.startswith(parent_folder + '|')]
            if len(remaining) == 1 and remaining[0] == f"{parent_folder}|_internal":
                internal_data = self.registry["modules"][root_name].pop(remaining[0])
                self.registry["modules"][root_name][parent_folder] = internal_data

    def resolve_paths(self, patterns, is_dir_mode=False):
        """将带宏的通配符解析为硬盘上的真实物理路径列表"""
        matched = set()
        for pat in patterns:
            resolved_pat = pat
            for mac, val in self.registry.get("macros", {}).items():
                resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)
            
            pat_obj = Path(resolved_pat)
            
            if is_dir_mode:
                if pat_obj.exists() and pat_obj.is_dir():
                    matched.add(pat_obj.as_posix())
            else:
                if pat_obj.is_absolute():
                    anchor = pat_obj.anchor
                    rest = str(pat_obj.relative_to(anchor))
                    for f in Path(anchor).glob(rest):
                        if f.is_file(): matched.add(f.as_posix())
        return sorted(list(matched))