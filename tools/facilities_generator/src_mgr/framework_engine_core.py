import os
import json
from pathlib import Path

class FrameworkEngine:
    def __init__(self, gmp_location_path, json_dict_path):
        self.gmp_location = Path(gmp_location_path)
        self.json_path = Path(json_dict_path)
        self.registry = {}
        
        # 加载字典
        if self.json_path.exists():
            with open(self.json_path, 'r', encoding='utf-8') as f:
                self.registry = json.load(f)
        else:
            print(f"[错误] 找不到字典文件: {self.json_path}")

    def resolve_dependencies(self, selected_modules: list) -> set:
        """
        根据用户选择的模块列表，递归解析出所有的底层依赖模块。
        使用广度优先搜索 (BFS)，自动去重，防止循环依赖。
        """
        resolved_set = set()
        queue = list(selected_modules) # 待解析队列

        print("\n--- 正在解析依赖链 ---")
        while queue:
            current_mod_id = queue.pop(0)
            
            # 如果已经解析过，直接跳过 (防循环依赖)
            if current_mod_id in resolved_set:
                continue
                
            resolved_set.add(current_mod_id)
            print(f"  [+] 锁定模块: {current_mod_id}")

            # 解析 cat 和 mod (例如: "ctl|component|interface" -> cat="ctl", mod="component|interface")
            if '|' not in current_mod_id:
                print(f"  [警告] 模块 ID 格式异常 (应为 cat|mod): {current_mod_id}")
                continue
                
            cat, mod = current_mod_id.split('|', 1)
            
            # 获取该模块的字典数据
            mod_data = self.registry.get("modules", {}).get(cat, {}).get(mod)
            if not mod_data:
                print(f"  [警告] 字典中未定义模块: {current_mod_id}")
                continue
                
            # 将它的依赖加入队列继续解析
            depends_on = mod_data.get("depends_on", [])
            for dep in depends_on:
                if dep not in resolved_set:
                    queue.append(dep)
                    print(f"      -> 发现底层依赖: {dep}")

        return resolved_set

    def extract_physical_files(self, resolved_modules: set):
        """
        根据解析好的所有模块，结合通配符，去物理硬盘上捞文件。
        """
        src_files_result = set()
        inc_files_result = set()

        print("\n--- 正在映射物理文件 (Glob 解析) ---")
        for mod_id in resolved_modules:
            cat, mod = mod_id.split('|', 1)
            mod_data = self.registry["modules"][cat][mod]
            
            # 物理路径映射 (例如: ctl|component|interface -> %GMP_PRO_LOCATION%/ctl/component/interface)
            # 把 JSON key 中的 '|' 替换成路径分隔符 '/'
            physical_mod_dir = self.gmp_location / cat / mod.replace('|', '/')
            
            # 解析源文件
            src_patterns = mod_data.get("src_patterns", [])
            src_files = self._apply_glob(physical_mod_dir, src_patterns)
            src_files_result.update(src_files)
            
            # 解析头文件
            inc_patterns = mod_data.get("inc_patterns", [])
            inc_files = self._apply_glob(physical_mod_dir, inc_patterns)
            inc_files_result.update(inc_files)

        return src_files_result, inc_files_result

    def _apply_glob(self, base_dir: Path, patterns: list) -> set:
        """内部工具: 对指定目录执行通配符搜索，兼容各种特例"""
        matched = set()
        
        # 特例 1: 如果整个模块文件夹都不存在，不要崩溃，打个警告就行
        if not base_dir.exists():
            print(f"  [警告] 物理目录缺失: {base_dir}")
            return matched

        for pattern in patterns:
            # pathlib 的 glob 支持 "**/*.h" (递归) 和 "src/*.c" (单层) 甚至 "file.h" (指定文件)
            # 特例 2: 如果 pattern 指向的文件夹 (如 src/) 不存在，glob 会静默返回空列表，完美规避异常！
            for file_path in base_dir.glob(pattern):
                if file_path.is_file():
                    # 统一转换为相对路径，方便后续拷贝和生成头文件
                    rel_path = file_path.relative_to(self.gmp_location).as_posix()
                    matched.add(rel_path)
                    
        return matched


# ================== 测试代码 ==================
if __name__ == "__main__":
    # 模拟环境设置 (请确保环境变量已设置，或手动填入一个测试路径)
    gmp_root = os.environ.get('GMP_PRO_LOCATION', r"D:\GMP_Test_Project_Root") 
    json_path = "gmp_framework_dic.json" # 确保你刚才的 json 和这个 py 文件同级
    
    engine = FrameworkEngine(gmp_root, json_path)
    
    # 场景模拟: 用户在界面上只勾选了一个最上层的控制接口模块
    # 注意: "ctl|component|interface" 依赖了 "math_block" 和 "framework"
    user_selections = ["ctl|component|interface"]
    
    # 1. 解析依赖
    all_required_modules = engine.resolve_dependencies(user_selections)
    print(f"\n[汇总] 最终需要参与工程的模块共 {len(all_required_modules)} 个: {all_required_modules}")
    
    # 2. 捞取物理文件
    # (如果你本地对应的 GMP_PRO_LOCATION 还没建这些文件夹，这里会安全地打出警告并返回空)
    final_src, final_inc = engine.extract_physical_files(all_required_modules)
    
    print(f"\n[汇总] 待拷贝源文件 ({len(final_src)}):")
    for f in sorted(list(final_src)): print(f"  - {f}")
        
    print(f"\n[汇总] 待拷贝头文件 ({len(final_inc)}):")
    for f in sorted(list(final_inc)): print(f"  - {f}")