import os
import json
from pathlib import Path

def generate_initial_dic():
    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        print("错误: 未找到环境变量 GMP_PRO_LOCATION。")
        return

    dev_dir = Path(gmp_location) / "core" / "dev"
    output_json = Path(gmp_location) / "tools" / "facilities_generator" / "peripheral_mgr" / "gmp_peripheral_dic.json"

    if not dev_dir.exists():
        print(f"错误: 找不到外设目录 {dev_dir}")
        return

    ignore_dirs = ['.git', 'build', 'common_includes']
    ignore_files = ['readme.md', 'prompt.h']

    registry = {"tree_nodes": {}}
    
    # 专门为 dev/ 根目录下的文件建立一个分类
    registry["tree_nodes"]["_Root_Files"] = {}
    root_module_buffer = {}

    print(f"开始扫描目录: {dev_dir} ...")

    # 遍历 dev 目录下的所有内容 (包括文件夹和文件)
    for item_path in dev_dir.iterdir():
        
        # --- 1. 处理子文件夹 (例如 sensor, display) ---
        if item_path.is_dir() and item_path.name not in ignore_dirs:
            category_name = item_path.name
            registry["tree_nodes"][category_name] = {}
            module_buffer = {}

            for file_path in item_path.iterdir():
                if file_path.is_file() and file_path.name not in ignore_files:
                    stem = file_path.stem
                    ext = file_path.suffix.lower()
                    if ext in ['.c', '.h']:
                        if stem not in module_buffer:
                            module_buffer[stem] = {'c': [], 'h': []}
                        rel_path = file_path.relative_to(Path(gmp_location)).as_posix()
                        if ext == '.c': module_buffer[stem]['c'].append(rel_path)
                        elif ext == '.h': module_buffer[stem]['h'].append(rel_path)

            for module_name, files in module_buffer.items():
                if files['c'] or files['h']:
                    registry["tree_nodes"][category_name][module_name] = {
                        "type": "module",
                        "c_files": files['c'],
                        "h_files": files['h'],
                        "tags": [category_name],
                        "description": f"Auto-scanned module: {module_name}"
                    }
                    
        # --- 2. 处理直接在 dev/ 根目录下的文件 ---
        elif item_path.is_file() and item_path.name not in ignore_files:
            stem = item_path.stem
            ext = item_path.suffix.lower()
            if ext in ['.c', '.h']:
                if stem not in root_module_buffer:
                    root_module_buffer[stem] = {'c': [], 'h': []}
                rel_path = item_path.relative_to(Path(gmp_location)).as_posix()
                if ext == '.c': root_module_buffer[stem]['c'].append(rel_path)
                elif ext == '.h': root_module_buffer[stem]['h'].append(rel_path)

    # 将根目录下的文件转化为标准 module 结构
    for module_name, files in root_module_buffer.items():
        if files['c'] or files['h']:
            registry["tree_nodes"]["_Root_Files"][module_name] = {
                "type": "module",
                "c_files": files['c'],
                "h_files": files['h'],
                "tags": ["Root_System"],
                "description": f"Root module: {module_name}"
            }
            
    # 如果根目录下碰巧没有 C/H 文件，就把这个空分类清理掉
    if not registry["tree_nodes"]["_Root_Files"]:
        del registry["tree_nodes"]["_Root_Files"]

    # 确保输出目录存在并保存
    output_json.parent.mkdir(parents=True, exist_ok=True)
    with open(output_json, 'w', encoding='utf-8') as f:
        json.dump(registry, f, indent=4, ensure_ascii=False)

    print(f"扫描完成！字典已保存至: {output_json}")

if __name__ == "__main__":
    generate_initial_dic()