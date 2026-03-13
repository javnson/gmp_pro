import os
import json
import shutil
from pathlib import Path

def run_forward_sync():
    """执行正向同步：读取配置，清理旧文件，增量复制 .c 文件，生成 .inl 头文件"""
    
    print(">>> 开始执行 GMP 外设代码正向同步...")

    # 1. 路径准备
    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        return False, "未找到环境变量 GMP_PRO_LOCATION"
        
    global_dic_path = Path(gmp_location) / "tools" / "facilities_generator" / "peripheral_mgr" / "gmp_peripheral_dic.json"
    cwd = Path(os.getcwd())
    local_config_path = cwd / "gmp_perpheral_config.json"
    src_dest_dir = cwd / "gmp_peripheral_src"
    inl_dest_path = cwd / "include_headers.inl"

    # 2. 加载配置数据
    if not global_dic_path.exists() or not local_config_path.exists():
        return False, "找不到全局字典或本地配置文件，请先完成配置！"

    try:
        with open(global_dic_path, 'r', encoding='utf-8') as f:
            global_registry = json.load(f)
        with open(local_config_path, 'r', encoding='utf-8') as f:
            local_config = json.load(f)
    except Exception as e:
        return False, f"读取 JSON 失败: {e}"

    # 3. 梳理需要同步的 .c 和 .h 文件列表
    required_c_files = set() # 存储源文件的绝对路径
    required_h_files = set() # 存储相对路径，用于生成 inl
    
    # 记录目标目录中预期存在的 .c 文件名 (用于后续清理)
    expected_c_filenames = set()

    for item in local_config.get("selected_modules", []):
        cat = item["category"]
        mod = item["module"]
        try:
            mod_data = global_registry["tree_nodes"][cat][mod]
            
            # 收集 C 文件
            for c_rel_path in mod_data.get("c_files", []):
                required_c_files.add(Path(gmp_location) / c_rel_path)
                expected_c_filenames.add(Path(c_rel_path).name)
                
            # 收集 H 文件
            for h_rel_path in mod_data.get("h_files", []):
                required_h_files.add(h_rel_path)
                
        except KeyError:
            print(f"警告: 本地配置中的模块 {cat}|{mod} 在全局库中不存在，已跳过。")

    # 4. 目标文件夹处理与清理 (Pruning)
    src_dest_dir.mkdir(parents=True, exist_ok=True)
    
    deleted_count = 0
    for existing_file in src_dest_dir.iterdir():
        if existing_file.is_file() and existing_file.suffix == '.c':
            if existing_file.name not in expected_c_filenames:
                print(f"  [-] 清理废弃文件: {existing_file.name}")
                existing_file.unlink()
                deleted_count += 1

    # 5. 增量复制 .c 文件 (对比修改时间)
    copied_count = 0
    skipped_count = 0
    
    for src_c_path in required_c_files:
        if not src_c_path.exists():
            print(f"  [!] 警告: 源文件丢失 {src_c_path}")
            continue
            
        dest_c_path = src_dest_dir / src_c_path.name
        
        # 判断是否需要拷贝: 目标不存在，或者源文件更新时间晚于目标文件
        need_copy = True
        if dest_c_path.exists():
            src_mtime = src_c_path.stat().st_mtime
            dest_mtime = dest_c_path.stat().st_mtime
            if src_mtime <= dest_mtime:
                need_copy = False
                
        if need_copy:
            print(f"  [+] 复制/更新文件: {src_c_path.name}")
            shutil.copy2(src_c_path, dest_c_path) # copy2 会保留原文件的时间戳等元数据
            copied_count += 1
        else:
            skipped_count += 1

    # 6. 生成 include_headers.inl
    try:
        with open(inl_dest_path, 'w', encoding='utf-8') as f:
            f.write("/* \n")
            f.write(" * GMP Auto-Generated Peripheral Include Headers\n")
            f.write(" * 警告: 本文件由工具自动生成，请勿手动修改！\n")
            f.write(" */\n\n")
            f.write("#ifndef GMP_PERIPHERAL_HEADERS_INL\n")
            f.write("#define GMP_PERIPHERAL_HEADERS_INL\n\n")
            
            for h_file in sorted(list(required_h_files)):
                # 按照你的要求，包含相对于 GMP_PRO_LOCATION 的路径
                f.write(f'#include "{h_file}"\n')
                
            f.write("\n#endif // GMP_PERIPHERAL_HEADERS_INL\n")
            print(f"  [+] 生成头文件汇总: {inl_dest_path.name}")
    except Exception as e:
        return False, f"生成 .inl 文件失败: {e}"

    summary = f"同步完成！\n\n新增/更新: {copied_count} 个文件\n跳过(未修改): {skipped_count} 个文件\n清理(已废弃): {deleted_count} 个文件\n\n头文件已汇总至 include_headers.inl"
    print(f">>> {summary}")
    return True, summary

if __name__ == "__main__":
    # 仅供独立测试运行
    success, msg = run_forward_sync()
    print(msg)