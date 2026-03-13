import os
import sys
import json
import shutil
from pathlib import Path

def run_forward_sync():
    """执行正向同步：读取配置，清理旧文件，增量复制 .c 文件，生成 .inl 头文件"""
    
    print(">>> [GMP Sync] 开始执行外设代码正向同步...")

    # 1. 路径准备
    gmp_location = os.environ.get('GMP_PRO_LOCATION')
    if not gmp_location:
        return False, "未找到环境变量 GMP_PRO_LOCATION，请检查系统配置！"
        
    global_dic_path = Path(gmp_location) / "tools" / "facilities_generator" / "peripheral_mgr" / "gmp_peripheral_dic.json"
    cwd = Path(os.getcwd())
    local_config_path = cwd / "gmp_perpheral_config.json"
    src_dest_dir = cwd / "gmp_peripheral_src"
    inl_dest_path = cwd / "include_headers.inl"

    # 2. 加载配置数据
    if not global_dic_path.exists():
        return False, f"找不到全局字典: {global_dic_path}"
        
    if not local_config_path.exists():
        # 对于 CLI 来说，如果当前工程没有配置文件，说明用户不想用外设库，直接返回成功即可，不要报错打断编译
        print(">>> [GMP Sync] 未检测到本地 gmp_perpheral_config.json，跳过同步。")
        return True, "跳过同步"

    try:
        with open(global_dic_path, 'r', encoding='utf-8') as f:
            global_registry = json.load(f)
        with open(local_config_path, 'r', encoding='utf-8') as f:
            local_config = json.load(f)
    except Exception as e:
        return False, f"读取 JSON 失败: {e}"

    # 3. 梳理需要同步的 .c 和 .h 文件列表
    required_c_files = set()
    required_h_files = set()
    expected_c_filenames = set()

    for item in local_config.get("selected_modules", []):
        cat = item["category"]
        mod = item["module"]
        try:
            mod_data = global_registry["tree_nodes"][cat][mod]
            for c_rel_path in mod_data.get("c_files", []):
                required_c_files.add(Path(gmp_location) / c_rel_path)
                expected_c_filenames.add(Path(c_rel_path).name)
            for h_rel_path in mod_data.get("h_files", []):
                required_h_files.add(h_rel_path)
        except KeyError:
            print(f"  [!] 警告: 本地配置中的模块 {cat}|{mod} 在全局库中不存在，已跳过。")

    # 4. 目标文件夹处理与清理 (Pruning)
    src_dest_dir.mkdir(parents=True, exist_ok=True)
    
    deleted_count = 0
    for existing_file in src_dest_dir.iterdir():
        if existing_file.is_file() and existing_file.suffix == '.c':
            if existing_file.name not in expected_c_filenames:
                print(f"  [-] 清理废弃文件: {existing_file.name}")
                existing_file.unlink()
                deleted_count += 1

    # 5. 增量复制 .c 文件
    copied_count = 0
    skipped_count = 0
    
    for src_c_path in required_c_files:
        if not src_c_path.exists():
            print(f"  [!] 错误: 源文件丢失 {src_c_path}")
            return False, f"源文件丢失: {src_c_path.name}"
            
        dest_c_path = src_dest_dir / src_c_path.name
        
        need_copy = True
        if dest_c_path.exists():
            if src_c_path.stat().st_mtime <= dest_c_path.stat().st_mtime:
                need_copy = False
                
        if need_copy:
            print(f"  [+] 复制/更新: {src_c_path.name}")
            shutil.copy2(src_c_path, dest_c_path)
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
                f.write(f'#include "{h_file}"\n')
                
            f.write("\n#endif // GMP_PERIPHERAL_HEADERS_INL\n")
    except Exception as e:
        return False, f"生成 .inl 文件失败: {e}"

    summary = f">>> [GMP Sync] 成功! 更新:{copied_count} 跳过:{skipped_count} 清理:{deleted_count}"
    return True, summary

# ================= CLI 启动入口 =================
if __name__ == "__main__":
    success, msg = run_forward_sync()
    if success:
        print(msg)
        sys.exit(0)  # 返回 0 给编译器，表示预编译步骤成功
    else:
        # 将错误信息输出到标准错误流 (stderr)，在编译器中通常会显示为红色报错
        print(f">>> [GMP Sync ERROR] 严重错误: {msg}", file=sys.stderr)
        sys.exit(1)  # 返回非 0 给编译器，强制打断编译过程