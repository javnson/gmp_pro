import os
from pathlib import Path

def run_orphan_check(model, ignore_dirs=('.git', 'build', 'suite', 'doc', 'docs', 'example', 'examples')):
    """
    执行游离资产(未映射文件)扫描
    只扫描配置的 Roots 目录内的 .c, .cpp, .h, .hpp, .s, .asm 文件
    """
    report_lines = []
    orphan_count = 0
    
    report_lines.append("="*50)
    report_lines.append(" 👻 GMP 框架游离文件扫描报告 (Orphan Check)")
    report_lines.append(" (说明: 物理存在但未被收录进 JSON 字典的源码文件)")
    report_lines.append("="*50 + "\n")

    gmp_base = Path(model.gmp_location)
    registry = model.registry
    
    # 1. 收集所有已映射的物理文件绝对路径 (作为一个集合，方便极速比对)
    mapped_files_abs = set()
    for root_name, modules in registry.get("modules", {}).items():
        for mod_data in modules.values():
            src_abs = model.resolve_paths(mod_data.get("src_patterns", []), return_absolute=True)
            inc_abs = model.resolve_paths(mod_data.get("inc_patterns", []), return_absolute=True)
            mapped_files_abs.update(src_abs)
            mapped_files_abs.update(inc_abs)
            
    # 统一路径格式 (全小写、正斜杠) 防止跨平台比对失效
    mapped_files_normalized = {Path(p).resolve().as_posix().lower() for p in mapped_files_abs}

    # 关注的源码后缀
    target_extensions = {'.c', '.cpp', '.h', '.hpp', '.inl', '.s', '.asm'}

    # 2. 遍历所有的根目录 (Roots)
    for root_name in registry.get("roots", []):
        root_path = gmp_base / root_name
        if not root_path.exists() or not root_path.is_dir():
            continue
            
        root_orphans = []
        
        for dirpath, dirnames, filenames in os.walk(root_path):
            # 过滤忽略的文件夹 (原地修改 dirnames 以阻止 os.walk 进入)
            dirnames[:] = [d for d in dirnames if d not in ignore_dirs and not d.startswith('.')]
            
            for f in filenames:
                file_path = Path(dirpath) / f
                if file_path.suffix.lower() in target_extensions:
                    # 将物理文件转为标准格式
                    norm_path = file_path.resolve().as_posix().lower()
                    
                    if norm_path not in mapped_files_normalized:
                        # 这是一个游离文件！为了好看，我们计算它的相对路径
                        rel_to_gmp = file_path.relative_to(gmp_base).as_posix()
                        root_orphans.append(rel_to_gmp)
                        orphan_count += 1
                        
        if root_orphans:
            report_lines.append(f"📁 在根目录 [{root_name}] 中发现游离文件:")
            for orphan in sorted(root_orphans):
                report_lines.append(f"    ❓ {orphan}")
            report_lines.append("")

    report_lines.append("="*50)
    if orphan_count == 0:
        report_lines.append("✅ 扫描完美！所有相关的源码文件都已被纳入字典管理，无遗漏！")
        return False, "\n".join(report_lines)
    else:
        report_lines.append(f"⚠️ 扫描结束。共发现 {orphan_count} 个文件未被框架收录。")
        report_lines.append("   (可能是新增加的文件忘记配置，或是被废弃的冗余文件)")
        return True, "\n".join(report_lines)