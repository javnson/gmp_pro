import os
import fnmatch
from pathlib import Path

def run_orphan_check(model):
    """
    执行游离资产(未映射文件)扫描
    升级: 基于相对路径的绝对精准匹配，并支持 .gmpignore 过滤机制
    """
    report_lines = []
    orphan_count = 0
    
    report_lines.append("="*50)
    report_lines.append(" 👻 GMP 框架游离文件扫描报告 (Orphan Check)")
    report_lines.append(" (物理存在但未被收录进 JSON 的源码文件)")
    report_lines.append("="*50 + "\n")

    gmp_base = Path(model.gmp_location).resolve()
    registry = model.registry
    
    # ================= 1. 加载忽略规则 (.gmpignore) =================
    # 默认系统级忽略名单
    ignore_patterns = ['.git', 'build', 'doc', 'docs', '.svn', '__pycache__']
    
    ignore_file_path = gmp_base / '.gmpignore'
    if ignore_file_path.exists():
        report_lines.append(f"ℹ️ 已加载自定义忽略规则文件: {ignore_file_path.name}")
        with open(ignore_file_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    ignore_patterns.append(line)
    else:
        report_lines.append("ℹ️ 未检测到 .gmpignore，使用系统默认过滤规则。")
        
    report_lines.append("")

    def is_ignored(rel_path):
        """检查相对路径是否命中任何忽略规则"""
        rel_path_obj = Path(rel_path)
        for pat in ignore_patterns:
            # 1. 匹配整个相对路径 (支持 / 号，如 ctl/suite)
            if fnmatch.fnmatch(rel_path, pat): 
                return True
            # 2. 匹配单一目录/文件名 (如 suite 会忽略所有名为 suite 的目录)
            for part in rel_path_obj.parts:
                if fnmatch.fnmatch(part, pat):
                    return True
        return False

    # ================= 2. 收集已映射的相对路径集合 =================
    mapped_files_rel = set()
    
    for root_name, modules in registry.get("modules", {}).items():
        for mod_data in modules.values():
            src_abs = model.resolve_paths(mod_data.get("src_patterns", []), return_absolute=True)
            inc_abs = model.resolve_paths(mod_data.get("inc_patterns", []), return_absolute=True)
            
            for p in src_abs + inc_abs:
                abs_path = Path(p).resolve()
                # 确保只计算位于 GMP_PRO_LOCATION 内部的文件
                if abs_path.is_relative_to(gmp_base):
                    # 转换为全小写、正斜杠的相对路径，彻底解决 Windows 大小写差异 Bug
                    rel = abs_path.relative_to(gmp_base).as_posix().lower()
                    mapped_files_rel.add(rel)

    target_extensions = {'.c', '.cpp', '.h', '.hpp', '.inl', '.s', '.asm'}

    # ================= 3. 物理硬盘遍历与排查 =================
    for root_name in registry.get("roots", []):
        root_path = gmp_base / root_name
        if not root_path.exists() or not root_path.is_dir():
            continue
            
        root_orphans = []
        
        for dirpath, dirnames, filenames in os.walk(root_path):
            current_dir_rel = Path(dirpath).relative_to(gmp_base).as_posix()
            
            # --- 过滤子目录 (阻止 os.walk 进入被忽略的文件夹) ---
            valid_dirs = []
            for d in dirnames:
                dir_rel = f"{current_dir_rel}/{d}" if current_dir_rel != "." else d
                if not is_ignored(dir_rel):
                    valid_dirs.append(d)
            dirnames[:] = valid_dirs # 原地修改，os.walk 就不会进被屏蔽的目录了
            
            # --- 检查当前目录下的文件 ---
            for f in filenames:
                file_path = Path(dirpath) / f
                if file_path.suffix.lower() in target_extensions:
                    file_rel = file_path.relative_to(gmp_base).as_posix()
                    
                    if is_ignored(file_rel):
                        continue
                        
                    # 标准化后与字典进行比对
                    norm_file_rel = file_rel.lower()
                    if norm_file_rel not in mapped_files_rel:
                        root_orphans.append(file_rel) # 记录原本的路径以便好看
                        orphan_count += 1
                        
        if root_orphans:
            report_lines.append(f"📁 在根目录 [{root_name}] 中发现游离文件:")
            for orphan in sorted(root_orphans):
                report_lines.append(f"    ❓ {orphan}")
            report_lines.append("")

    report_lines.append("="*50)
    if orphan_count == 0:
        report_lines.append("✅ 完美！除忽略项外，所有物理源码均已被纳入字典管理，无遗漏！")
        return False, "\n".join(report_lines)
    else:
        report_lines.append(f"⚠️ 扫描结束。共发现 {orphan_count} 个文件未被框架收录。")
        report_lines.append("   (提示: 如果这些是测试文件，请将它们添加到 .gmpignore 中屏蔽)")
        return True, "\n".join(report_lines)