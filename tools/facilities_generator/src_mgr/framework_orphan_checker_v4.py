import os
import re
from pathlib import Path

def run_orphan_check(model):
    """
    执行游离资产(未映射文件)扫描
    终极版：优先加载本地 .gmpignore，并支持媲美 Git 的 ** 跨级正则匹配引擎
    """
    report_lines = []
    orphan_count = 0
    
    report_lines.append("="*50)
    report_lines.append(" 👻 GMP 框架游离文件扫描报告 (Orphan Check)")
    report_lines.append(" (说明: 物理存在但未被收录进 JSON 字典的源码文件)")
    report_lines.append("="*50 + "\n")

    gmp_base = Path(model.gmp_location).resolve()
    dict_dir = model.json_path.parent.resolve() # 字典(工具)所在的目录
    registry = model.registry
    
    # ================= 1. 智能加载忽略规则 (.gmpignore) =================
    ignore_patterns = ['.git', 'build', 'doc', 'docs', '.svn', '__pycache__']
    
    # 优先在工具当前目录寻找，其次在框架根目录寻找
    ignore_file_path = dict_dir / '.gmpignore'
    if not ignore_file_path.exists():
        ignore_file_path = gmp_base / '.gmpignore'

    if ignore_file_path.exists():
        report_lines.append(f"ℹ️ 已加载自定义忽略规则: {ignore_file_path}")
        with open(ignore_file_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    ignore_patterns.append(line)
    else:
        report_lines.append("ℹ️ 未检测到 .gmpignore，使用系统默认过滤规则。")
        
    report_lines.append("")

    # ================= 2. 编译媲美 Git 的强力正则引擎 =================
    compiled_ignores = []
    for pat in ignore_patterns:
        p = pat.strip().replace('\\', '/')
        if not p: continue
        
        is_root_anchored = p.startswith('/')
        if is_root_anchored: p = p[1:]
        if p.endswith('/'): p = p[:-1]
        
        # 将特殊通配符转换为独特的 Token，防止被 re.escape 误伤
        p = p.replace('/**/', '__GLOB_MID__')
        if p.startswith('**/'): p = '__GLOB_START__' + p[3:]
        if p.endswith('/**'): p = p[:-3] + '__GLOB_END__'
        p = p.replace('**', '__GLOB_ANY__')
        p = p.replace('*', '__GLOB_ONE__')
        p = p.replace('?', '__GLOB_CHAR__')
        
        # 基础正则转义 (保护 . + ( ) 等字符)
        p = re.escape(p)
        
        # 将 Token 还原为强大的正则表达式
        p = p.replace('__GLOB_MID__', '(?:/|/.+/)')   # 匹配中间任意级目录
        p = p.replace('__GLOB_START__', '(?:.+/)?')   # 匹配开头任意级目录
        p = p.replace('__GLOB_END__', '(?:/.*)?')     # 匹配结尾任意级目录及文件
        p = p.replace('__GLOB_ANY__', '.*')           # 兜底的 **
        p = p.replace('__GLOB_ONE__', '[^/]*')        # 单层匹配 *
        p = p.replace('__GLOB_CHAR__', '[^/]')        # 单字符匹配 ?
        
        # 组装最终的锚定正则
        if is_root_anchored:
            regex_str = '^' + p + '(/.*)?$'
        else:
            regex_str = '(^|/)' + p + '(/.*)?$'
            
        compiled_ignores.append(re.compile(regex_str, re.IGNORECASE))

    def is_ignored(rel_path):
        """用编译好的正则库，极速校验路径是否被忽略"""
        rel_path_norm = rel_path.replace('\\', '/')
        for regex in compiled_ignores:
            if regex.search(rel_path_norm):
                return True
        return False

    # ================= 3. 收集已映射的相对路径集合 =================
    mapped_files_rel = set()
    for root_name, modules in registry.get("modules", {}).items():
        for mod_data in modules.values():
            src_abs = model.resolve_paths(mod_data.get("src_patterns", []), return_absolute=True)
            inc_abs = model.resolve_paths(mod_data.get("inc_patterns", []), return_absolute=True)
            
            for p in src_abs + inc_abs:
                abs_path = Path(p).resolve()
                if abs_path.is_relative_to(gmp_base):
                    # 转为全小写标准相对路径，解决 Windows 平台大小写玄学 Bug
                    rel = abs_path.relative_to(gmp_base).as_posix().lower()
                    mapped_files_rel.add(rel)

    target_extensions = {'.c', '.cpp', '.h', '.hpp', '.inl', '.s', '.asm'}

    # ================= 4. 物理硬盘遍历与排查 =================
    for root_name in registry.get("roots", []):
        root_path = gmp_base / root_name
        if not root_path.exists() or not root_path.is_dir():
            continue
            
        root_orphans = []
        for dirpath, dirnames, filenames in os.walk(root_path):
            current_dir_rel = Path(dirpath).relative_to(gmp_base).as_posix()
            
            # --- 过滤子目录 (切断 os.walk 进入被忽略文件夹的路径) ---
            valid_dirs = []
            for d in dirnames:
                dir_rel = f"{current_dir_rel}/{d}" if current_dir_rel != "." else d
                if not is_ignored(dir_rel):
                    valid_dirs.append(d)
            dirnames[:] = valid_dirs 
            
            # --- 检查当前目录下的文件 ---
            for f in filenames:
                file_path = Path(dirpath) / f
                if file_path.suffix.lower() in target_extensions:
                    file_rel = file_path.relative_to(gmp_base).as_posix()
                    
                    if is_ignored(file_rel):
                        continue
                        
                    norm_file_rel = file_rel.lower()
                    if norm_file_rel not in mapped_files_rel:
                        root_orphans.append(file_rel)
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
        report_lines.append("   (提示: 如果这些是测试文件，请在 .gmpignore 中添加规则将其屏蔽)")
        return True, "\n".join(report_lines)