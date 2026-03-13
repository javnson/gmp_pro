import os
from pathlib import Path

def run_validation(model):
    """
    执行框架完整性校验
    返回: (has_errors: bool, report: str)
    """
    report_lines = []
    error_count = 0
    warning_count = 0

    report_lines.append("="*50)
    report_lines.append(" 🔍 GMP 框架资产完整性校验报告 (Sanity Check)")
    report_lines.append("="*50 + "\n")

    registry = model.registry
    
    for root_name, modules in registry.get("modules", {}).items():
        for mod_key, mod_data in modules.items():
            # 记录当前模块是否有错，方便排版
            mod_has_error = False
            mod_report = []
            
            def check_patterns(patterns, pattern_type, is_dir_mode=False):
                nonlocal error_count, mod_has_error
                for pat in patterns:
                    # 单独解析这一条 pattern
                    resolved = model.resolve_paths([pat], is_dir_mode=is_dir_mode)
                    if not resolved:
                        mod_has_error = True
                        error_count += 1
                        mod_report.append(f"    ❌ [{pattern_type}] 失效/找不到目标: {pat}")

            # 检查源文件、头文件、包含目录和帮助文档
            check_patterns(mod_data.get("src_patterns", []), "源文件")
            check_patterns(mod_data.get("inc_patterns", []), "头文件")
            check_patterns(mod_data.get("inc_dirs", []), "包含目录", is_dir_mode=True)
            check_patterns(mod_data.get("help_docs", []), "帮助文档")

            # 检查依赖是否存在
            for dep in mod_data.get("depends_on", []):
                try:
                    d_root, d_mod = dep.split('|', 1)
                    if d_root not in registry["modules"] or d_mod not in registry["modules"][d_root]:
                        mod_has_error = True
                        error_count += 1
                        mod_report.append(f"    🔗 [依赖缺失] 找不到依赖的模块: {dep}")
                except ValueError:
                    mod_has_error = True
                    error_count += 1
                    mod_report.append(f"    🔗 [依赖格式错误] 非法的依赖声明: {dep}")

            if mod_has_error:
                report_lines.append(f"📦 模块: {root_name} | {mod_key}")
                report_lines.extend(mod_report)
                report_lines.append("")

    report_lines.append("="*50)
    if error_count == 0:
        report_lines.append("✅ 校验通过！所有配置的路径、通配符和依赖均有效。")
        return False, "\n".join(report_lines)
    else:
        report_lines.append(f"⚠️ 校验结束。共发现 {error_count} 处失效配置，请根据报告修正！")
        return True, "\n".join(report_lines)