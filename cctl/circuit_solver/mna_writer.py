import symengine as se
import json
from mna_utils import format_as_poly

def write_results_to_json(output_path, solutions, state_vars, substitutions, physical_quantities, input_source_name, verbose=False, simplify_level='full'):
    """将分析结果按照指定顺序写入JSON文件"""
    print(f"\nWriting results to {output_path}...")
    v_in_source = se.Symbol(input_source_name)
    s = se.Symbol('s')

    # --- HOTFIX STARTS HERE ---
    # 补丁：修复在主分析器中可能发生的数值解析错误。
    # 此代码块会检查代入字典，并尝试重新解析那些被错误识别为符号的数值。
    if verbose: print("Applying hotfix to substitution values...")
    fixed_substitutions = {}
    
    # 定义单位，遵循 SPICE 标准惯例
    units = {
        'T': 1e12, 'G': 1e9, 'MEG': 1e6, 'K': 1e3,
        'M': 1e-3,  # 关键修正：根据 SPICE 惯例，M/m 代表 milli (10^-3)。
        'U': 1e-6, 'N': 1e-9, 'P': 1e-12, 'F': 1e-15,
    }
    sorted_suffixes = sorted(units.keys(), key=len, reverse=True)

    for k, v in substitutions.items():
        # 如果值是一个符号 (Symbol)，则尝试重新解析它
        if v.is_Symbol:
            val_str = str(v).upper()
            parsed = False
            for suffix in sorted_suffixes:
                if val_str.endswith(suffix):
                    numeric_part = val_str[:-len(suffix)]
                    try:
                        num_val = float(numeric_part) * units[suffix]
                        fixed_substitutions[k] = se.Rational(str(num_val))
                        parsed = True
                        if verbose: print(f"  - Hotfix applied: Converted symbol '{val_str}' to {num_val}")
                        break
                    except (ValueError, TypeError):
                        continue
            if not parsed:
                fixed_substitutions[k] = v # 如果无法解析，则保留原始符号
        else:
            # 如果值已经是数字，则直接保留
            fixed_substitutions[k] = v
    
    # 使用修复后的字典进行后续所有计算
    substitutions = fixed_substitutions
    # --- HOTFIX ENDS HERE ---
    
    def process_expr(expr):
        """根据简化级别对表达式进行基础处理。"""
        if simplify_level == 'full':
            return se.expand(expr)
        return expr

    def format_as_poly_numerical(expr, s):
        """
        将有理表达式格式化为s的多项式之比，并将所有纯数字系数计算为浮点数。
        这是为了让数值结果更清晰易读。
        """
        try:
            # 1. 分离分子和分母
            num, den = se.fraction(se.cancel(expr))

            def process_poly(p_expr):
                """辅助函数：处理单个多项式，将其系数转为浮点数。"""
                if p_expr == 0: return se.sympify(0)
                p = se.Poly(p_expr, s)
                new_poly_expr = se.sympify(0)
                for (power,), coeff in p.as_dict().items():
                    evaluated_coeff = coeff
                    if not coeff.free_symbols:
                        try:
                            evaluated_coeff = coeff.n()
                        except RuntimeError:
                            pass
                    new_poly_expr += se.sympify(evaluated_coeff) * (s**power)
                return new_poly_expr

            num_str = str(process_poly(num))
            den_str = str(process_poly(den))
            
            if den_str == "1" or den_str == "1.0":
                return f"({num_str})"
            else:
                return f"({num_str}) / ({den_str})"
                
        except Exception:
            return str(expr)

    json_substitutions = {}
    for k, v in substitutions.items():
        if v.is_Number:
            json_substitutions[str(k)] = str(float(v))
        else:
            json_substitutions[str(k)] = str(v)

    output_data = {
        "parameters": {"count": len(substitutions), "substitutions": json_substitutions},
        "physicalQuantities": {"count": len(physical_quantities), "definitions": {key: str(val) for key, val in physical_quantities.items()}},
        "physicalQuantitiesNumerical": {"count": len(physical_quantities), "results": {}},
        "symbolicExpressions": {"solutions": {}, "transferFunctions": {}},
        "numericalResults": {"solutions": {}, "transferFunctions": {}}
    }

    if verbose: print("Processing numerical physical quantities...")
    full_subs_dict = substitutions.copy()
    full_subs_dict.update(solutions)
    for key, expr in physical_quantities.items():
        numerical_expr = expr.subs(full_subs_dict)
        if not numerical_expr.free_symbols:
            try:
                numerical_result = str(numerical_expr.n())
            except RuntimeError:
                numerical_result = str(process_expr(numerical_expr))
        else:
            numerical_result = str(process_expr(numerical_expr))
        output_data["physicalQuantitiesNumerical"]["results"][key] = numerical_result

    if verbose: print("Processing symbolic and numerical solutions...")
    for var in state_vars:
        if var in solutions:
            var_str = str(var)
            symbolic_sol = str(solutions[var])
            
            numerical_expr = solutions[var].subs(substitutions)
            if not numerical_expr.free_symbols:
                try:
                    numerical_sol = str(numerical_expr.n())
                except RuntimeError:
                    numerical_sol = str(process_expr(numerical_expr))
            else:
                numerical_sol = str(process_expr(numerical_expr))
            
            output_data["symbolicExpressions"]["solutions"][var_str] = symbolic_sol
            output_data["numericalResults"]["solutions"][var_str] = numerical_sol

    if verbose: print("Processing symbolic and numerical transfer functions...")
    valid_tf_vars = [var for var in state_vars if var in solutions and v_in_source in solutions[var].free_symbols]
    for var in valid_tf_vars:
        H_symbolic_raw = solutions[var] / v_in_source
        if verbose: print(f"  Simplifying transfer function for {var}...")
        H_symbolic_simplified = se.cancel(H_symbolic_raw)
        H_numeric_substituted = H_symbolic_simplified.subs(substitutions)
        H_numeric_simplified = se.cancel(H_numeric_substituted)

        tf_key = f"H({var}/{v_in_source})"
        output_data["symbolicExpressions"]["transferFunctions"][tf_key] = format_as_poly(H_symbolic_simplified, s)
        output_data["numericalResults"]["transferFunctions"][tf_key] = format_as_poly_numerical(H_numeric_simplified, s)

    output_data["symbolicExpressions"]["count"] = len(output_data["symbolicExpressions"]["solutions"]) + len(output_data["symbolicExpressions"]["transferFunctions"])
    output_data["numericalResults"]["count"] = len(output_data["numericalResults"]["solutions"]) + len(output_data["numericalResults"]["transferFunctions"])

    if verbose: print("Finalizing JSON structure and writing to file...")
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=4)

    print("Successfully wrote results to JSON file.")
