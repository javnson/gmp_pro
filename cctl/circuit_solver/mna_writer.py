import symengine as se
import json
from mna_utils import format_as_poly, parse_value

def write_results_to_json(output_path, solutions, state_vars, substitutions, physical_quantities, input_source_name, verbose=False, simplify_level='full'):
    """将分析结果按照指定顺序写入JSON文件"""
    print(f"\nWriting results to {output_path}...")
    v_in_source = se.Symbol(input_source_name)
    s = se.Symbol('s')
    
    def process_expr(expr):
        """根据简化级别对表达式进行基础处理。"""
        if simplify_level == 'full':
            return se.expand(expr)
        return expr

    def format_solution_poly(expr, s):
        """
        将一个有理表达式格式化为两个标准多项式的比值。
        - s的每个幂次的系数都会被单独提取和化简。
        - 纯数字系数将被计算为浮点数。
        - 含符号的系数将被展开为最简形式。
        """
        try:
            # 1. 首先对整个表达式进行彻底化简，得到一个清晰的 N/D 形式
            simplified_expr = se.cancel(se.expand(expr))
            
            # 2. 将表达式分离为分子 (num) 和分母 (den)
            num, den = se.fraction(simplified_expr)

            def process_poly_coeffs(p_expr):
                """辅助函数，用于处理单个多项式并格式化其系数。"""
                if p_expr == 0:
                    return "0"
                # 将表达式看作关于 s 的多项式
                p = se.Poly(p_expr, s)
                
                terms = []
                # 3. 遍历多项式的每一个系数 (从最高次幂开始)
                # p.all_coeffs() 返回从高到低的系数列表
                all_coeffs = p.all_coeffs()
                degree = p.degree()

                for i, coeff in enumerate(all_coeffs):
                    power = degree - i
                    if coeff == 0:
                        continue

                    evaluated_coeff = coeff
                    # 4. 检查系数是否为纯数字
                    if not coeff.free_symbols:
                        try:
                            # 计算其浮点数值
                            evaluated_coeff = coeff.n()
                        except RuntimeError:
                            pass # 如果评估失败则保持原样
                    else:
                        # 如果系数包含符号，则对其进行展开化简
                        evaluated_coeff = se.expand(coeff)

                    # 5. 构建当前项的字符串表示
                    if power == 0:
                        term_str = f"{evaluated_coeff}"
                    elif power == 1:
                        # 对于s^1, 如果系数是1, 则省略
                        if evaluated_coeff == 1:
                            term_str = "s"
                        elif evaluated_coeff == -1:
                            term_str = "-s"
                        else:
                            term_str = f"({evaluated_coeff})*s"
                    else:
                        if evaluated_coeff == 1:
                            term_str = f"s**{power}"
                        elif evaluated_coeff == -1:
                            term_str = f"-s**{power}"
                        else:
                            term_str = f"({evaluated_coeff})*s**{power}"
                    terms.append(term_str)
                
                if not terms:
                    return "0"
                # 用 " + " 连接所有项，并处理负号
                return " + ".join(terms).replace("+ -", "- ")

            # 6. 分别处理分子和分母
            num_str = process_poly_coeffs(num)
            den_str = process_poly_coeffs(den)

            # 7. 组合最终结果
            if den_str == "1" or den_str == "1.0":
                return num_str
            else:
                return f"({num_str}) / ({den_str})"

        except Exception:
            # 如果无法处理为多项式，则返回其字符串形式作为后备
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
    full_subs_dict = {**substitutions, **solutions}
    for key, expr in physical_quantities.items():
        numerical_expr = expr.subs(full_subs_dict)
        if s not in numerical_expr.free_symbols:
            try:
                numerical_result = str(numerical_expr.n())
            except RuntimeError:
                numerical_result = str(process_expr(numerical_expr))
        else:
            numerical_result = format_solution_poly(numerical_expr, s)
        output_data["physicalQuantitiesNumerical"]["results"][key] = numerical_result

    if verbose: print("Processing symbolic and numerical solutions...")
    for var in state_vars:
        if var in solutions:
            var_str = str(var)
            symbolic_sol = str(solutions[var])
            
            numerical_expr = solutions[var].subs(substitutions)
            # --- EDIT: Apply new formatting logic ---
            if s not in numerical_expr.free_symbols:
                # 如果解不依赖于频率s，直接计算其数值
                try:
                    numerical_sol = str(numerical_expr.n())
                except RuntimeError:
                    numerical_sol = str(process_expr(numerical_expr))
            else:
                # 如果解依赖于频率s，使用新的多项式格式化函数
                numerical_sol = format_solution_poly(numerical_expr, s)
            # --- END OF EDIT ---
            
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
        # Use the same robust formatter for numerical transfer functions
        output_data["numericalResults"]["transferFunctions"][tf_key] = format_solution_poly(H_numeric_simplified, s)

    output_data["symbolicExpressions"]["count"] = len(output_data["symbolicExpressions"]["solutions"]) + len(output_data["symbolicExpressions"]["transferFunctions"])
    output_data["numericalResults"]["count"] = len(output_data["numericalResults"]["solutions"]) + len(output_data["numericalResults"]["transferFunctions"])

    if verbose: print("Finalizing JSON structure and writing to file...")
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=4)

    print("Successfully wrote results to JSON file.")
