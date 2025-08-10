import symengine as se
import json
from mna_utils import format_as_poly

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
                # 如果表达式本身就是0，直接返回
                if p_expr == 0: return se.sympify(0)
                
                # 将表达式看作关于 s 的多项式
                p = se.Poly(p_expr, s)
                
                # 准备用评估后的系数重建多项式
                new_poly_expr = se.sympify(0)
                
                # 2. 遍历多项式的每一个系数
                # p.as_dict() 返回一个类似 {(power,): coeff} 的字典
                for (power,), coeff in p.as_dict().items():
                    evaluated_coeff = coeff
                    # 3. 关键检查：如果系数中不包含任何自由符号，那么它就是一个纯数字（或数字表达式）
                    if not coeff.free_symbols:
                        try:
                            # 使用 .n() 方法将其评估为浮点数
                            evaluated_coeff = coeff.n()
                        except RuntimeError:
                            # 如果评估失败（非常罕见），则保持原样
                            pass
                    
                    # 4. 用新的浮点数系数重建多项式的这一项
                    new_poly_expr += se.sympify(evaluated_coeff) * (s**power)
                return new_poly_expr

            # 5. 分别处理分子和分母，并组合成最终的字符串
            num_str = str(process_poly(num))
            den_str = str(process_poly(den))
            
            if den_str == "1" or den_str == "1.0":
                return f"({num_str})"
            else:
                return f"({num_str}) / ({den_str})"
                
        except Exception:
            # 对于无法处理为s的有理多项式的表达式，进行回退
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
        # 如果表达式不依赖于s，则直接计算其浮点数值
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
            # 如果解不依赖于s，则直接计算其浮点数值
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
        # 对符号结果使用标准格式化
        output_data["symbolicExpressions"]["transferFunctions"][tf_key] = format_as_poly(H_symbolic_simplified, s)
        # 对数值结果使用新的、带浮点数转换的格式化
        output_data["numericalResults"]["transferFunctions"][tf_key] = format_as_poly_numerical(H_numeric_simplified, s)

    output_data["symbolicExpressions"]["count"] = len(output_data["symbolicExpressions"]["solutions"]) + len(output_data["symbolicExpressions"]["transferFunctions"])
    output_data["numericalResults"]["count"] = len(output_data["numericalResults"]["solutions"]) + len(output_data["numericalResults"]["transferFunctions"])

    if verbose: print("Finalizing JSON structure and writing to file...")
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=4)

    print("Successfully wrote results to JSON file.")
