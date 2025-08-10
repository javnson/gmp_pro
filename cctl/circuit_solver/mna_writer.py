import symengine as se
import json
from mna_utils import format_as_poly

def write_results_to_json(output_path, solutions, state_vars, substitutions, physical_quantities, input_source_name, verbose=False, simplify_level='full'):
    """将分析结果按照指定顺序写入JSON文件"""
    print(f"\nWriting results to {output_path}...")
    v_in_source = se.Symbol(input_source_name)
    s = se.Symbol('s')
    
    def process_expr(expr):
        if simplify_level == 'full':
            return se.expand(expr)
        return expr

    def format_as_poly_numerical(expr, s):
        """
        将有理表达式格式化为s的多项式之比，并将所有纯数字系数计算为浮点数。
        """
        try:
            num, den = se.fraction(se.cancel(expr))

            def process_poly(p_expr):
                if p_expr == 0: return se.sympify(0)
                
                p = se.Poly(p_expr, s)
                
                # 使用评估后的系数重建多项式表达式
                new_poly_expr = se.sympify(0)
                # p.as_dict() 返回一个类似 {(power,): coeff} 的字典
                for (power,), coeff in p.as_dict().items():
                    evaluated_coeff = coeff
                    # 如果系数没有自由符号，则它是纯数字
                    if not coeff.free_symbols:
                        try:
                            # .n() 将其评估为浮点数
                            evaluated_coeff = coeff.n()
                        except RuntimeError:
                            # 如果评估失败，则保持原样
                            pass
                    new_poly_expr += se.sympify(evaluated_coeff) * (s**power)
                return new_poly_expr

            num_str = str(process_poly(num))
            den_str = str(process_poly(den))
            
            if den_str == "1":
                return f"({num_str})"
            else:
                return f"({num_str}) / ({den_str})"
                
        except Exception:
            # 对于无法处理为s的有理多项式的表达式，进行回退
            return str(expr)

    # --- BUG FIX STARTS HERE ---
    # Create the substitutions dictionary for JSON output by converting values to floats
    json_substitutions = {}
    for k, v in substitutions.items():
        # --- FIX: Use the .is_Number property for a robust check ---
        # This correctly identifies numeric types and prevents trying to convert a Symbol to a float.
        if v.is_Number:
            json_substitutions[str(k)] = str(float(v))
        else: # It's a symbol (e.g., from 'SYMBOLIC' keyword) or an expression
            json_substitutions[str(k)] = str(v)
    # --- BUG FIX ENDS HERE ---

    output_data = {
        "parameters": {
            "count": len(substitutions),
            "substitutions": json_substitutions
        },
        "physicalQuantities": {
            "count": len(physical_quantities),
            "definitions": {key: str(val) for key, val in physical_quantities.items()}
        },
        "physicalQuantitiesNumerical": {
            "count": len(physical_quantities),
            "results": {}
        },
        "symbolicExpressions": {
            "solutions": {},
            "transferFunctions": {}
        },
        "numericalResults": {
            "solutions": {},
            "transferFunctions": {}
        }
    }

    if verbose: print("Processing numerical physical quantities...")
    full_subs_dict = substitutions.copy()
    full_subs_dict.update(solutions)
    for key, expr in physical_quantities.items():
        numerical_expr = expr.subs(full_subs_dict)
        # --- EDIT: Evaluate to float if possible ---
        # Check if the expression is frequency-dependent. If not, evaluate to float.
        if s in numerical_expr.free_symbols:
            numerical_result = str(process_expr(numerical_expr))
        else:
            try:
                # .n() evaluates the expression to a floating-point number.
                numerical_result = str(numerical_expr.n())
            except RuntimeError:
                # Fallback if it cannot be evaluated (e.g., contains other symbols)
                numerical_result = str(process_expr(numerical_expr))
        output_data["physicalQuantitiesNumerical"]["results"][key] = numerical_result
        # --- END OF EDIT ---

    if verbose: print("Processing symbolic and numerical solutions...")
    for var in state_vars:
        if var in solutions:
            var_str = str(var)
            symbolic_sol = str(solutions[var])
            
            # --- EDIT: Evaluate to float if possible ---
            numerical_expr = solutions[var].subs(substitutions)
            if s in numerical_expr.free_symbols:
                numerical_sol = str(process_expr(numerical_expr))
            else:
                try:
                    numerical_sol = str(numerical_expr.n())
                except RuntimeError:
                    numerical_sol = str(process_expr(numerical_expr))
            # --- END OF EDIT ---
            
            output_data["symbolicExpressions"]["solutions"][var_str] = symbolic_sol
            output_data["numericalResults"]["solutions"][var_str] = numerical_sol

    if verbose: print("Processing symbolic and numerical transfer functions...")
    valid_tf_vars = [var for var in state_vars if var in solutions and v_in_source in solutions[var].free_symbols]
    for var in valid_tf_vars:
        # Calculate the raw symbolic transfer function
        H_symbolic_raw = solutions[var] / v_in_source

        # --- OPTIMIZATION: Use cancel() for efficient rational function simplification ---
        if verbose: print(f"  Simplifying transfer function for {var}...")
        # Simplify the symbolic transfer function for a cleaner P/Q form
        H_symbolic_simplified = se.cancel(H_symbolic_raw)
        
        # Substitute numerical values into the simplified symbolic TF
        H_numeric_substituted = H_symbolic_simplified.subs(substitutions)
        
        # It's good practice to cancel again after substitution in case new simplifications are possible
        H_numeric_simplified = se.cancel(H_numeric_substituted)
        # --- END OF OPTIMIZATION ---

        tf_key = f"H({var}/{v_in_source})"
        # --- EDIT: Use different formatters for symbolic and numerical results ---
        output_data["symbolicExpressions"]["transferFunctions"][tf_key] = format_as_poly(H_symbolic_simplified, s)
        output_data["numericalResults"]["transferFunctions"][tf_key] = format_as_poly_numerical(H_numeric_simplified, s)
        # --- END OF EDIT ---

    output_data["symbolicExpressions"]["count"] = len(output_data["symbolicExpressions"]["solutions"]) + len(output_data["symbolicExpressions"]["transferFunctions"])
    output_data["numericalResults"]["count"] = len(output_data["numericalResults"]["solutions"]) + len(output_data["numericalResults"]["transferFunctions"])

    if verbose: print("Finalizing JSON structure and writing to file...")
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=4)

    print("Successfully wrote results to JSON file.")
