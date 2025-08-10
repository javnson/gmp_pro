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

    # --- BUG FIX STARTS HERE ---
    # Create the substitutions dictionary for JSON output by converting values to floats
    json_substitutions = {}
    for k, v in substitutions.items():
        # Check if the value is a Symengine numeric type
        if hasattr(v, 'is_Number'):
            json_substitutions[str(k)] = str(float(v))
        else: # It's a symbol (e.g., from 'SYMBOLIC' keyword)
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
        numerical_val = expr.subs(full_subs_dict)
        output_data["physicalQuantitiesNumerical"]["results"][key] = str(process_expr(numerical_val))

    if verbose: print("Processing symbolic and numerical solutions...")
    for var in state_vars:
        if var in solutions:
            var_str = str(var)
            symbolic_sol = str(solutions[var])
            numerical_sol = str(process_expr(solutions[var].subs(substitutions)))
            
            output_data["symbolicExpressions"]["solutions"][var_str] = symbolic_sol
            output_data["numericalResults"]["solutions"][var_str] = numerical_sol

    if verbose: print("Processing symbolic and numerical transfer functions...")
    valid_tf_vars = [var for var in state_vars if var in solutions and v_in_source in solutions[var].free_symbols]
    for var in valid_tf_vars:
        H_symbolic = solutions[var] / v_in_source
        H_numeric = H_symbolic.subs(substitutions)
        
        tf_key = f"H({var}/{v_in_source})"
        output_data["symbolicExpressions"]["transferFunctions"][tf_key] = format_as_poly(H_symbolic, s)
        output_data["numericalResults"]["transferFunctions"][tf_key] = format_as_poly(H_numeric, s)

    output_data["symbolicExpressions"]["count"] = len(output_data["symbolicExpressions"]["solutions"]) + len(output_data["symbolicExpressions"]["transferFunctions"])
    output_data["numericalResults"]["count"] = len(output_data["numericalResults"]["solutions"]) + len(output_data["numericalResults"]["transferFunctions"])

    if verbose: print("Finalizing JSON structure and writing to file...")
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=4)

    print("Successfully wrote results to JSON file.")
