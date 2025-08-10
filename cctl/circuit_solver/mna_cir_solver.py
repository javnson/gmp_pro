import symengine as se
import argparse
import json
from pathlib import Path
import re

def parse_value(value_str):
    """
    Helper function to parse a value from the netlist.
    Handles scientific notation and standard SPICE unit suffixes (K, M, U, N, P, etc.).
    Converts values to symengine.Rational for precision and performance.
    If it's not a valid number (e.g., "SYMBOLIC"), returns a symengine.Symbol.
    """
    value_str = value_str.upper()
    
    # SPICE unit suffixes (case-insensitive)
    units = {
        'T': 1e12,
        'G': 1e9,
        'MEG': 1e6,
        'K': 1e3,
        'M': 1e-3,
        'U': 1e-6,
        'N': 1e-9,
        'P': 1e-12,
        'F': 1e-15,
    }

    # Check for unit suffixes
    for suffix, multiplier in units.items():
        if value_str.endswith(suffix):
            numeric_part = value_str[:-len(suffix)]
            try:
                value = float(numeric_part) * multiplier
                return se.Rational(str(value)) # Convert to rational for precision
            except (ValueError, TypeError):
                break 

    try:
        return se.Rational(value_str)
    except (ValueError, TypeError, SyntaxError):
        return se.Symbol(str(value_str))

def analyze_circuit(netlist_path, tina_mode=False, verbose=False, no_simplify=False):
    """
    使用改进节点分析法(MNA)对电路进行符号分析。
    该函数读取一个网表文件，构建MNA方程组 (A*x = z)，
    并符号化地求解节点电压和支路电流。

    --- SPICE-Standard Component Definitions ---
    - R: R<name> n1 n2 <value|Symbolic>
    - C: C<name> n1 n2 <value|Symbolic>
    - L: L<name> n1 n2 <value|Symbolic>
    - V: V<name> n+ n- <value|Symbolic>
    - I: I<name> n+ n- <value|Symbolic>
    - D: D<name> n+ n-                (被建模为符号电阻 R_D<name>)
    - M: M<name> nd ng ns [nb] <model> (被建模为D-S间的符号电阻 R_M<name>, ng和nb被忽略)
    - O: O<name> n+ n- n_out          (理想运放)
    - E: E<name> n+ n- nc+ nc- <gain> (VCVS)
    - G: G<name> n+ n- nc+ nc- <gain> (VCCS)
    - H: H<name> n+ n- V_ctrl <gain>  (CCVS)
    - F: F<name> n+ n- V_ctrl <gain>  (CCCS)
    """
    
    # --- Pass 1: 解析网表, 统计节点和电压源 ---
    print("--- Pass 1: Parsing netlist and identifying sources ---")
    components = []
    highest_node = 0
    vs_names = [] # 存储所有会引入未知电流的元件名
    substitutions = {} # 存储符号到数值的代换规则
    
    VALID_PREFIXES = ('R', 'C', 'L', 'V', 'I', 'D', 'M', 'O', 'E', 'G', 'H', 'F')

    # --- Pre-process file for line continuations (+) ---
    processed_lines = []
    with open(netlist_path, 'r') as f:
        raw_lines = f.readlines()
        
        if tina_mode and raw_lines:
            first_line_content = raw_lines[0].strip()
            if first_line_content:
                print(f"Info: Identified TINA file header (skipped) -> '{first_line_content}'")
            raw_lines = raw_lines[1:]

        for line in raw_lines:
            stripped_line = line.strip()
            if not stripped_line:
                continue
            
            if stripped_line.startswith('+'):
                if processed_lines:
                    processed_lines[-1] += ' ' + stripped_line[1:].strip()
                else:
                    print(f"Warning: Skipped continuation line with no preceding line -> '{stripped_line}'")
            else:
                processed_lines.append(stripped_line)

    for line in processed_lines:
        original_line = line
        line_upper = original_line.upper()
        
        if line_upper.startswith(('*', '.', '%')):
            continue
        
        if not line_upper.startswith(VALID_PREFIXES):
            print(f"Info: Identified and skipped title/header line -> '{original_line}'")
            continue

        parts = line_upper.split()
        components.append(parts)
        name = parts[0]
        
        node_indices = []
        if name.startswith(('R', 'C', 'L', 'V', 'I', 'D', 'H', 'F')):
            node_indices = [1, 2]
        elif name.startswith('O'):
            node_indices = [1, 2, 3]
        elif name.startswith('M'):
            node_indices = [1, 3] 
        elif name.startswith(('E', 'G')):
            node_indices = [1, 2, 3, 4]
        
        nodes_in_comp = [int(parts[i]) for i in node_indices if i < len(parts) and parts[i].isdigit()]

        if nodes_in_comp:
            highest_node = max(highest_node, *nodes_in_comp)

        if name.startswith(('V', 'O', 'E', 'H')):
            vs_names.append(name)
    
    n = highest_node
    m = len(vs_names)

    vs_map = {name: i for i, name in enumerate(vs_names)}
    print(f"Found {n} nodes and {m} voltage sources/op-amps.")
    if verbose:
        print(f"Voltage source map (name -> index): {vs_map}")

    # --- Pass 2: 初始化矩阵并填充 ---
    print("\n--- Pass 2: Stamping components into MNA matrices ---")
    s = se.Symbol('s')
    
    G = se.zeros(n, n)
    B = se.zeros(n, m)
    C = se.zeros(m, n)
    D = se.zeros(m, m)
    i_vec = se.zeros(n, 1)
    e_vec = se.zeros(m, 1)
    
    for comp in components:
        name = comp[0]
        comp_symbol = se.Symbol(name)
        
        # --- Stamping logic ---
        if name.startswith(('R', 'C', 'L')):
            n1, n2 = int(comp[1]), int(comp[2])
            idx1, idx2 = (n1 - 1 if n1 != 0 else -1), (n2 - 1 if n2 != 0 else -1)
            if len(comp) > 3 and comp[3].upper() != 'SYMBOLIC':
                value = parse_value(comp[3])
                substitutions[comp_symbol] = value
            if name.startswith('R'): g = 1 / comp_symbol
            elif name.startswith('C'): g = s * comp_symbol
            elif name.startswith('L'): g = 1 / (s * comp_symbol)
            
            if idx1 != -1: G[idx1, idx1] += g
            if idx2 != -1: G[idx2, idx2] += g
            if idx1 != -1 and idx2 != -1:
                G[idx1, idx2] -= g
                G[idx2, idx1] -= g

        elif name.startswith('D'):
            n1, n2 = int(comp[1]), int(comp[2])
            idx1, idx2 = (n1 - 1 if n1 != 0 else -1), (n2 - 1 if n2 != 0 else -1)
            resistor_name = se.Symbol('R_' + name)
            g = 1 / resistor_name
            if idx1 != -1: G[idx1, idx1] += g
            if idx2 != -1: G[idx2, idx2] += g
            if idx1 != -1 and idx2 != -1:
                G[idx1, idx2] -= g
                G[idx2, idx1] -= g
        
        elif name.startswith('M'): # M<name> nd ng ns
            nd, ns = int(comp[1]), int(comp[3])
            idx_d, idx_s = (nd - 1 if nd != 0 else -1), (ns - 1 if ns != 0 else -1)
            resistor_name = se.Symbol('R_' + name)
            g = 1 / resistor_name
            if idx_d != -1: G[idx_d, idx_d] += g
            if idx_s != -1: G[idx_s, idx_s] += g
            if idx_d != -1 and idx_s != -1:
                G[idx_d, idx_s] -= g
                G[idx_s, idx_d] -= g

        elif name.startswith('I'):
            n1, n2 = int(comp[1]), int(comp[2])
            idx1, idx2 = (n1 - 1 if n1 != 0 else -1), (n2 - 1 if n2 != 0 else -1)
            if len(comp) > 3 and comp[3].upper() != 'SYMBOLIC':
                value = parse_value(comp[3])
                substitutions[comp_symbol] = value
            if idx1 != -1: i_vec[idx1] -= comp_symbol
            if idx2 != -1: i_vec[idx2] += comp_symbol

        elif name.startswith('V'):
            n1, n2 = int(comp[1]), int(comp[2])
            idx1, idx2 = (n1 - 1 if n1 != 0 else -1), (n2 - 1 if n2 != 0 else -1)
            vs_idx = vs_map[name]
            if len(comp) > 3 and comp[3].upper() != 'SYMBOLIC':
                value = parse_value(comp[3])
                substitutions[comp_symbol] = value
            e_vec[vs_idx] = comp_symbol
            if idx1 != -1: B[idx1, vs_idx] = 1; C[vs_idx, idx1] = 1
            if idx2 != -1: B[idx2, vs_idx] = -1; C[vs_idx, idx2] = -1

        elif name.startswith('O'): # O<name> n+ n- n_out
            vs_idx = vs_map[name]
            n_plus, n_minus, n_out = int(comp[1]), int(comp[2]), int(comp[3])
            
            if n_out == 0:
                print(f"\n[FATAL ERROR] Invalid configuration for Op-Amp '{name}'.")
                print("The output of an op-amp cannot be connected to ground (node 0). This creates a singular matrix.")
                return None, None, None, None

            idx_plus = n_plus - 1 if n_plus != 0 else -1
            idx_minus = n_minus - 1 if n_minus != 0 else -1
            idx_out = n_out - 1
            
            B[idx_out, vs_idx] = 1
            if idx_plus != -1: C[vs_idx, idx_plus] = 1
            if idx_minus != -1: C[vs_idx, idx_minus] = -1

        elif name.startswith('E'): # E<name> n+ n- nc+ nc- gain
            n1, n2 = int(comp[1]), int(comp[2])
            idx1, idx2 = (n1 - 1 if n1 != 0 else -1), (n2 - 1 if n2 != 0 else -1)
            vs_idx = vs_map[name]
            nc1, nc2 = int(comp[3]), int(comp[4])
            idx_c1, idx_c2 = nc1-1 if nc1 != 0 else -1, nc2-1 if nc2 != 0 else -1
            if len(comp) > 5 and comp[5].upper() != 'SYMBOLIC':
                gain = parse_value(comp[5])
                substitutions[comp_symbol] = gain
            
            if idx1 != -1: B[idx1, vs_idx] = 1; C[vs_idx, idx1] = 1
            if idx2 != -1: B[idx2, vs_idx] = -1; C[vs_idx, idx2] = -1
            if idx_c1 != -1: C[vs_idx, idx_c1] -= comp_symbol
            if idx_c2 != -1: C[vs_idx, idx_c2] += comp_symbol

        elif name.startswith('G'): # G<name> n+ n- nc+ nc- gain
            n1, n2 = int(comp[1]), int(comp[2])
            idx1, idx2 = (n1 - 1 if n1 != 0 else -1), (n2 - 1 if n2 != 0 else -1)
            nc1, nc2 = int(comp[3]), int(comp[4])
            idx_c1, idx_c2 = nc1-1 if nc1 != 0 else -1, nc2-1 if nc2 != 0 else -1
            if len(comp) > 5 and comp[5].upper() != 'SYMBOLIC':
                gain = parse_value(comp[5])
                substitutions[comp_symbol] = gain
            
            if idx1 != -1 and idx_c1 != -1: G[idx1, idx_c1] += comp_symbol
            if idx1 != -1 and idx_c2 != -1: G[idx1, idx_c2] -= comp_symbol
            if idx2 != -1 and idx_c1 != -1: G[idx2, idx_c1] -= comp_symbol
            if idx2 != -1 and idx_c2 != -1: G[idx2, idx_c2] += comp_symbol

        elif name.startswith('H'): # H<name> n+ n- V_ctrl gain
            n1, n2 = int(comp[1]), int(comp[2])
            idx1, idx2 = (n1 - 1 if n1 != 0 else -1), (n2 - 1 if n2 != 0 else -1)
            vs_idx = vs_map[name]
            ctrl_idx = vs_map[comp[3]]
            if len(comp) > 4 and comp[4].upper() != 'SYMBOLIC':
                gain = parse_value(comp[4])
                substitutions[comp_symbol] = gain
            D[vs_idx, ctrl_idx] = -comp_symbol
            if idx1 != -1: B[idx1, vs_idx] = 1; C[vs_idx, idx1] = 1
            if idx2 != -1: B[idx2, vs_idx] = -1; C[vs_idx, idx2] = -1
            
        elif name.startswith('F'): # F<name> n+ n- V_ctrl gain
            n1, n2 = int(comp[1]), int(comp[2])
            idx1, idx2 = (n1 - 1 if n1 != 0 else -1), (n2 - 1 if n2 != 0 else -1)
            ctrl_idx = vs_map[comp[3]]
            if len(comp) > 4 and comp[4].upper() != 'SYMBOLIC':
                gain = parse_value(comp[4])
                substitutions[comp_symbol] = gain
            if idx1 != -1: B[idx1, ctrl_idx] += gain
            if idx2 != -1: B[idx2, ctrl_idx] -= gain

    A = se.zeros(n + m, n + m)
    A[:n, :n] = G
    if m > 0:
        A[:n, n:] = B
        A[n:, :n] = C
        A[n:, n:] = D

    z_vec = se.zeros(n + m, 1)
    z_vec[:n, :] = i_vec
    if m > 0:
        z_vec[n:, :] = e_vec
    
    if verbose:
        print("\n--- Verbose Output: MNA System ---")
        print("A matrix (before solving):")
        print(A)
        print("\nz vector (before solving):")
        print(z_vec)
        print("---------------------------------")

    v_vars = [se.Symbol(f'v_{k}') for k in range(1, n + 1)]
    j_vars = [se.Symbol('I_' + name) for name in vs_names]
    x_vars = v_vars + j_vars
    
    print("\nSolving MNA equations symbolically... (This may take a long time for complex circuits)")
    try:
        if verbose: print("Step 1: Performing LU decomposition and solving...")
        solution_vec = A.LUsolve(z_vec)
        if verbose: print("Step 1: Solving complete.")
    except Exception as e:
        print(f"\nError solving matrix equations: {e}")
        print("The matrix may be singular. Please check your circuit netlist.")
        return None, None, None, None

    if verbose: print("Step 2: Expanding/Simplifying final expressions...")
    if no_simplify:
        results = {var: expr for var, expr in zip(x_vars, solution_vec)}
        if verbose: print("Step 2: Simplification skipped.")
    else:
        results = {var: se.expand(expr) for var, expr in zip(x_vars, solution_vec)}
        if verbose: print("Step 2: Expansion complete.")
    
    physical_quantities = {}
    for comp in components:
        name = comp[0]
        comp_symbol = se.Symbol(name)
        
        if name.startswith(('R', 'C', 'L', 'V', 'I', 'D', 'E', 'G', 'H', 'F')):
            n1, n2 = int(comp[1]), int(comp[2])
        elif name.startswith('O'):
            n1, n2 = int(comp[1]), int(comp[2])
        elif name.startswith('M'):
            n1, n2 = int(comp[1]), int(comp[3])
        else:
            continue
        
        v1_sym = se.Symbol(f'v_{n1}') if n1 != 0 else 0
        v2_sym = se.Symbol(f'v_{n2}') if n2 != 0 else 0
        
        V_expr = v1_sym - v2_sym
        physical_quantities[f'V_{name}'] = V_expr

        if name.startswith('R'): I_expr = V_expr / comp_symbol
        elif name.startswith('C'): I_expr = V_expr * (s * comp_symbol)
        elif name.startswith('L'): I_expr = V_expr / (s * comp_symbol)
        elif name.startswith('I'): I_expr = comp_symbol
        elif name.startswith(('V', 'O', 'E', 'H')): I_expr = se.Symbol('I_' + name)
        else: I_expr = se.Symbol('I_uncalculated')
        
        physical_quantities[f'I_{name}'] = I_expr

    return results, x_vars, substitutions, physical_quantities

def write_results_to_json(output_path, solutions, state_vars, substitutions, physical_quantities, input_source_name, verbose=False, no_simplify=False):
    """将分析结果按照指定顺序写入JSON文件"""
    print(f"\nWriting results to {output_path}...")
    v_in_source = se.Symbol(input_source_name)
    
    def process_expr(expr):
        if no_simplify:
            return expr
        return se.expand(expr)

    output_data = {
        "parameters": {
            "count": len(substitutions),
            "substitutions": {str(k): str(v) for k, v in substitutions.items()}
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
    for key, expr in physical_quantities.items():
        substituted_expr = expr.subs(solutions)
        numerical_val = substituted_expr.subs(substitutions)
        output_data["physicalQuantitiesNumerical"]["results"][key] = str(process_expr(numerical_val))

    if verbose: print("Processing symbolic and numerical solutions...")
    for var in state_vars:
        if var in solutions:
            var_str = str(var)
            symbolic_sol = str(solutions[var]) # Already processed in analyze_circuit
            numerical_sol = str(process_expr(solutions[var].subs(substitutions)))
            
            output_data["symbolicExpressions"]["solutions"][var_str] = symbolic_sol
            output_data["numericalResults"]["solutions"][var_str] = numerical_sol

    if verbose: print("Processing symbolic and numerical transfer functions...")
    valid_tf_vars = [var for var in state_vars if var in solutions and v_in_source in solutions[var].free_symbols]
    for var in valid_tf_vars:
        H_symbolic = solutions[var] / v_in_source
        H_numeric = H_symbolic.subs(substitutions)
        
        tf_key = f"H({var}/{v_in_source})"
        output_data["symbolicExpressions"]["transferFunctions"][tf_key] = str(process_expr(H_symbolic))
        output_data["numericalResults"]["transferFunctions"][tf_key] = str(process_expr(H_numeric))

    output_data["symbolicExpressions"]["count"] = len(output_data["symbolicExpressions"]["solutions"]) + len(output_data["symbolicExpressions"]["transferFunctions"])
    output_data["numericalResults"]["count"] = len(output_data["numericalResults"]["solutions"]) + len(output_data["numericalResults"]["transferFunctions"])

    if verbose: print("Finalizing JSON structure and writing to file...")
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=4)

    print("Successfully wrote results to JSON file.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Perform symbolic MNA on a SPICE-like netlist (.cir file) and export transfer functions.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "netlist_file",
        type=Path,
        help="Path to the .cir netlist file."
    )
    parser.add_argument(
        "--input_source",
        type=str,
        default="Vin",
        help="Name of the input source for calculating transfer functions (default: Vin)."
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output to show detailed steps of the analysis."
    )
    parser.add_argument(
        "--tina",
        action="store_true",
        help="Enable TINA mode: treats the first line of the netlist as a title to be skipped."
    )
    parser.add_argument(
        "--no-simplify",
        action="store_true",
        help="Disable the final expression expansion/simplification step for performance."
    )
    args = parser.parse_args()
    
    if not args.netlist_file.is_file():
        print(f"Error: File not found at {args.netlist_file}")
        exit(1)
    if args.netlist_file.suffix.lower() != '.cir':
        print(f"Warning: Input file '{args.netlist_file.name}' does not have a .cir extension.")

    analysis_results = analyze_circuit(args.netlist_file, args.tina, args.verbose, args.no_simplify)
    
    if analysis_results[0] is not None:
        solutions, state_variables, subs_dict, phys_quantities = analysis_results
        output_file_path = args.netlist_file.with_name(f"{args.netlist_file.stem}_results.json")
        write_results_to_json(output_file_path, solutions, state_variables, subs_dict, phys_quantities, args.input_source, args.verbose, args.no_simplify)
    else:
        print("\nAnalysis failed. No output file will be generated.")
