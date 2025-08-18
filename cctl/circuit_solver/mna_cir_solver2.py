import argparse
from pathlib import Path

# 从其他模块导入核心功能
from mna_analyzer import analyze_circuit
from mna_writer import write_results_to_json

def main():
    """
    主函数，负责解析命令行参数并协调程序的执行流程。
    """
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
        "--simplify-level",
        type=str,
        choices=['full', 'light', 'none'],
        default='full',
        help="Set the expression simplification level:\n"
             "  'full':  (Default) Thorough simplification. Slowest, but best results.\n"
             "  'light': Basic simplification after solving. Good balance.\n"
             "  'none':  No simplification. Fastest, but expressions are raw.\n"
    )
    args = parser.parse_args()
    
    # --- 验证输入文件 ---
    if not args.netlist_file.is_file():
        print(f"Error: File not found at {args.netlist_file}")
        exit(1)
    if args.netlist_file.suffix.lower() != '.cir':
        print(f"Warning: Input file '{args.netlist_file.name}' does not have a .cir extension.")

    # --- 执行分析 ---
    analysis_results = analyze_circuit(args.netlist_file, args.tina, args.verbose, args.simplify_level)
    
    # --- 导出结果 ---
    if analysis_results[0] is not None:
        solutions, state_variables, subs_dict, phys_quantities = analysis_results
        output_file_path = args.netlist_file.with_name(f"{args.netlist_file.stem}_results.json")
        write_results_to_json(output_file_path, solutions, state_variables, subs_dict, phys_quantities, args.input_source, args.verbose, args.simplify_level)
    else:
        print("\nAnalysis failed. No output file will be generated.")

if __name__ == '__main__':
    main()
