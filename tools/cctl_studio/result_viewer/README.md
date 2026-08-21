# GMP CCTL Result Viewer

This Qt/pyqtgraph utility plots very large numeric CCTL CSV/TSV result files without loading them into Excel. It loads only selected columns in a worker thread, caches them, and uses extrema-preserving min/max decimation before display.

Launch `run_result_viewer.bat`, optionally passing a result filename. Each plot has an independent curve set. Enable linked X zoom to align time axes or linked Y zoom for fully synchronized scaling. See `README_CN.md` for the complete workflow.
