# GMP CCTL Result Viewer

This Qt/pyqtgraph utility plots very large numeric CCTL CSV/TSV result files without loading them into Excel. It loads only selected columns in a worker thread, caches them, and uses extrema-preserving min/max decimation before display. A file may remain open in the simulator while it is viewed: enable **Dynamic refresh (20 Hz)** to read only newly appended, newline-terminated rows every 50 ms. A partially written final line is deferred until it becomes complete, and a malformed trailing row is ignored without discarding the valid prefix.

Launch `run_result_viewer.bat`, optionally passing a result filename. Each plot has an independent curve set. Enable linked X zoom to align time axes or linked Y zoom for fully synchronized scaling. See `README_CN.md` for the complete workflow.
