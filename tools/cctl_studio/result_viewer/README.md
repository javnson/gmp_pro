# GMP CCTL Result Viewer

This Qt/pyqtgraph utility plots very large numeric CCTL CSV/TSV result files without loading them into Excel. It loads only selected columns in a worker thread, caches them, and uses extrema-preserving min/max decimation before display. A file may remain open in the simulator while it is viewed: enable **Dynamic refresh (20 Hz)** to read only newly appended, newline-terminated rows every 50 ms. A partially written final line is deferred until it becomes complete, and a malformed trailing row is ignored without discarding the valid prefix.

For repeated PMSM simulations, open the previous CSV, add the desired curves, enable dynamic refresh, and then rerun the simulator. When the simulator truncates and rewrites the same file, the viewer discards the old samples and follows the new run automatically.

Install the completed GMP private environment and configure `GMP_PRO_LOCATION`, then
launch `run_result_viewer.bat`, optionally passing a result filename. The launcher
validates the environment and explicitly uses
`%GMP_PRO_LOCATION%\bin\python\python.exe`, so it cannot accidentally select a
system Python. Each plot has an independent curve set. Enable linked X zoom to align
only the time axes, linked Y zoom to align only the vertical ranges, or both when
full synchronization is wanted. See `README_CN.md` for the complete workflow.

The toolbar provides pan, X-only rectangle zoom, Y-only rectangle zoom, two-axis magnifier, and fit actions. X and Y links are independent. Double-click a source column to add it to the active blue-bordered plot; remove active-plot curves from the curve list or with `Delete`. Drag the splitter handles to resize plot heights, and double-click a title inside a plot to rename it.
