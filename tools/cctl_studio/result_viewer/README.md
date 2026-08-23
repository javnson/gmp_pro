# GMP CCTL Simulation Viewer Manager

This Qt/pyqtgraph utility plots very large numeric CCTL CSV/TSV result files without loading them into Excel. It streams only selected columns and retains a bounded time segment in memory (1 s by default), then applies extrema-preserving min/max decimation before display. Select the newest segment or a fixed start time on the **Configuration** page. A file may remain open in the simulator while it is viewed: **Dynamic refresh (20 Hz)** reads newly appended, newline-terminated rows every 50 ms. A partially written final line is deferred, and a malformed trailing row is ignored without discarding the valid prefix.

For repeated PMSM simulations, open the previous CSV, add the desired curves, enable dynamic refresh, and then rerun the simulator. When the simulator truncates and rewrites the same file, the viewer discards the old samples and follows the new run automatically.

The Viewer defaults to a rolling 0.1 s X window, like an oscilloscope. The
visible window and the retained-memory window are independent and neither
deletes historical CSV data. The equivalent command-line form is
`--rolling-window 0.1`.

Install the completed GMP private environment and configure `GMP_PRO_LOCATION`, then
launch `run_result_viewer.bat`, optionally passing a result filename. The launcher
validates the environment and explicitly uses
`%GMP_PRO_LOCATION%\bin\python\python.exe`, so it cannot accidentally select a
system Python. Waveforms live on multiple tabs. The **Data and curves** panel is
shown beside waveform tabs rather than on the Configuration page. Y signals are
organized as a tree with one top-level node per result file and raw signal names
as children. Hidden file-plus-column keys still disambiguate duplicate names;
leaf nodes support cross-file multi-selection and double-click addition. The Layout
menu opens a 4-column by 6-row visual picker: hovering previews the rectangle
from the upper-left cell and clicking selects any grid from 1x1 through 6x4.
The Waveforms menu adds or removes pages and plots. Selecting a grid immediately creates exactly that many plots;
the plots are equally sized and fill the page in a MATLAB-style tiled layout.
Shrinking a layout preserves the leading plots and removes excess ones. Each
plot has an independent curve set. Axis links are page-local; enable linked X
or Y independently. See `README_CN.md` for the complete workflow.

The toolbar provides pan, X-only rectangle zoom, Y-only rectangle zoom, two-axis magnifier, and fit actions. Double-click a source column to add it to the active blue-bordered plot; remove active-plot curves from the curve list or with `Delete`. Double-click a title inside a plot to rename it.

**Auto-fit visible data** is enabled by default. It explicitly scans cached
samples inside the current visible X interval, computes finite Y bounds with 5%
padding, and updates after live refresh, rolling, or manual X-range changes.
Linked Y axes use the combined bounds of their page. Disable Auto-fit to
strictly preserve manual Y-axis zoom.

The **Configuration** page owns simulator and display settings. Start, Pause,
Resume, Stop, progress, and metrics remain in one fixed row at the bottom of the
window. The Viewer uses `QProcess`; control and status are JSON, while user and
GMP text output remains in the launching console. Output streams announced by
the simulator are attached automatically and refreshed at 20 Hz. Starting a
CCTL executable with no arguments opens this manager and automatically restarts
that executable under supervision. A newly opened managed Viewer prepares the
supervised child in its wait-for-start state, so every CSV header and selectable
channel is available before simulation. It remains in Stop with zero numerical
steps until the user presses Start. Use `--headless` for command-line-only use.

## Data Link and online debugging

The **Data Link** page reuses the maintained GMP Data Link Studio engine and
provides its Raw, Echo, Tunable, Memory, Chronos, and Data Link Scope pages.
It does not open a physical serial port. Standard framed Data Link bytes are
Base64-encoded inside the supervised JSON channel, delivered through the CCTL
CSP virtual communication peripheral, processed by the target application's
normal `gmp_dev_dl_loop_cb()` and facility dispatcher, and returned over the
same route. Discovery and online access remain available while the numerical
simulation is paused.

Only facilities actually registered by the target respond. The current
`mcs_pmsm_nt/project/cctl` target provides Echo, Tunable, and whitelisted
Memory through this page. Chronos and Data Link Scope remain visible for targets
that register those standard facilities.
