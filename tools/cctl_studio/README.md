# GMP CCTL Studio

**English** | [简体中文](README_CN.md)

See [architecture and generation plan](ARCHITECTURE.md) for the reviewed target
data model, the `mcs_pmsm_nt` generation boundary, and phased acceptance gates.

This directory contains a Qt offline desktop editor. Newly authored analog circuits
export directly to the existing MNA Solver dialect, while legacy schema-v1 projects
retain the Xyce generator. It follows the useful separation seen in TI
SysConfig-based tools: component metadata and project wiring are data, while one
generic engine validates the data and generates the simulator input.

No TI source code or assets are included. The current prototype does not modify GMP
UDP/TCP communication code and does not yet couple CCTL plant models into Xyce.

## What is implemented

- A PyQt5/Qt 5 desktop editor using the pinned GMP private Python environment.
- A two-level editor: the system layer is a numeric-signal block diagram, and a
  composite block opens its type-specific child editor on double-click.
- System-layer electrical topology, digital module, motor, and signal-adapter
  blocks. Every system-layer port has the `numeric` domain.
- An analog catalog synchronized with `mna_solver.py::parse_netlist()`: R/L/C,
  independent sources, ideal op-amps, IdOpamp, E/G/F/H controlled sources,
  D/M/S devices, and ammeters.
- A ground component searchable as either GND or GMD; both aliases export as MNA
  node `0`.
- MOSFET and voltage-switch composites expose only their power terminals. Export
  expands them in graph order to `MTn/SWn + VPWMn`, which the MNA generator exposes
  as the external `PWMn` command.
- Circuit symbols with user-routed, editable polyline wires. Click a port to start,
  click the canvas for any number of orthogonal corners, then click the destination.
  Ending on a wire or double-clicking a wire creates an electrical junction.
- Digital child layers with input/output, AND/OR/NOT, delay, and one-shot symbols
  plus typed directed logic wires.
- Back and breadcrumb navigation between parent and child layers.
- Searchable component palette, drag/drop and double-click creation, movement, and
  marquee/Shift selection.
- A parameter-first inspector. Execution order appears only on the system layer,
  never on circuit or digital child components.
- 90-degree rotation and horizontal mirroring with transformed connection ports.
  Moving or transforming a component stretches adjacent wire segments orthogonally;
  Space rotates it without clearing the selection.
- Grid-aligned connection terminals, zoom/pan/fit, alignment, and distribution.
- Undo/redo, duplicate/delete, and JSON project open/save.
- Direct MNA `.cir` export accepted by the repository MNA parser.
- JSON component definitions with ports, parameters, validation types, and a Xyce
  netlist template.
- JSON projects containing instances, connectivity, transient analysis, and probes.
- Generation of conservative SPICE/Xyce `.cir` netlists.
- Optional component libraries. Adding a component JSON file does not require a
  change to the Python generator.
- A runner that finds Xyce from `--xyce`, `XYCE_EXECUTABLE`, or `PATH`.
- Unit tests and an RC low-pass example.

The first line of a SPICE netlist is its title, device statements follow, analysis
and output directives start with `.`, and the file ends with `.END`. Xyce accepts
this plain-text netlist form; `.cir` is a conventional extension, not a different
file format. Vendor-specific PSpice/HSPICE/Spectre syntax may still need translation.

## Try it

Run `tools\gmp_installer\activate_env.bat` to activate the installed GMP private
environment, then execute these commands from the repository root:

```powershell
tools\cctl_studio\run_cctl_studio.bat
tools\cctl_studio\run_cctl_studio.bat tools\cctl_studio\cctl_core\examples\rc_low_pass\project.json
```

The editor can also be launched with
`python tools/cctl_studio/cctl_core/qt_studio.py [project.json]`. It opens at
`System [system]`; double-click a composite module to enter its child. Click a port,
place polyline corners on the canvas, and click the destination port. During routing,
Space toggles the orthogonal corner direction. Drag a wire segment directly;
double-click a wire to create a junction. Middle-drag or right-drag empty canvas pans,
the wheel zooms, F6 fits, Space or R rotates, M mirrors,
and Alt+Left returns to the parent layer.

The command-line generator remains independently available:

```powershell
python tools/cctl_studio/cctl_core/cctl_studio.py list-components
python tools/cctl_studio/cctl_core/cctl_studio.py validate tools/cctl_studio/cctl_core/examples/rc_low_pass/project.json
$output = Join-Path $env:GMP_PRO_LOCATION 'tmp\cctl_studio\rc_low_pass\cctl_studio_rc.cir'
python tools/cctl_studio/cctl_core/cctl_studio.py generate tools/cctl_studio/cctl_core/examples/rc_low_pass/project.json -o $output
```

When Xyce is installed:

```powershell
$runOutput = Join-Path $env:GMP_PRO_LOCATION 'tmp\cctl_studio\rc_low_pass\xyce'
python tools/cctl_studio/cctl_core/cctl_studio.py run tools/cctl_studio/cctl_core/examples/rc_low_pass/project.json -o $runOutput --xyce C:/path/to/Xyce.exe
```

The generated `.PRINT TRAN FORMAT=CSV` directive writes the configured waveform file
in the run directory.

## Add a component without changing the generator

Create a JSON file following the files in `cctl_core/components/`. A minimal DC current source
would use a template such as:

```json
{
  "schema_version": 1,
  "id": "local.current_dc",
  "display_name": "DC current source",
  "instance_prefix": "I",
  "ports": [{ "name": "p" }, { "name": "n" }],
  "parameters": {
    "current": { "type": "spice_scalar", "required": true }
  },
  "xyce": {
    "netlist_template": "$instance $port_p $port_n DC $param_current"
  }
}
```

Reference it with a project-relative `libraries` entry or pass its file/directory
with `--library`. Template substitution is deliberately limited to one netlist line;
this prevents a data file from silently injecting extra analyses or output commands.

## Compatibility boundary

The editor currently reads and writes schema v1. Optional `editor` metadata holds
layout, z-order, view state, and execution order. The existing Xyce generator
ignores that member, so legacy projects open directly and saved projects generate
the same netlist. This compatibility layer does not replace the planned schema-v2
typed explicit connection model.

`editor.hierarchy` records each layer kind, nodes, connections, view state, and
composite `child_layer`. Legacy `project.instances` are mapped to the default Main
Topology compatibility child and retain their Xyce path. New analog children use
explicit edges and export to the current MNA Solver netlist dialect. Digital
children and cross-layer system graphs are not silently included in CCTL code
generation until schema v2 owns that contract.

All Studio Python sources, built-in component data, examples, and tests live under
`cctl_core/`; the outer directory provides the batch entry points.

## Intended next layers

1. Formal JSON Schema and migration of this UI framework to the normalized
   schema-v2 model.
2. Normalize editor-private hierarchy nodes into generated hierarchical/subcircuit
   modules and vendor model manifests.
3. Typed digital/control ports and a deterministic CCTL-to-Xyce co-simulation bridge.
4. ADC, encoder, PWM and probe adapters with explicit sample-time contracts.
5. An OpenDSS operating-point importer above the transient project format.

The JSON model is the stable boundary: a future SysConfig-like UI should read and
write the same project data instead of embedding component knowledge in UI code.
