# GMP CCTL Studio

**English** | [简体中文](README_CN.md)

See [architecture and generation plan](ARCHITECTURE.md) for the reviewed target
data model, the `mcs_pmsm_nt` generation boundary, and phased acceptance gates.

This directory contains an offline desktop graphical editor and a data-driven
circuit-authoring flow for CCTL and Xyce. It follows the useful separation seen in TI
SysConfig-based tools: component metadata and project wiring are data, while one
generic engine validates the data and generates the simulator input.

No TI source code or assets are included. The current prototype does not modify GMP
UDP/TCP communication code and does not yet couple CCTL plant models into Xyce.

## What is implemented

- A Python/Tk 8.6 desktop editor with no additional GUI runtime dependency.
- Searchable component palette, drag/drop and double-click creation, movement,
  marquee/Shift selection, and keyboard nudging.
- Port-to-port wiring, net merge/disconnect, property editing, execution-order
  badges, and project settings.
- Grid snapping, zoom/pan/fit, alignment, distribution, and drawing order.
- Undo/redo, copy/paste/duplicate/delete, and JSON project open/save.
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
tools\cctl_studio\run_cctl_studio.bat tools\cctl_studio\examples\rc_low_pass\project.json
```

The editor can also be launched with
`python tools/cctl_studio/studio_gui.py [project.json]`. Drag between port dots to
create a net; middle-drag or Space-drag pans, the wheel zooms, and F6 fits the
diagram. Arrow keys nudge by one world unit and Shift+arrow uses one grid step.

The command-line generator remains independently available:

```powershell
python tools/cctl_studio/cctl_studio.py list-components
python tools/cctl_studio/cctl_studio.py validate tools/cctl_studio/examples/rc_low_pass/project.json
$output = Join-Path $env:GMP_PRO_LOCATION 'tmp\cctl_studio\rc_low_pass\cctl_studio_rc.cir'
python tools/cctl_studio/cctl_studio.py generate tools/cctl_studio/examples/rc_low_pass/project.json -o $output
```

When Xyce is installed:

```powershell
$runOutput = Join-Path $env:GMP_PRO_LOCATION 'tmp\cctl_studio\rc_low_pass\xyce'
python tools/cctl_studio/cctl_studio.py run tools/cctl_studio/examples/rc_low_pass/project.json -o $runOutput --xyce C:/path/to/Xyce.exe
```

The generated `.PRINT TRAN FORMAT=CSV` directive writes the configured waveform file
in the run directory.

## Add a component without changing the generator

Create a JSON file following the files in `components/`. A minimal DC current source
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

## Intended next layers

1. Formal JSON Schema and migration of this UI framework to the normalized
   schema-v2 model.
2. Hierarchical/subcircuit modules and vendor model file manifests.
3. Typed digital/control ports and a deterministic CCTL-to-Xyce co-simulation bridge.
4. ADC, encoder, PWM and probe adapters with explicit sample-time contracts.
5. An OpenDSS operating-point importer above the transient project format.

The JSON model is the stable boundary: a future SysConfig-like UI should read and
write the same project data instead of embedding component knowledge in UI code.
