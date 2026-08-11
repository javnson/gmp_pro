# CCTL Studio prototype

This directory contains a small, offline-first validation of a data-driven circuit
authoring flow for CCTL and Xyce. It follows the useful separation seen in TI
SysConfig-based tools: component metadata and project wiring are data, while one
generic engine validates the data and generates the simulator input.

No TI source code or assets are included. The current prototype does not modify GMP
UDP/TCP communication code and does not yet couple CCTL plant models into Xyce.

## What is implemented

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

From the repository root:

```powershell
python tools/cctl_studio/cctl_studio.py list-components
python tools/cctl_studio/cctl_studio.py validate tools/cctl_studio/examples/rc_low_pass/project.json
python tools/cctl_studio/cctl_studio.py generate tools/cctl_studio/examples/rc_low_pass/project.json -o $env:TEMP/cctl_studio_rc.cir
```

When Xyce is installed:

```powershell
python tools/cctl_studio/cctl_studio.py run tools/cctl_studio/examples/rc_low_pass/project.json -o tools/cctl_studio/build/rc --xyce C:/path/to/Xyce.exe
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

## Intended next layers

1. Formal JSON Schema and a graphical editor that renders the same metadata.
2. Hierarchical/subcircuit modules and vendor model file manifests.
3. Typed digital/control ports and a deterministic CCTL-to-Xyce co-simulation bridge.
4. ADC, encoder, PWM and probe adapters with explicit sample-time contracts.
5. An OpenDSS operating-point importer above the transient project format.

The JSON model is the stable boundary: a future SysConfig-like UI should read and
write the same project data instead of embedding component knowledge in UI code.
