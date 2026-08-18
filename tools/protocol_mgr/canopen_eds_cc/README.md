# canopen_eds_cc

`canopen_eds_cc.py` converts CANopen dictionaries between CiA-306 EDS files, a canonical GMP JSON
manifest, and RB-tree C/Od sources (`.h/.c`). It uses only the Python standard library.

## Execution modes

## GUI mode (GMP launcher)

Launch the editor from GMP environment via:

```powershell
gmp_canopen_eds_cc_gui.bat [path\to\project.json]
```

- When a project file path is provided, it will be opened directly.
- Without argument, it opens an empty project template.
- The launcher ensures `GMP_PRO_LOCATION` and imports GMP Python runtime with:
  `tools\gmp_installer\ensure_gmp_environment.bat`.

The GUI supports:

- editing OD entries in tabular form;
- importing one or more `.eds` files and merging into the current project;
- preserving imported origin as `imports` records;
- generating `.h`/`.c` in one click; optional merged EDS export.
- full SDPE_v2-style interaction:
  - dual entry views:
    - **Entries Table** (flat list)
    - **Parameter Tree** (group path tree view)
  - import manager (add/edit/remove/move/import replacement)
  - preset templates: save/load/delete project templates under
    `tools/protocol_mgr/canopen_eds_cc/presets`
- keyboard shortcuts:
  - `Ctrl+N` new project
  - `Ctrl+O` open project
  - `Ctrl+S` save
  - `Ctrl+Shift+S` save as
  - `Insert` add entry
  - `Enter` edit selected entry
  - `Delete` delete selected entry
  - `Ctrl+I` append entries from EDS
  - `Ctrl+G` generate code
  - `F5` refresh JSON preview
  - `Ctrl+Q` quit

### 1) Legacy mode

The legacy mode keeps compatibility with previous usage:

```powershell
python canopen_eds_cc.py device.eds --output-dir generated --name device_od --storage pointer --node-id 7
```

Supported fields remain:

- `ParameterName`
- `DataType`
- `AccessType`
- `DefaultValue` or `ParameterValue`
- `PDOMapping`
- `DataLength` for variable-length types

Sections are `[1234]` or `[1234sub5]`.
`GMPStorage=pointer|value` can override the command-line default.

### 2) Project mode (JSON manifest)

Project mode builds from one JSON manifest and supports:

- multiple EDS bindings with inheritance/overrides (`imports`)
- per-entry storage mode:
  - `pointer`: generated internal storage variable
  - `value`: value-copy storage
  - `variable`: existing user variable binding
- hierarchical organization (`tree` / `group_path`)
- custom injected code fragments for generated header/source
  (`code.header_prefix`, `code.source_prefix`, `code.header_suffix`, `code.source_suffix`)
- conflict policy on duplicate objects (`error|first|last`)
- optional `--emit-eds` to export one merged `.eds`

```powershell
python canopen_eds_cc.py --project dictionary.json --output-dir generated --node-id 7 --emit-eds generated/combo.eds
```

### 3) Import mode

Import one or more EDS files into a canonical JSON manifest:

```powershell
python canopen_eds_cc.py --to-json combo.json --import-eds cia301.eds cia402.eds --name gmp_combo_od --node-id 7
```

Example JSON (trimmed):

```json
{
  "version": 1,
  "name": "gmp_combo_od",
  "node_id": 7,
  "default_storage": "pointer",
  "conflict_policy": "last",
  "code": {
    "header_prefix": "",
    "header_suffix": "",
    "source_prefix": "",
    "source_suffix": ""
  },
  "imports": [
    {
      "path": "core/protocol/canopen/cia301/cia301.eds",
      "alias": "CiA301",
      "storage": "pointer",
      "node_id": 7
    }
  ],
  "entries": [
    {
      "index": "0x1000",
      "subindex": "0x00",
      "parameter_name": "Device type",
      "data_type": "0x0007",
      "access": "ro",
      "pdo_mapping": true,
      "default_value": "0x00000000",
      "size": 4,
      "storage": "pointer",
      "group_path": ["CiA301"]
    }
  ]
}
```

### Notes

Generated pointer-storage scalars are emitted as `extern` symbols in header and concrete storage in source.
Generated string/octet/domain fields are emitted as `byte_gt` arrays.
Call `<name>_init()` exactly once to install all generated dictionary entries.
