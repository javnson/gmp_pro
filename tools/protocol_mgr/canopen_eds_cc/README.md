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

The GUI (PySide6-based) now keeps a single unified entries view that combines the parameter tree and table, with **Entities** on top and **Imports** below:

- single unified entry view (tree-table) for Object/Group hierarchy;
- index entries sharing the same index are automatically grouped into one expandable tree node;
- index node title is shown in a friendly form, e.g. `0x2000 [0x00..0x02] (3 entries: ActualPos (+2))`;
- importing one or more `.eds` files and merging into the current project;
- preserving imported origin as `imports` records;
- generating `.h`/`.c` in one click; optional merged EDS export.
- full SDPE_v2-style interaction:
  - parameter tree as the main entry editor surface;
  - preset management (save/load/delete) for project skeletons under
    `tools/protocol_mgr/canopen_eds_cc/presets`;
  - complete shortcut mapping for editor actions;
  - variable-storage naming helpers;
  - data-type friendly labels (`UNSIGNED16` etc.) with fixed-size auto-sync;
  - double-click inline cell editing (no mandatory editor popup);
  - PDO field is a checkbox for fast mapping enable/disable;
  - Access field uses a combo + `Const` checkbox editor for typo-safe editing;
  - Access column displays concise labels `R`, `W`, `R/W`, `C` (const), rendered in fixed-width/center-aligned style for cleaner engineering readability while internal values remain canonical `ro/wo/rw/const`;
  - import manager (add/edit/remove/move/import replacement)
- left-side quick-open dock:
  - `tools/protocol_mgr/canopen_eds_cc/canopen_eds_cc_gui_config.json` defines default files under `quick_open_projects`;
  - default points to the three GMP CiA profile projects (`cia301/cia401/cia402`);
  - list entries can be shown/hidden via **View → ...** and are loaded from the config (relative paths are resolved from the GMP root);
  - dock can be shown/hidden, and stores a compact list for one-click open.
  - access flags split into dedicated controls (`Read` / `Write` / `Const`) in entry editor.
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

### 2.1) GMP profile project presets

Profile-specific project files are stored with their corresponding EDS files:

- `core/protocol/canopen/cia301/cia301_project.json`
- `core/protocol/canopen/cia401/cia401_project.json`
- `core/protocol/canopen/cia402/cia402_project.json`

These projects keep imports and output directory relative to their own folder, so regeneration can be done directly in place:

```powershell
python tools/protocol_mgr/canopen_eds_cc/canopen_eds_cc.py --project core/protocol/canopen/cia301/cia301_project.json --node-id 1
python tools/protocol_mgr/canopen_eds_cc/canopen_eds_cc.py --project core/protocol/canopen/cia401/cia401_project.json --node-id 1
python tools/protocol_mgr/canopen_eds_cc/canopen_eds_cc.py --project core/protocol/canopen/cia402/cia402_project.json --node-id 1
```

GUI usage:

```powershell
gmp_canopen_eds_cc_gui.bat core/protocol/canopen/cia301/cia301_project.json
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
