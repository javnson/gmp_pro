# CCTL Studio architecture and generation plan

**English** | [简体中文](ARCHITECTURE_CN.md)

## Reviewed baseline

The repository already contains three useful layers: a data-driven Xyce
prototype, a switched MNA solver and C++ generator, and the complete
`ctl/suite/mcs_pmsm_nt/project/cctl` closed-loop reference. The reference proves
the numerical and runtime path, but it is a golden integration case rather than
a source file that should be emitted verbatim for every graph.

The following reference inputs are suitable for generation:

- the circuit netlist and solver settings;
- SDPE requirements and private hardware calibration;
- source-manager module selection;
- component instances, typed ports, connections, parameters, probes, sample
  periods, and execution order;
- reusable CSP, peripheral, circuit, motor, and output-service implementations.

The following project details are currently handwritten and must become either
metadata or reusable templates before the reference can be generated safely:

- `PmsmCircuit`, `PWM1..PWM6`, `VADC_*`, phase routing, and the seven-channel ADC
  contract;
- the three-ePWM/one-eQEP aggregate and controller register bindings;
- motor parameter unit conversions and circuit-to-motor current/voltage coupling;
- record structures, CSV columns, sample rates, and probe routing;
- CMake target names, generated header/archive paths, backend selection, and
  post-build deployment;
- validation rules that currently know PMSM phase names and bridge numbers.

Code under `gmp_src_mgr/gmp_src`, generated SDPE outputs, generated circuit JSON,
matrix archives, and generated C++ calculation classes are outputs. A Studio
project, component metadata, SDPE requirements, source-manager configuration,
netlists, and generator templates are sources of truth.

## Canonical graph model

TI Interconnect Studio 1.8 stores editable examples as SysConfig JavaScript. Its
stable concepts are `scripting.addModule`, `addInstance`, property assignments,
`scripting.connect(source, port, target, port)`, `$position`, and
`scripting.probe`. CCTL Studio should preserve those concepts, not copy TI module
identifiers, implementation code, or assets.

The canonical CCTL format should remain non-executable JSON and contain:

- schema/tool versions and project metadata;
- component-library references;
- instances with stable IDs, module IDs, display names, positions, parameters,
  and optional execution metadata;
- explicit connection objects with source and target instance/port IDs;
- probes and result-stream definitions;
- solver, timing, backend, and build settings.

The current schema-v1 electrical format stores a node name inside every instance
port. It remains valid for the Xyce prototype, but it is not sufficient for the
multi-domain editor because it cannot represent direction, typed point-to-point
control links, execution order, or adapters without inference. Schema v2 should
use explicit edges. An importer can normalize v1 node labels into electrical-net
objects, and a SysConfig adapter can import/export the compatible semantic subset.
Byte-for-byte compatibility with TI `.syscfg` is neither required nor safe because
that format is executable JavaScript and refers to TI-specific module catalogs.

Every port must declare a domain (`electrical`, `mechanical`, `analog_signal`,
`digital`, or `control`), direction, data type, unit, and rate contract where
applicable. Voltage/current and ADC/PWM/encoder conversions are explicit adapter
instances rather than hidden connection behavior.

Execution metadata should identify a phase and an integer order. At minimum the
phases are `pre_control`, `control`, `plant`, and `observe`. The generator validates
unique order within a phase and dependency direction; when order is omitted it
uses a deterministic topological order. An MNA subnet is one atomic plant module,
not a separately ordered item for each resistor.

## Two-level editor contract

The root system-composition layer carries numeric signals only. Electrical
topologies, motors, digital modules, and voltage/current adapters are blocks with
typed `numeric` ports; resistors, logic gates, and physical wires are not rendered
at this layer.

A composite block opens a `child_layer`, whose `kind` selects its renderer:

- `circuit` uses standard electrical symbols, undirected electrical nets, and
  user-editable orthogonal polyline wires. Components support 90-degree rotation
  and mirroring; child-layer inspectors prioritize parameters and omit order;
- `digital` uses logic/timing symbols and directed logic connections;
- future mechanical, thermal, or other domains receive separate renderers rather
  than inheriting root-layer block semantics.

The compatibility implementation stores this graph in schema-v1
`editor.hierarchy` and maps existing `project.instances` into the default Main
Topology compatibility child. New analog circuits share their catalog with the
MNA parser and explicit edges export directly to its netlist dialect. Digital and
cross-layer system graphs do not enter CCTL code generation until schema v2 defines
that contract.

MOSFET and VSWITCH nodes are composite editor components: the UI exposes only power
terminals, assigns `PWM1..PWMn` in stable graph order, and expands each node to the
`MT/SW` device plus ideal `VPWM` control source required by the MNA examples. MOSFET
bulk is tied to source; VSWITCH control negative is tied to node `0`. GMD is a UI
search alias, while canonical MNA ground remains `0/GND`.

## Deterministic generation pipeline

```text
project.cctl.json + component libraries + SDPE requirements
  -> schema, type, unit, rate, connectivity, and order validation
  -> normalized graph with stable IDs
  -> MNA netlist/circuit data and C++ solver
  -> CCTL topology bindings, MCU adapters, probes, and output schema
  -> source-manager selection and CMake manifest
  -> compile, unit checks, generated-model checks, closed-loop regression
```

Generation must be atomic, path-independent, reproducible, and fail before
overwriting the last good output. Handwritten extension points use separate files;
generated files carry a banner and are never reverse-edited.

## Phased delivery and acceptance gates

1. **Foundation** — maintain the new `cctl/component` layout, source-manager
   registry, baseline tests, and this reviewed reference contract.
2. **Schema v2 and catalog** — publish JSON Schema, stable IDs, typed ports,
   explicit edges, positions, execution phases/order, migration from schema v1,
   and component metadata for MNA subnets, motors, adapters, and peripherals.
3. **Headless project generator** — generate a second build directory for the
   PMSM reference without editing its handwritten golden project. The generated
   build must pass the same model-gain, routing, compilation, and 40-million-step
   regression checks.
4. **Graphical editor** — the first Qt two-level UI framework now implements a
   numeric root block diagram, composite navigation, the complete MNA component
   catalog, circuit and digital renderers, a parameter-first inspector, editable
   polyline wires, rotate/mirror, layout, undo/redo, deterministic save, and MNA
   netlist export through compatible schema-v1 `editor.hierarchy` metadata. Migrate
   its document model to the schema-v2 normalizer next, then invoke the headless
   generator.
5. **Interconnect adapter** — import/export the compatible SysConfig concepts and
   add round-trip fixtures. Unsupported TI-specific properties must be retained as
   namespaced extension data or reported, never silently discarded.
6. **Distribution** — package the tool, runtime dependencies, examples, migration
   tool, and CI matrix; document schema compatibility and generated/source
   boundaries.

The current UI work is deliberately a solver-independent editor shell and does not
freeze schema-v1 inference as the final contract. Full multidomain validation,
code generation, and distribution remain gated on the schema-v2 normalizer and
the headless PMSM generation acceptance test; those layers keep the UI, CLI,
generated code, and future importers consistent.
