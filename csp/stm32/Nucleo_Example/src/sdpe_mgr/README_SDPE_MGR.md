# LaunchPad SDPE manager

This directory follows the same composed-requirement model used by existing
`ctl/suite` projects, adapted to one CCS project with eight build
configurations.

## Sources of truth

- `sdpe_requirement.json` is the Common requirement. It owns portable timing,
  Data Link, CAN and feature settings.
- `requirements/<BOARD>/sdpe_requirement.json` is the Private requirement for
  one build configuration. It includes the target board as hardware, selects
  ADC/PWM/DAC application channels, and binds the Common file through
  `common_requirements`.
- `../../../../sdpe_component` owns reusable C2000 board schemas and entities.
  The repository-wide `tools/SDPE_v2/sdpe_settings.json` registers that
  library for the CLI, GUI and generator.

This separation is intentional: reusable physical facts belong to CSP,
portable application policy belongs to the Common requirement, and a board's
application choices belong to its Private requirement.

## Generation

Generate one active board with:

```bat
tools\generate_board_sdpe.bat F280049C %GMP_PRO_LOCATION%
```

The Private requirement is the generation entry point. SDPE composes it with
the Common requirement and writes:

```text
C2000Lib_F280049C\launchpad_board.h
src\sdpe_mgr\requirements\F280049C\
  ctrl_settings_matlab_init.m
  launchpad_board_matlab_init.m
```

There is deliberately no separately consumed `src/sdpe_mgr/ctrl_settings.h`.
All C macros required by the selected configuration are merged into its single
`launchpad_board.h`. MATLAB keeps the standard Common-then-Private two-script
chain, but both generated scripts are co-located below the board requirement,
so one board cannot accidentally load another board's stale output.

CCS runs the same command for the active configuration when
`GMP_PREBUILD_SDPE=1`. Generated `.h` and `.m` files must not be edited by hand.
After editing a schema, entity, Common requirement or Private requirement,
regenerate the affected board and rebuild its Debug and Release configurations.

See `doc/SDPE_ARCHITECTURE.md` for the design decision, channel-selection
contract and stale-output safeguards.
