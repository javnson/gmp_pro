# GMP Peripheral Simulator

**English** | [简体中文](readme_cn.md)

This directory is reserved for host-side peripheral simulation used by GMP
tests and SIL workflows. Implementations placed here should document the
emulated peripheral contract, timing model, transport, supported platforms, and
the test or suite that validates them.

Do not depend on target-only headers or embed repository absolute paths. Resolve
shared resources through `GMP_PRO_LOCATION` and keep reusable protocol logic
separate from GUI or operating-system adapters.
