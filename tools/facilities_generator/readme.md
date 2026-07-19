# GMP Facilities and Source Generator

**English** | [简体中文](readme_cn.md)

This directory contains repository registration, CCS product metadata, and GMP
project source-management tools.

## Installation tasks

- `gmp_fac_install_ccs_product.py` registers GMP as a CCS product.
- `gmp_fac_generate_cfg_json.py` generates facility configuration metadata.
- `src_mgr/framework_distribute_tools_v3.py` distributes current source-manager
  launchers and regenerates project headers and sources.

The root installers run these tasks automatically. They resolve the repository
through `GMP_PRO_LOCATION`; do not invoke them with embedded absolute paths.

## Source manager

Each project `gmp_src_mgr/gmp_framework_config.json` selects required GMP
modules. The distributed BAT tools generate `gmp_inc`, `gmp_src`, and
`gmp_compiler_includes.txt`. Generated output is ignored by Git in the main
repository, while the BAT tools remain available when a project is copied out.

Edit canonical launchers only under `src_mgr/gmp_src_mgr`, then run the
distributor. Build-tree copies under `Debug`, `Release`, or another ignored path
are excluded through the repository's real `.gitignore` rules.

Historical implementation notes are retained in
[`development_notes.md`](development_notes.md).
