# GMP Facilities and Source Generator

**English** | [简体中文](readme_cn.md)

This directory contains repository registration, CCS product metadata, and GMP
project source-management tools.

## Installation tasks

- `ccs_product_installer/GMP_CCS_Product_Info.json` is the source of truth for
  common and family-specific C28x/C29x Product metadata.
- `ccs_product_installer/install_ccs_products.bat` validates the registry and
  generates each Product's `.metadata/product.json` and `.metadata/.tirex`
  files.
- `gmp_fac_install_ccs_product.py/.bat` are compatibility forwarding entry
  points for older callers.
- `gmp_fac_generate_cfg_json.py` generates facility configuration metadata.
- `src_mgr/framework_distribute_tools_v3.py` distributes current source-manager
  launchers and regenerates project headers and sources.

The root installers run these tasks automatically. They resolve the repository
through `GMP_PRO_LOCATION`; do not invoke them with embedded absolute paths.

After generation, add `csp\c28x_syscfg` and/or `csp\c29x_syscfg` to the CCS
Product discovery paths. Both Products export `GMP_PRO_ROOT`; the selected
family also exports `GMP_C28X_CSP_ROOT` or `GMP_C29X_CSP_ROOT`. Projects must no
longer depend on `COM_GMP_PRO_SDK_INSTALL_DIR`. Use `--product c28x` or
`--product c29x` for one family and `--check` for a non-writing consistency
check.

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
