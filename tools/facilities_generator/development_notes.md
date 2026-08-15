# Historical facilities-generator notes

This file is retained only as a compatibility pointer. The former `facilities.json`, `gmp_fac_install.py`, and `gmp_source_dic.json` workflow described by earlier revisions is no longer the source-management contract.

Use the current [facilities overview](readme.md). In particular:

- the canonical module registry is `src_mgr/gmp_framework_dic.json`;
- each project selects modules in `gmp_src_mgr/gmp_framework_config.json`;
- `src_mgr/framework_sync_inc_v3.py` and `framework_sync_src_v3.py` generate the project include/source trees;
- `src_mgr/framework_distribute_tools_v3.py` distributes the canonical launchers;
- CCS Product metadata is managed under `ccs_product_installer`.

The root installers invoke the supported repository setup workflow. Do not recreate the obsolete installer or registry files named above.
