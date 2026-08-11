# GMP Keil Pack generator

This tool converts every registered CTL module with physical files in
`tools/facilities_generator/src_mgr/gmp_framework_dic.json` into CMSIS-Pack
components. Package metadata comes from `GMP.GeneralMotorPlatform.xml`.

Run `build_keil_pack.bat` to create the `.pdsc` and `.pack` files under
`build/keil_pack`, or pass `--check` to validate generation without keeping an
artifact. Repository-relative paths are preserved; files are never flattened.

Select `Portable:Common` and one `Portable:Target` variant in Keil RTE, then
define `GMP_CTL_PORTABLE` globally. Add new CTL content to the source-manager
registry instead of hard-coding it in this generator.
