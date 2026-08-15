# Generated CSP source notice

Files synchronized by the GMP source manager are generated copies. Do not make
an upstream fix inside a generated `gmp_src_mgr/gmp_inc` or
`gmp_src_mgr/gmp_src` tree; edit the canonical GMP source or change the local
module selection instead.

If a project intentionally carries a private driver modification, exclude the
corresponding CSP module from its source-manager selection before regenerating,
otherwise synchronization may overwrite that private copy.

# 生成 CSP 源码说明

GMP 源管理器同步的文件属于生成副本。不要在生成的 `gmp_src_mgr/gmp_inc` 或
`gmp_src_mgr/gmp_src` 中做上游修复；应修改 GMP 仓库中的规范源码，或调整当前工程的模块选择。

如果工程确实需要保留私有驱动修改，请在重新生成前从源管理器选择中排除对应 CSP 模块，否则同步过程可能覆盖该私有副本。
