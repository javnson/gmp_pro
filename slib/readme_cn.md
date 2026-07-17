# GMP Simulink Library（slib）

[English](readme.md) | **简体中文**

`slib` 保存 GMP MATLAB/Simulink 库源码、版本化安装目录和安装脚本。

在 MATLAB 中运行：

```matlab
run(fullfile(getenv('GMP_PRO_LOCATION'), ...
    'slib', 'install_gmp_simulink_lib.m'));
```

安装器会先检查 Simscape Electrical Specialized Power Systems 是否存在并可加载，然后部署当前 MATLAB Release 对应的 GMP 库。Specialized Power Systems 从 MATLAB R2026a 起已被移除，因此现有模型应使用 R2025b 或更早版本。

修改库时应编辑 `simulink_lib_src`，不能直接修改 `install_path` 中的部署副本。升级前如需清理旧版本，可运行 `uninstall_gmp_simulink_lib.m`。
