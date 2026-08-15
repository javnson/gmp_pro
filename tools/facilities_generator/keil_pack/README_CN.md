# GMP Keil Pack 生成器

[English](README.md) | **简体中文**

该工具读取 source manager 的权威模块字典
`tools/facilities_generator/src_mgr/gmp_framework_dic.json`，将其中已注册且
具有实际文件的 CTL 模块转换成 CMSIS-Pack 组件，并生成 `.pdsc` 与 `.pack`。

元数据（vendor、name、版本、发布日期、URL）来自仓库根目录的
`GMP.GeneralMotorPlatform.xml`。模块文件不会扁平化，而是保持相对 GMP 根目录
的路径，因此同名源文件不会发生覆盖。构建结果默认写入 `build/keil_pack`。

## 使用

```bat
tools\facilities_generator\keil_pack\build_keil_pack.bat
tools\facilities_generator\keil_pack\build_keil_pack.bat --check
```

也可直接运行：

```text
python tools/facilities_generator/keil_pack/gmp_keil_pack.py --help
```

Pack 中的 `Portable:Common` 提供 no-CSP 公共依赖；用户还必须在 Keil RTE
中为 `Portable:Target` 选择 `STM32` 或 `TI DSP` variant，并在工程全局定义
`GMP_CTL_PORTABLE`。普通 CTL 组件会声明对 portable common 的依赖，字典中的
CTL 内部依赖会转换成对应的组件依赖。

新增 CTL 模块时只维护 source manager 字典，然后重新执行构建脚本。不要在
Python 中硬编码模块文件。

生成规则遵循 [Open-CMSIS-Pack PDSC 与 Pack 文件命名规范](https://open-cmsis-pack.github.io/Open-CMSIS-Pack-Spec/main/html/packFormat.html)。
