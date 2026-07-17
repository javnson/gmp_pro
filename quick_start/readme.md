# GMP Quick Start

**English** | [简体中文](readme_cn.md)

The recommended first step is to install the repository-private environment:

```bat
install_gmp_virtual_env.bat
```

This registers `GMP_PRO_LOCATION` and installs the required Python and command
line tools under `bin`. Visual Studio is optional unless you build a PC
simulation project.

After installation, open the GMP prompt:

```bat
gmp_env.bat
```

The `gmp_file_generator` example demonstrates project source selection and the
`usr` directory contains a minimal user-side layout. For real applications,
start from a maintained target under `ctl/suite/<suite>/project`.

See the repository [English manual](../README.md) for the complete workflow.
