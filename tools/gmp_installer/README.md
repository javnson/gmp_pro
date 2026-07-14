# GMP Pro development environment installers

GMP Pro provides two compatible installation modes. Both first validate the
repository path and register the user environment variable
`GMP_PRO_LOCATION=<gmp_pro root>`. The variable is also set immediately for the
current installer process, so no new command prompt is needed during setup.

The classic system mode installs user-scoped applications with Scoop and enables
user-wide vcpkg integration. The private mode keeps GMP-managed tools under
`gmp_pro/bin`; it does not install Scoop, change the persistent user `PATH`, or
run user-wide `vcpkg integrate install`.

Visual Studio with the Desktop development with C++ workload and a Windows SDK
is a host prerequisite. Visual Studio itself is intentionally not copied into
`bin`; its licensing and installer servicing model do not support treating an
existing VS installation as a portable repository tool. The activation script
detects it with `vswhere` and imports its x64 developer-command-prompt settings.

## Layout

```text
bin/
  python/                  private Python 3.12 and site-packages
  apps/
    git/                   portable MinGit
    cmake/
    ninja/
    doxygen/
    graphviz/
  vcpkg/                   pinned vcpkg registry and executable
  vcpkg_installed/         shared manifest installation tree
  cache/                   downloads and vcpkg binary/download caches
  gmp_environment.json     generated inventory (contains no absolute root path)
```

The complete `bin/` folder is ignored by Git.

## 1. Install the classic system environment

For maximum compatibility with the historical GMP setup, run:

```bat
install_gmp.bat
```

This installs or verifies Scoop-managed Git, Python, CMake, Ninja, Doxygen,
Graphviz, and vcpkg; installs the shared Python requirement set; runs
`vcpkg integrate install`; restores the motor-control simulation manifest; and
runs the complete repository setup, including CCS product registration.

## 2. Build a private environment from zero

Run from the repository root:

```bat
install_gmp_virtual_env.bat
```

The initial Python installer is downloaded with Windows `curl.exe`. All later
downloads and extraction are performed by the private Python interpreter. The
installer honors existing `HTTP_PROXY` and `HTTPS_PROXY` variables.

The default installation also restores the vcpkg manifest for:

```text
ctl/suite/mcs_pmsm/project/motor_control_simulink/vcpkg.json
```

and runs the facilities/source-manager repository setup. Useful diagnostic
options are:

```bat
install_gmp_virtual_env.bat --skip-vcpkg-packages
install_gmp_virtual_env.bat --skip-project-setup
```

These options are intended for installer debugging; they produce an incomplete
environment and should not be used when preparing a `bin` folder for others.

## 3. Deploy a copied private environment

Copy a fully prepared `bin` folder beside `install_gmp.bat`, then run:

```bat
deploy_gmp_env.bat
```

This mode performs no downloads and no package installations. It validates the
Python version and imports, portable applications, vcpkg executable, and the
required `asio`, `fmt`, and `nlohmann-json` package trees. It then runs only the
repository-local configuration/generation steps. The generated environment
inventory deliberately stores versions rather than the original absolute path,
so moving the repository does not require rewriting it.

For validation without repository generation:

```bat
deploy_gmp_env.bat --skip-project-setup
```

## 4. Enter the private environment

Double-click or run:

```bat
gmp_env.bat
```

This opens a new x64 GMP developer prompt. It prepends private tools to `PATH`,
reads the registered `GMP_PRO_LOCATION`, isolates Python from user site-packages, configures the
vcpkg caches/toolchain, and then imports Visual Studio's developer environment.
It does not persist any additional user or machine environment variables.

A single command can also be run without opening an interactive prompt:

```bat
gmp_env.bat python --version
gmp_env.bat cmake --version
gmp_env.bat msbuild ctl\suite\mcs_pmsm\project\motor_control_simulink\motor_control_simulink.vcxproj /p:Platform=x64
```

Scripts that need to activate the environment in an existing command prompt can
use:

```bat
call tools\gmp_installer\activate_env.bat
```

## Maintenance

Pinned application versions and URLs are in `environment_manifest.json`.
Python packages are pinned in `requirements-gmp.txt`. Update these two files
together, build a fresh environment, run the doctor command, and only then
publish/copy the resulting `bin` folder:

```bat
bin\python\python.exe tools\gmp_installer\environment_manager.py doctor
```

The Visual Studio project consumes local vcpkg through `Directory.Build.props`
and `Directory.Build.targets` only when `bin/vcpkg` exists. Otherwise it remains
compatible with the classic mode's user-wide `vcpkg integrate install` setup.
