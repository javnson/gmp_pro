# GMP Pro development environment installers

**English** | [简体中文](README_CN.md)

GMP Pro provides two compatible installation modes. Both first validate the
repository path and register the user environment variable
`GMP_PRO_LOCATION=<gmp_pro root>`. The variable is also set immediately for the
current installer process, so no new command prompt is needed during setup.

The classic system mode installs user-scoped applications with Scoop and enables
user-wide vcpkg integration. The private mode keeps GMP-managed tools under
`gmp_pro/bin`; it does not install Scoop, change the persistent user `PATH`, or
run user-wide `vcpkg integrate install`.

Before either installer downloads anything, it checks existing
`HTTPS_PROXY`/`HTTP_PROXY`/`ALL_PROXY` variables and the enabled Windows user
proxy setting. When a proxy is found, the installer displays its address and
asks `Y/N` whether it should be used. The answer affects only that installation
process and its child tools; it does not rewrite the Windows proxy setting. For
a private online installation, the installer always asks whether a proxy should
be used. If no proxy was detected and the user selects `Y`, it asks for a proxy
URL such as `http://127.0.0.1:7890`. The choice is additionally stored under `bin` and loaded
by later GMP prompts and the GMP Visual Studio launcher. Copied-bin deployment
asks again and replaces the source computer's choice. For unattended
installation, set `GMP_INSTALLER_PROXY_CHOICE=Y` or `N` before launching the
installer; interactive runs leave this variable unset.

The root installation/deployment launchers pause after both success and failure
so a window opened by double-click remains visible. Automated callers can
disable this final pause by setting `GMP_INSTALLER_NO_PAUSE=1`; the original
exit code is preserved in both cases.

Visual Studio is an optional capability, not an installation prerequisite.
Hardware/CCS workflows, private Python, documentation, source-management, and
SDPE tools install and run on a computer without Visual Studio. When the
installer detects Visual Studio with the Desktop development with C++ workload,
it also restores the suite simulation vcpkg packages. Otherwise it prints an
`[OPTIONAL]` warning, skips only that step, and still creates the private
environment completion marker. Visual Studio itself is intentionally not copied
into `bin`; its licensing and installer servicing model do not support treating
an existing VS installation as a portable repository tool. After Visual Studio
is installed later, run `repair_gmp_vcpkg.bat` for a private environment or
rerun `install_gmp.bat` for the classic environment to add simulation support.

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
  gmp_proxy.json           private proxy/direct mode selected on this computer
  gmp_proxy_env.bat        generated activation settings for child processes
  gmp_environment.json     generated inventory (contains no absolute root path)
```

The complete `bin/` folder is ignored by Git.

## 1. Install the classic system environment

For maximum compatibility with the historical GMP setup, run:

```bat
install_gmp.bat
```

This installs or verifies Scoop-managed Git, Python, CMake, Ninja, Doxygen,
Graphviz, and vcpkg; installs the shared Python requirement set; and runs the
complete repository setup, including CCS product registration. It runs
`vcpkg integrate install` and restores every suite simulation manifest only when
the Visual Studio x64 C++ workload is detected.

## 2. Build a private environment from zero

Run from the repository root:

```bat
install_gmp_virtual_env.bat
```

The initial Python installer is downloaded with Windows `curl.exe`. All later
downloads and extraction are performed by the private Python interpreter. The
selected proxy is inherited by curl, Python/pip, vcpkg, Git, and Scoop.

When Visual Studio C++ is available, the default installation automatically
restores every vcpkg manifest matching:

```text
ctl/suite/*/project/simulate/vcpkg.json
```

Any `simulate` directory containing a `.sln` or `.vcxproj` is a managed Visual
Studio simulation project and must contain `vcpkg.json`. This convention means
a new suite receives installation support without another installer edit.

and runs CCS product registration plus the facilities/source-manager repository
setup. Useful diagnostic
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
Python version and imports, portable applications, and the vcpkg executable. If
Visual Studio C++ is present, it additionally requires the copied `bin` to
contain the required `asio`, `fmt`, and `nlohmann-json` package trees; without
Visual Studio those optional package trees are not required. It then runs only
the CCS registration and repository configuration/generation steps. The generated environment
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

This opens a new GMP developer prompt. It prepends private tools to `PATH`,
reads the registered `GMP_PRO_LOCATION`, isolates Python from user site-packages, configures the
vcpkg caches/toolchain, and imports Visual Studio's x64 developer environment
when it is available. Without Visual Studio it prints a warning and leaves all
non-Visual-Studio GMP tools active.
It does not persist any additional user or machine environment variables.

A single command can also be run without opening an interactive prompt:

```bat
gmp_env.bat python --version
gmp_env.bat cmake --version
gmp_env.bat msbuild ctl\suite\mcs_pmsm\project\simulate\motor_control_simulink.vcxproj /p:Platform=x64
```

Visual Studio must inherit the GMP environment if its MSBuild-driven vcpkg
restore may need the private proxy. Close an older Visual Studio instance and
launch the solution through:

```bat
gmp_vs.bat ctl\suite\dps_fsbb\project\simulate\GMP_Motor_Control_simulink.sln
```

To change only the private proxy choice, or to repair/pre-download vcpkg tools
and all discovered project dependencies without reinstalling Python and other
applications, use:

```bat
configure_gmp_proxy.bat
repair_gmp_vcpkg.bat
```

`repair_gmp_vcpkg.bat` requires Visual Studio C++ and does not create the
installation completion marker. It
uses the saved proxy to obtain vcpkg's auxiliary CMake/7zip/7zr tools and then
restores every discovered suite manifest.

Scripts that need to activate the environment in an existing command prompt can
use:

```bat
call tools\gmp_installer\activate_env.bat
```

## Automatic environment guard for repository batch files

Repository BAT entry points that can use GMP-managed tools carry this marker:

```bat
rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1
```

`ensure_gmp_environment.bat` treats `bin/gmp_virtual_env_installed.flag` as the
single installation-completion marker and additionally checks that private
Python exists. Both online installation and copied-bin deployment delete any old
flag before doing work and create it atomically only after every required setup
and validation step succeeds. `gmp_environment.json` remains an inventory file;
it is not an installation marker. The guard exports
`GMP_ENV_MODE=virtual` after automatic activation, or `GMP_ENV_MODE=system` when
no completed private environment exists. `GMP_ENV_ACTIVE` records the activated
repository root and prevents repeated activation. The guard preserves the
caller's working directory.

The marker is currently required on the CTL Doxygen generator, SDPE v2 BAT
entry points, GMP Debugger v2 launchers, and source-manager BAT templates.
Search for `GMP_ENV_GUARD` when auditing new or renamed scripts. The coverage
audit can also be run directly:

```bat
python tools\gmp_installer\audit_env_guards.py
```

## Maintainer guide

This section is the maintenance contract for GMP developers. Do not add a
one-off `pip install`, downloader, hard-coded drive letter, or persistent
`PATH` modification to a feature script. The installer must remain the single
place that owns third-party dependencies, while each GMP entry-point script
must select the installed environment through the common guard.

### Sources of truth

| Change | Authoritative file | Other files that normally need review |
| --- | --- | --- |
| Add or update a Python package | `tools/gmp_installer/requirements-gmp.txt` | `environment_manifest.json` (`python.imports`) |
| Change the private Python version | `environment_manifest.json` (`python.version`) | `install_online.bat` (`PYTHON_VERSION` and download URL) |
| Add or update a portable executable | `environment_manifest.json` (`applications`) | `activate_env.bat`, `environment_manager.py`, `install_system.bat` |
| Add a C/C++ package to an existing project | That project's `vcpkg.json` | `environment_manifest.json` only if a new project is introduced |
| Add a GMP BAT entry point | The owning tool directory | `audit_env_guards.py` scope, when a new managed directory is introduced |
| Change a source-manager BAT copied into projects | `tools/facilities_generator/src_mgr/gmp_src_mgr` | Run the framework distributor; do not edit generated project copies |
| Change an SDPE BAT copied into projects | `tools/SDPE_v2/sdpe_mgr` | Run `distribute_sdpe_mgr.py`; project copies are ignored release artifacts |
| Change standalone project ignore rules | `tools/gmp_installer/project_gitignore.template` | Run `distribute_project_gitignores.py`; keep project-only additions outside the managed block |

`tools/gmp_installer/environment_manifest.json` describes the reproducible
private environment. Increment its `environment_version` whenever the published
contents of `bin` change. `schema_version` describes the manifest format and
must be changed only when `environment_manager.py` is updated to understand a
new schema.

### Adding a Python package

All Python dependencies shared by GMP tools belong in:

```text
tools/gmp_installer/requirements-gmp.txt
```

Use the package's pip distribution name and pin an exact version, for example:

```text
example-package==1.2.3
```

Then decide whether the package is required for a healthy, distributable GMP
environment. If it is, add its Python import name to `python.imports` in
`environment_manifest.json`. Distribution names and import names are not
always identical: `pyserial` is imported as `serial`, and `Jinja2` is imported
as `jinja2`. The doctor and copied-environment deployment validate the import
names, not the pip distribution names.

The same requirements file is consumed by both installation modes. The classic
installer installs it into system/user Python, while the private installer
installs it under `bin/python`. Therefore:

- Do not put `pip install` in `install_env.bat`, a service BAT, or an individual
  Python tool.
- Invoke pip as `python -m pip`, so it always targets the Python selected by the
  environment guard.
- Prefer invoking a Python CLI as `python -m package` rather than relying on a
  user-global `Scripts` wrapper.
- Before pinning a version, verify that it supplies a Windows x64 wheel for the
  private Python version. Packages that unexpectedly build native extensions
  can turn a normal install into a Visual Studio/compiler-dependent install.
- If the private Python version changes, update both
  `environment_manifest.json` and `install_online.bat`, then retest every pinned
  package. Changing only one file makes the bootstrap interpreter disagree with
  the declared environment.

After changing Python dependencies, rebuild both a private environment and a
classic environment. At minimum, run:

```bat
bin\python\python.exe -m pip check
bin\python\python.exe tools\gmp_installer\environment_manager.py doctor
```

If a package is optional for only one tool, it may be omitted from
`python.imports`, but it still belongs in the shared requirements file if that
tool is part of a standard GMP installation. Document truly optional packages
next to the owning tool and do not silently install them on first launch.

### Adding a portable executable

A third-party command-line application that must be available in the private
environment is declared in the `applications` array of
`environment_manifest.json`. A normal entry has this shape:

```json
{
  "name": "example",
  "version": "1.2.3",
  "url": "https://vendor.example/example-1.2.3-windows-x64.zip",
  "sha256": "<64 lowercase hexadecimal characters>",
  "destination": "apps/example",
  "strip_single_root": true,
  "executables": [
    "bin/example.exe"
  ]
}
```

The current generic application installer accepts ZIP archives. It downloads
the archive into `bin/cache/downloads`, extracts it into
`bin/<destination>`, optionally removes one common top-level archive directory,
and verifies every path listed in `executables`. Use an HTTPS vendor/release
URL, pin the version, and provide the vendor-published SHA-256 whenever one is
available. Inspect the archive layout before choosing `strip_single_root`.

Adding the manifest entry installs and validates the application, but does not
by itself make its command visible everywhere. Complete all of these steps:

1. Add the executable directory to the `PATH` assembled by
   `tools/gmp_installer/activate_env.bat`.
2. Add the same directory to `private_environment()` in
   `tools/gmp_installer/environment_manager.py`. Repository setup and doctor
   subprocesses use this Python-built environment rather than the BAT file.
3. If classic installation must support the application, add its Scoop package
   name to the install/verification loop in `install_system.bat`.
4. Update the `--plan` output in the relevant installer BAT files so dry-run
   output continues to describe what will actually be installed.
5. Add a version smoke test and, for a required application, keep at least one
   executable path in the manifest so copied-bin deployment can reject an
   incomplete archive.

If the vendor distributes an MSI, EXE installer, tarball, or archive with
special layout, extend the generic download/extraction code in
`environment_manager.py`. Do not hide an application-specific installer inside
an unrelated service BAT. A repository-owned Python program is not a portable
application entry: place it under the appropriate `tools` directory, declare
its Python packages as above, and invoke it through a guarded BAT launcher.

Never store an absolute path in the manifest or generated inventory. Everything
under `bin` must remain relocatable to another computer after
`deploy_gmp_env.bat` validates and finishes the deployment.

### Maintaining vcpkg packages

Add C/C++ dependencies to the owning project's `vcpkg.json`. The standard GMP
suite convention is:

```text
ctl/suite/<suite-name>/project/simulate/vcpkg.json
```

The installer discovers this pattern on every run. If the `simulate` directory
contains one or more top-level `.sln` or `.vcxproj` files but has no
`vcpkg.json`, both installation modes fail with the missing manifest path.
Therefore, adding a new suite requires no installer change: create the
conventional directory, place the Visual Studio project and its manifest
together, and commit both.

`vcpkg.projects` in `environment_manifest.json` is only for exceptional projects
outside the suite convention. Paths listed there are added to the automatically
discovered set; normal suite projects must not be listed individually. Keep the
vcpkg repository version and URL pinned together, restore every manifest with
the configured triplet, and verify both installation modes.

`ctl/suite/Directory.Build.props` and `Directory.Build.targets` provide the
shared MSBuild rule. When a project directory contains `vcpkg.json`, they enable
manifest mode automatically and, when `bin/vcpkg` exists, point Visual Studio at
the repository-private vcpkg tree and shared installed directory. They also set
`VCPkgLocalAppDataDisabled=true`, preventing a user-wide Scoop/classic vcpkg
integration from injecting a second, incompatible `vcpkg.props/targets` pair.
This does not uninstall or modify the user's vcpkg. Do not create suite-specific
copies of these files unless that project genuinely needs to override the
common behavior: MSBuild uses the nearest `Directory.Build.*` files, which can
shadow the central rule.

The Visual Studio project consumes local vcpkg through `Directory.Build.props`
and `Directory.Build.targets` when `bin/vcpkg` exists. Otherwise it remains
compatible with classic mode's user-wide `vcpkg integrate install`. A service
script must not run `vcpkg integrate install`; that is persistent user state and
belongs only to the classic installer.

### Rules for GMP BAT services and launchers

Every BAT entry point under an environment-managed tool area must contain the
exact audit marker and call the common guard before using Python or any managed
executable:

```bat
@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1
```

The guard selects the completed private environment when
`bin/gmp_virtual_env_installed.flag` exists; otherwise it leaves the script in
classic/system mode. It also avoids repeated activation and preserves the
caller's working directory. Scripts must follow these rules:

- Resolve repository files from `%GMP_PRO_LOCATION%`; never assume a drive
  letter or derive the repository root from a chain such as `..\..\..`.
- Quote every path. Use `cd /d` only after the guard and only when a tool truly
  requires a particular working directory.
- Use `call` when invoking another BAT file. Without it, control does not
  reliably return to the caller.
- Use the plain commands `python`, `cmake`, `doxygen`, and so on after the
  guard. The guard owns interpreter/tool selection.
- Return a non-zero exit code on failure. Check `if errorlevel 1` immediately or
  capture `%ERRORLEVEL%` before `echo`, `pause`, `cd`, or another command can
  overwrite it.
- Forward user arguments with `%*` unless the launcher deliberately defines a
  smaller public interface.
- Do not modify persistent `PATH`, proxy settings, registry values, or
  `GMP_PRO_LOCATION`; installers own persistent configuration.
- Do not install dependencies at run time and do not create, delete, or copy the
  virtual-environment completion marker.
- Do not require a `.ps1` script. GMP launch paths must work on machines where
  PowerShell script execution is disabled; use BAT plus Python.
- Make services idempotent where practical, and do not use `pause` in helpers
  called by automation. A top-level double-click launcher may pause only for a
  useful human-readable result.

A typical launcher that preserves the child exit code is:

```bat
@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

python "%GMP_PRO_LOCATION%\tools\example\example_service.py" %*
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" echo [ERROR] Example service failed with code %RESULT%.
exit /b %RESULT%
```

Avoid expanding `%ERRORLEVEL%` deep inside a parenthesized block because cmd
expands percent variables when it parses the block. Prefer an immediate
`if errorlevel 1 exit /b 1`, or capture the result on the line following the
command as shown above.

`setlocal` is correct for ordinary launchers because it prevents temporary PATH
and tool variables from leaking back to the caller. A BAT whose documented job
is to export settings to its caller is the exception: it must be invoked with
`call`, must avoid `setlocal` (or deliberately transfer values across
`endlocal`), and must state the exported variable names in its header.

The Python implementation behind a service launcher follows the same ownership
rules:

- Read the root with `os.environ["GMP_PRO_LOCATION"]` and build paths with
  `pathlib.Path`; never embed the checkout path or infer it from the current
  working directory.
- Do not activate an environment, edit `PATH`, call pip, or create the
  completion marker from the service. Its BAT launcher and the installer own
  those operations.
- Put executable behavior in a `main()` function, return an integer status, and
  terminate with `raise SystemExit(main())`. Report actionable failures to
  stderr and never convert a failed child process into exit code zero.
- Pass subprocess arguments as a list, use explicit `cwd` only when required,
  and check the return code (`check=True` or an equivalent explicit test).
- Use UTF-8 for repository text files and write generated files atomically when
  interruption could leave an invalid configuration.
- Keep repeat runs safe. Generation and registration services should update or
  replace their owned output instead of accumulating duplicate entries.
- Keep third-party imports represented in `requirements-gmp.txt`; a guarded BAT
  plus a pinned dependency is the supported execution contract.

A minimal Python service therefore looks like:

```python
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path


def main() -> int:
    root = Path(os.environ["GMP_PRO_LOCATION"]).resolve()
    command = ["example", "--input", str(root / "config" / "example.json")]
    try:
        subprocess.run(command, cwd=root, check=True)
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"[ERROR] Example service failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

Source-manager launchers require one additional rule: edit the canonical files
under `tools/facilities_generator/src_mgr/gmp_src_mgr`. Copies under individual
projects are generated artifacts. After changing the canonical templates, run
the framework distributor so every project's `gmp_src_mgr` receives the same
implementation.

Fleet discovery applies the repository's real `.gitignore` rules through
`git check-ignore`. A `gmp_src_mgr` copied into CCS `Debug`, `Release`, or any
other ignored build tree is not a deploy/generation target. Do not replace this
with a hard-coded build-directory list; adding a repository ignore rule must be
enough to exclude future generated trees. After copying the canonical scripts,
the distributor runs `gmp_generate_inc.bat` and then
`gmp_generate_src.bat` for every valid target. A copy or generation failure
returns a non-zero status, which prevents the installer completion marker from
being created.

SDPE project launchers follow the same release model. The canonical
`sdpe_edit.bat`, `sdpe_generate.bat`, `sdpe_settings.bat`, and
`sdpe_validate.bat` live under `tools/SDPE_v2/sdpe_mgr` and are distributed by
`tools/SDPE_v2/distribute_sdpe_mgr.py`. The distributor searches the repository
for `sdpe_mgr`, applies `.gitignore` through `git check-ignore`, and updates only
those four BAT files. It never overwrites `sdpe_requirement.json`, project
README files, generated headers, Matlab initialization files, or
`hardware_preset`. Project copies of the BAT files are ignored by Git and must
not be edited directly.

Standalone project `.gitignore` files are managed by
`tools/gmp_installer/distribute_project_gitignores.py`. The distributor covers
each immediate project under `csp/stm32` (excluding shared `common` and `src`),
each project under `csp/c28x_syscfg` (excluding `doc`, `src`, and metadata), and
every `ctl/suite/*/project/*` directory. Its managed block is the portable,
project-relative subset in `project_gitignore.template`; the repository root
`.gitignore` remains authoritative and is never rewritten.

The distributor is idempotent and preserves rules outside the managed markers,
so hardware- or IDE-specific additions may be kept below the block. Do not edit
the managed block in an individual project. After adding a new project or
changing the template, run:

```bat
python tools\gmp_installer\distribute_project_gitignores.py
```

Both installers run this distribution during repository setup. Commit the
resulting per-project `.gitignore` files so a project copied outside `gmp_pro`
retains protection from generated sources, IDE state, compiler output, Simulink
work folders, and local vcpkg trees.

The standalone template must not ignore distributed BAT tools such as
`gmp_generate_inc.bat`, `gmp_generate_src.bat`, `gmp_config.bat`, or the
`sdpe_mgr` launchers. Those copies are redundant inside the main GMP repository
and remain ignored only by the root `.gitignore`; once a project is copied out,
they are required source-controlled tooling that preserves its generation and
SDPE capabilities.

When a new managed BAT directory is introduced, extend
`audit_env_guards.py` so guard coverage remains enforceable. Run the audit after
adding, renaming, or moving an entry point:

```bat
python tools\gmp_installer\audit_env_guards.py
```

### Completion-marker invariant

`bin/gmp_virtual_env_installed.flag` is the only indication that the private
environment is complete. `gmp_environment.json` is diagnostic inventory and
must never be used as the completion test. Online installation and copied-bin
deployment delete any old marker before work begins; only
`environment_manager.py` creates it atomically after all mandatory validation,
CCS registration, and framework distribution steps succeed. Suite vcpkg package
restoration is mandatory only when Visual Studio C++ is detected; its deliberate
absence must not prevent hardware-only installations from becoming complete.

No other program may create the marker. A debug install using `--skip-*` is
intentionally incomplete and must finish without it. This invariant ensures
that a failed reinstall cannot leave repository BAT files selecting a partial
environment.

### Release and regression checklist

Before committing an installer or dependency change:

1. Run `install_gmp.bat --plan` and `install_gmp_virtual_env.bat --plan`; verify
   that both reports match their real behavior.
2. Test the classic installation path and build a fresh private environment on
   both a Visual Studio C++ host and a host (or test environment) without it.
3. Run `bin\python\python.exe -m pip check` and the doctor command below.
4. Open `gmp_env.bat` and check `python --version` plus every changed
   application with its `--version` or equivalent command.
5. Run `python tools\gmp_installer\audit_env_guards.py`.
6. Force or simulate a failure and verify that
   `bin/gmp_virtual_env_installed.flag` is absent. Verify that it appears only
   after a complete successful run.
7. Copy the finished `bin` folder to a repository at a different absolute path
   and run `deploy_gmp_env.bat`; this catches embedded paths and incomplete
   archives.
8. On the Visual Studio test host, build the affected Visual Studio/vcpkg
   project. On both hosts, run the affected GMP service scripts from a working
   directory other than their own.
9. Run `git diff --check`. Confirm that `bin/` remains ignored and that no
   downloaded archive, token, proxy credential, or machine-specific path is
   staged.

Doctor command:

```bat
bin\python\python.exe tools\gmp_installer\environment_manager.py doctor
```

Only publish or copy `bin` after this checklist succeeds. The recipient must
still run `deploy_gmp_env.bat`; merely copying the directory does not register
`GMP_PRO_LOCATION`, register CCS, or distribute/generate repository tools.
