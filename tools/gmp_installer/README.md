# GMP installer maintenance

**English** | [简体中文](README_CN.md)

See the [GMP installation guide](../../manual/Install%20GMP.md) for the commands,
installer actions, and Windows settings each mode writes.

Both online modes prepare and hash-check all locked artifacts before installing
the same tools under `bin`. The system entry also publishes the pinned tools
on the user PATH and runs user-wide vcpkg integration if Visual Studio C++ is
present. Existing Scoop installs are managed separately.

`environment_manifest.json` defines the Windows installer, ZIP and vcpkg
artifacts. `requirements-gmp-windows.lock` defines the Windows x64 wheels and
pip. `--plan` previews actions without writes. `--download-only` verifies and
prepares the cache without installing.

The common environment guard checks the private or system completion marker
before selecting a managed environment. It also checks that the pinned Python
executable is present, preserves the caller's directory and avoids reactivation
in an existing GMP shell. To audit marker coverage, run
`python tools/gmp_installer/audit_env_guards.py`.

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

Both Windows installers consume `requirements-gmp-windows.lock`. When changing
Python or adding packages, regenerate the lock with the Windows x64 wheel URL
and SHA-256 for pip, direct dependencies and transitive dependencies. Update
the Python version and installer artifact in `environment_manifest.json` if
Python itself changes.

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

Use an HTTPS release URL and the verified SHA-256 for the ZIP. Review its
archive layout before selecting `strip_single_root`. The installer checks the
listed executables and their reported version. Set `version_arg` if the
executable uses a switch other than `--version`. Add its directory to
`tool_directories()` in `environment_manager.py` and to the PATH in
`activate_env.bat`.

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

Use `vcpkg.projects` in `environment_manifest.json` for exceptional projects
outside the suite convention. The installer combines their declared dependencies
with the discovered suites into a single manifest under `bin/vcpkg_manifest`.
Conflicting package declarations and project-specific version, registry,
feature or override settings require reconciliation before they can be shared.
Maintain the pinned versions, URLs and hashes of both the vcpkg tool and ports.

`ctl/suite/Directory.Build.props` and `Directory.Build.targets` enable manifest
mode and point completed GMP environments to the shared packages. They disable
automatic manifest restore in that configuration, so building one project
cannot reconcile the shared library directory against its own manifest.
Suite-local `Directory.Build.*` files must import the common rules. If ports
are missing or have changed, run `repair_gmp_vcpkg.bat`.

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

The private and system markers are `gmp_virtual_env_installed.flag` and
`gmp_system_env_installed.flag` in `bin`. The inventory in
`gmp_environment.json` records versions and capabilities. The online installer
verifies the entire cache before invalidating the prior markers and starting
reinstallation. Copied-bin deployment invalidates copied markers before
validation. The installer creates a new marker after successful setup.
Diagnostic `--skip-*` runs finish without an active marker. The repair entry
updates the existing capability inventory after successfully restoring vcpkg.

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
