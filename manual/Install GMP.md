# Install the GMP development environment

**English** | [简体中文](Install%20GMP_CN.md)

Use Windows x64 and a GMP checkout whose full path contains neither spaces nor non-ASCII characters. Run the commands from the repository root. Visual Studio with x64 C++ support is optional for hardware projects. C2000 builds require a suitable TI Code Composer Studio, C2000Ware and SysConfig installation.

| Entry point | Effect |
| --- | --- |
| install_gmp_virtual_env.bat | Installs the pinned tools under bin. Registers GMP_PRO_LOCATION for the current user. Open the environment with gmp_env.bat. |
| install_gmp.bat | Uses the same tools under bin and also adds their directories to the user's PATH. Enables user-wide vcpkg integration if Visual Studio C++ is available. Existing Scoop installations are managed separately. |
| deploy_gmp_env.bat | Validates a complete bin copied from another checkout, selects local proxy settings, refreshes repository files and registers the new root. |

~~~bat
install_gmp_virtual_env.bat --plan
install_gmp_virtual_env.bat --download-only
install_gmp_virtual_env.bat
~~~

All three entry points support --plan. It prints the tools and operations without downloading, generating project files or setting persistent user variables. Both online installers support --download-only, which fetches and verifies the locked artifacts into bin\cache without installing tools or changing the completion state. Set GMP_INSTALLER_NO_PAUSE=1 for unattended calls. Set GMP_INSTALLER_PROXY_CHOICE=N for a direct connection, or set it to Y and supply GMP_INSTALLER_PROXY_URL. The selected proxy is stored under bin for subsequent GMP tools.

## Installation sequence

1. Read the [tool manifest](../tools/gmp_installer/environment_manifest.json) and the [Windows x64 Python lock](../tools/gmp_installer/requirements-gmp-windows.lock). They pin CPython, pip and its packages, MinGit, CMake, Ninja, Doxygen, Graphviz, the vcpkg executable and its ports.
2. Verify SHA-256 for every cached or newly downloaded installer, archive and wheel before installing. New files are staged and promoted to the cache after verification.
3. Install CPython into bin\python if the pinned version is missing. Install Python wheels from the verified cache with pip's hash checking. Validate the installed application versions before reuse or after extraction.
4. If Visual Studio C++ is detected, restore one shared x64-windows vcpkg manifest composed from suite and Simulink dependencies. vcpkg can fetch and verify auxiliary tools and port sources at this stage.
5. Validate the environment. Generate CCS metadata and facility configuration, distribute and run the source manager, distribute SDPE launchers, and merge standalone project ignore rules.
6. Save the installation inventory and publish GMP_PRO_LOCATION. Mark the environment complete only after required steps succeed. Private installations and portable deployments use gmp_virtual_env_installed.flag. System installations use gmp_system_env_installed.flag.

A copied-bin deployment uses the checked local tools instead of downloading or installing them. If the destination has Visual Studio C++, copy a bin that already contains the restored simulation dependencies. Diagnostic runs with --skip-project-setup or --skip-vcpkg-packages finish without a completion marker.

~~~bat
gmp_env.bat python --version
gmp_env.bat cmake --version
gmp_env.bat bin\python\python.exe tools\gmp_installer\environment_manager.py doctor
~~~

Run repair_gmp_vcpkg.bat after adding the Visual Studio C++ workload or updating the project dependency manifests. Run configure_gmp_proxy.bat to change the stored proxy choice. See the [installer maintenance guide](../tools/gmp_installer/README.md) and the separate [GMP MATLAB/Simulink library guide](../slib/readme.md).
