#!/usr/bin/env python3
"""Build, validate, and relocate the repository-private GMP tool environment."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import urllib.parse
import urllib.request
import zipfile
from pathlib import Path, PurePosixPath


INSTALLER_DIR = Path(__file__).resolve().parent
MANIFEST_PATH = INSTALLER_DIR / "environment_manifest.json"
GMP_ROOT = Path(os.environ["GMP_PRO_LOCATION"]).resolve() if os.environ.get("GMP_PRO_LOCATION") else None
BIN_DIR = GMP_ROOT / "bin" if GMP_ROOT else None
STATE_PATH = BIN_DIR / "gmp_environment.json" if BIN_DIR else None
COMPLETION_MARKER_PATH = BIN_DIR / "gmp_virtual_env_installed.flag" if BIN_DIR else None
PROXY_CONFIG_PATH = BIN_DIR / "gmp_proxy.json" if BIN_DIR else None
PROXY_ACTIVATION_PATH = BIN_DIR / "gmp_proxy_env.bat" if BIN_DIR else None
SUITE_SIMULATE_GLOB = "ctl/suite/*/project/simulate"


class EnvironmentError(RuntimeError):
    pass


def load_manifest() -> dict:
    if GMP_ROOT is None:
        raise EnvironmentError("GMP_PRO_LOCATION is not defined")
    expected_installer = GMP_ROOT / "tools" / "gmp_installer"
    if expected_installer.resolve() != INSTALLER_DIR:
        raise EnvironmentError(
            f"GMP_PRO_LOCATION points to a different repository: {GMP_ROOT}"
        )
    with MANIFEST_PATH.open("r", encoding="utf-8") as stream:
        manifest = json.load(stream)
    if manifest.get("schema_version") != 1:
        raise EnvironmentError("Unsupported environment manifest schema")
    return manifest


def proxy_environment(mode: str, url: str | None = None) -> dict[str, str]:
    if mode == "proxy":
        if not url or any(character in url for character in "\r\n\""):
            raise EnvironmentError("The selected proxy URL is invalid")
        return {
            "GMP_PROXY_MODE": "proxy",
            "HTTP_PROXY": url,
            "HTTPS_PROXY": url,
            "ALL_PROXY": url,
            "http_proxy": url,
            "https_proxy": url,
            "all_proxy": url,
            "NO_PROXY": "localhost,127.0.0.1,::1",
            "no_proxy": "localhost,127.0.0.1,::1",
        }
    if mode == "direct":
        return {
            "GMP_PROXY_MODE": "direct",
            "HTTP_PROXY": "",
            "HTTPS_PROXY": "",
            "ALL_PROXY": "",
            "http_proxy": "",
            "https_proxy": "",
            "all_proxy": "",
            "NO_PROXY": "*",
            "no_proxy": "*",
        }
    raise EnvironmentError(f"Unsupported GMP proxy mode: {mode}")


def write_proxy_configuration() -> None:
    """Persist the user's installer proxy choice inside the portable bin tree."""
    mode = os.environ.get("GMP_INSTALLER_PROXY_MODE")
    if mode not in {"proxy", "direct"}:
        raise EnvironmentError("The GMP proxy choice has not been configured")
    url = os.environ.get("GMP_INSTALLER_PROXY_URL") if mode == "proxy" else None
    settings = proxy_environment(mode, url)
    data = {"schema_version": 1, "mode": mode}
    if url:
        data["url"] = url

    BIN_DIR.mkdir(parents=True, exist_ok=True)
    temporary_json = PROXY_CONFIG_PATH.with_suffix(".json.tmp")
    temporary_json.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary_json.replace(PROXY_CONFIG_PATH)

    lines = ["@echo off"]
    for name, value in settings.items():
        # A percent sign must be doubled when persisted in a BAT file. Proxy
        # URLs are kept inside SET quotes, so ampersands remain literal.
        lines.append(f'set "{name}={value.replace("%", "%%")}"')
    temporary_bat = PROXY_ACTIVATION_PATH.with_suffix(".bat.tmp")
    temporary_bat.write_text("\r\n".join(lines) + "\r\n", encoding="utf-8")
    temporary_bat.replace(PROXY_ACTIVATION_PATH)
    print(f"[PROXY] Saved GMP private environment proxy mode: {mode}")


def apply_persisted_proxy(env: dict[str, str]) -> None:
    if not PROXY_CONFIG_PATH.is_file():
        return
    try:
        data = json.loads(PROXY_CONFIG_PATH.read_text(encoding="utf-8"))
        env.update(proxy_environment(data["mode"], data.get("url")))
    except (KeyError, json.JSONDecodeError) as error:
        raise EnvironmentError(f"Invalid GMP proxy configuration: {PROXY_CONFIG_PATH}") from error


def display_path(path: Path) -> str:
    try:
        return str(path.relative_to(GMP_ROOT))
    except ValueError:
        return str(path)


def run(command, *, cwd=None, env=None, check=True) -> subprocess.CompletedProcess:
    printable = subprocess.list2cmdline([str(item) for item in command])
    print(f"[RUN] {printable}")
    result = subprocess.run([str(item) for item in command], cwd=cwd, env=env)
    if check and result.returncode:
        raise EnvironmentError(f"Command failed with exit code {result.returncode}: {printable}")
    return result


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def download(url: str, cache_dir: Path, expected_sha256: str | None = None) -> Path:
    cache_dir.mkdir(parents=True, exist_ok=True)
    filename = Path(urllib.parse.unquote(urllib.parse.urlparse(url).path)).name
    if not filename:
        raise EnvironmentError(f"Cannot determine download filename: {url}")
    destination = cache_dir / filename

    if destination.exists() and (not expected_sha256 or sha256_file(destination) == expected_sha256):
        print(f"[CACHE] {destination.name}")
        return destination

    temporary = destination.with_suffix(destination.suffix + ".part")
    temporary.unlink(missing_ok=True)
    request = urllib.request.Request(url, headers={"User-Agent": "GMP-Pro-Environment-Installer/1.0"})
    print(f"[DOWNLOAD] {url}")
    try:
        with urllib.request.urlopen(request, timeout=60) as response, temporary.open("wb") as output:
            total = int(response.headers.get("Content-Length", "0"))
            received = 0
            while True:
                block = response.read(1024 * 1024)
                if not block:
                    break
                output.write(block)
                received += len(block)
                if total:
                    print(f"           {received * 100 // total:3d}%", end="\r", flush=True)
        if total:
            print("           100%")
    except Exception:
        temporary.unlink(missing_ok=True)
        raise

    if expected_sha256 and sha256_file(temporary) != expected_sha256:
        temporary.unlink(missing_ok=True)
        raise EnvironmentError(f"SHA-256 mismatch for {url}")
    temporary.replace(destination)
    return destination


def _safe_archive_parts(name: str, strip_first: bool) -> tuple[str, ...] | None:
    path = PurePosixPath(name.replace("\\", "/"))
    parts = path.parts[1:] if strip_first else path.parts
    if not parts or path.is_absolute() or any(part in ("", ".", "..") for part in parts):
        return None
    return tuple(parts)


def extract_zip(archive: Path, destination: Path, strip_single_root: bool) -> None:
    staging = destination.parent / f".{destination.name}.staging-{os.getpid()}"
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)

    try:
        with zipfile.ZipFile(archive) as bundle:
            files = [entry for entry in bundle.infolist() if not entry.is_dir()]
            first_parts = {PurePosixPath(entry.filename.replace("\\", "/")).parts[0] for entry in files}
            strip_first = strip_single_root and len(first_parts) == 1
            for entry in files:
                parts = _safe_archive_parts(entry.filename, strip_first)
                if not parts:
                    continue
                output = staging.joinpath(*parts)
                output.parent.mkdir(parents=True, exist_ok=True)
                with bundle.open(entry) as source, output.open("wb") as target:
                    shutil.copyfileobj(source, target)

        if destination.exists():
            shutil.rmtree(destination)
        staging.replace(destination)
    except Exception:
        if staging.exists():
            shutil.rmtree(staging)
        raise


def application_is_ready(application: dict) -> bool:
    destination = BIN_DIR / application["destination"]
    return all((destination / relative).is_file() for relative in application["executables"])


def install_applications(manifest: dict) -> None:
    downloads = BIN_DIR / "cache" / "downloads"
    for application in manifest["applications"]:
        if application_is_ready(application):
            print(f"[OK] {application['name']} {application['version']} already installed")
            continue
        archive = download(application["url"], downloads, application.get("sha256"))
        destination = BIN_DIR / application["destination"]
        print(f"[EXTRACT] {application['name']} -> {display_path(destination)}")
        extract_zip(archive, destination, application.get("strip_single_root", False))
        if not application_is_ready(application):
            raise EnvironmentError(f"{application['name']} archive did not contain expected executables")


def install_python_packages(manifest: dict) -> None:
    python = BIN_DIR / "python" / "python.exe"
    requirements = INSTALLER_DIR / manifest["python"]["requirements"]
    env = private_environment()
    run([python, "-m", "pip", "install", "--upgrade", "pip"], env=env)
    run([python, "-m", "pip", "install", "--requirement", requirements], env=env)
    run([python, "-m", "pip", "check"], env=env)


def ensure_vcpkg_cache_directories() -> None:
    """Create every directory exported to vcpkg as a writable cache."""
    for directory in (
        BIN_DIR / "cache" / "vcpkg-downloads",
        BIN_DIR / "cache" / "vcpkg-archives",
    ):
        directory.mkdir(parents=True, exist_ok=True)


def discover_vcpkg_projects(manifest: dict) -> list[Path]:
    """Return convention-based suite manifests plus explicit extra projects."""
    projects: set[Path] = set()

    for simulate in sorted(GMP_ROOT.glob(SUITE_SIMULATE_GLOB)):
        if not simulate.is_dir():
            continue
        manifest_path = simulate / "vcpkg.json"
        has_visual_studio_project = any(simulate.glob("*.vcxproj")) or any(simulate.glob("*.sln"))
        if has_visual_studio_project and not manifest_path.is_file():
            relative = simulate.relative_to(GMP_ROOT).as_posix()
            raise EnvironmentError(
                f"Visual Studio simulate project is missing its vcpkg manifest: {relative}/vcpkg.json"
            )
        if manifest_path.is_file():
            projects.add(simulate.resolve())

    for relative in manifest["vcpkg"].get("projects", []):
        project = (GMP_ROOT / relative).resolve()
        try:
            project.relative_to(GMP_ROOT)
        except ValueError as error:
            raise EnvironmentError(f"vcpkg project escapes the GMP repository: {relative}") from error
        if not (project / "vcpkg.json").is_file():
            raise EnvironmentError(f"vcpkg manifest is missing: {relative}/vcpkg.json")
        projects.add(project)

    return sorted(projects, key=lambda path: path.relative_to(GMP_ROOT).as_posix().lower())


def install_vcpkg(manifest: dict) -> None:
    ensure_vcpkg_cache_directories()
    config = manifest["vcpkg"]
    vcpkg_root = BIN_DIR / "vcpkg"
    executable = vcpkg_root / "vcpkg.exe"
    if executable.is_file():
        print(f"[OK] vcpkg {config['repository_version']} already installed")
        return

    archive = download(config["repository_url"], BIN_DIR / "cache" / "downloads", config.get("sha256"))
    print(f"[EXTRACT] vcpkg -> {display_path(vcpkg_root)}")
    extract_zip(archive, vcpkg_root, True)

    metadata = vcpkg_root / "scripts" / "vcpkg-tool-metadata.txt"
    if not metadata.is_file():
        raise EnvironmentError("vcpkg tool metadata is missing from the repository archive")
    values = {}
    for line in metadata.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key.strip()] = value.strip()
    release = values.get("VCPKG_TOOL_RELEASE_TAG")
    if not release:
        raise EnvironmentError("Cannot determine the matching vcpkg executable version")
    tool_url = f"https://github.com/microsoft/vcpkg-tool/releases/download/{release}/vcpkg.exe"
    cached_tool = download(tool_url, BIN_DIR / "cache" / "downloads")
    shutil.copy2(cached_tool, executable)
    (vcpkg_root / "vcpkg.disable-metrics").touch()
    run([executable, "version", "--disable-metrics"], env=private_environment())


def private_environment() -> dict[str, str]:
    env = os.environ.copy()
    env.update(
        {
            "GMP_PRO_LOCATION": str(GMP_ROOT),
            "GMP_BIN": str(BIN_DIR),
            "GMP_ENV_ACTIVE": str(GMP_ROOT),
            "GMP_ENV_MODE": "virtual",
            "PYTHONHOME": str(BIN_DIR / "python"),
            "PYTHONNOUSERSITE": "1",
            "PYTHONUTF8": "1",
            "PIP_DISABLE_PIP_VERSION_CHECK": "1",
            "VCPKG_ROOT": str(BIN_DIR / "vcpkg"),
            "VCPKG_DOWNLOADS": str(BIN_DIR / "cache" / "vcpkg-downloads"),
            "VCPKG_DEFAULT_BINARY_CACHE": str(BIN_DIR / "cache" / "vcpkg-archives"),
            "VCPKG_INSTALLED_DIR": str(BIN_DIR / "vcpkg_installed" / "x64-windows"),
            "VCPKG_DEFAULT_TRIPLET": "x64-windows",
            "VCPKG_FEATURE_FLAGS": "manifests,binarycaching",
            "CMAKE_TOOLCHAIN_FILE": str(BIN_DIR / "vcpkg" / "scripts" / "buildsystems" / "vcpkg.cmake"),
        }
    )
    path_entries = [
        BIN_DIR / "python",
        BIN_DIR / "python" / "Scripts",
        BIN_DIR / "apps" / "git" / "cmd",
        BIN_DIR / "apps" / "git" / "mingw64" / "bin",
        BIN_DIR / "apps" / "cmake" / "bin",
        BIN_DIR / "apps" / "ninja",
        BIN_DIR / "apps" / "doxygen",
        BIN_DIR / "apps" / "graphviz" / "bin",
        BIN_DIR / "vcpkg",
    ]
    env["PATH"] = os.pathsep.join(str(path) for path in path_entries) + os.pathsep + env.get("PATH", "")
    apply_persisted_proxy(env)
    return env


def find_visual_studio_installation() -> Path | None:
    """Find a Visual Studio installation with the x64 C++ workload."""
    program_files_x86 = os.environ.get("ProgramFiles(x86)")
    if not program_files_x86:
        return None
    vswhere = Path(program_files_x86) / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
    if not vswhere.is_file():
        return None
    common_query = [
        str(vswhere),
        "-latest",
        "-products",
        "*",
        "-requires",
        "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
        "-property",
        "installationPath",
    ]
    query = subprocess.run(
        common_query[:2] + ["-version", "[17.0,18.0)"] + common_query[2:],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    installation = query.stdout.strip()
    if not installation:
        query = subprocess.run(
            common_query,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        installation = query.stdout.strip()
    if not installation:
        return None
    installation_path = Path(installation)
    if not (installation_path / "Common7" / "Tools" / "VsDevCmd.bat").is_file():
        return None
    return installation_path


def visual_studio_is_available() -> bool:
    return find_visual_studio_installation() is not None


def print_optional_visual_studio_warning() -> None:
    print("[OPTIONAL] Visual Studio C++ tools were not found.")
    print("           Skipping suite vcpkg package restoration; hardware/GMP tools remain available.")
    print("           Install the VS Desktop development with C++ workload later, then run repair_gmp_vcpkg.bat.")


def visual_studio_environment(base_env: dict[str, str]) -> dict[str, str]:
    installation = find_visual_studio_installation()
    if installation is None:
        raise EnvironmentError(
            "Visual Studio C++ tools are unavailable; install the Desktop development with C++ workload to use suite simulation packages"
        )
    developer_batch = installation / "Common7" / "Tools" / "VsDevCmd.bat"
    result = subprocess.run(
        [
            os.environ.get("COMSPEC", "cmd.exe"),
            "/d",
            "/c",
            "call",
            str(developer_batch),
            "-no_logo",
            "-arch=x64",
            "-host_arch=x64",
            ">nul",
            "&&",
            "set",
        ],
        capture_output=True,
        text=True,
        encoding="mbcs",
        errors="replace",
        env=base_env,
    )
    if result.returncode:
        detail = (result.stderr or result.stdout).strip()
        raise EnvironmentError(
            "Visual Studio developer environment initialization failed"
            + (f": {detail}" if detail else "")
        )
    activated = base_env.copy()
    for line in result.stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            activated[key] = value
    # VsDevCmd may advertise Visual Studio's bundled vcpkg. Repository-private
    # builds must retain the GMP roots/caches/proxy selected before VS setup.
    protected_names = (
        "GMP_PRO_LOCATION",
        "GMP_BIN",
        "GMP_ENV_ACTIVE",
        "GMP_ENV_MODE",
        "PYTHONHOME",
        "PYTHONNOUSERSITE",
        "PYTHONUTF8",
        "PIP_DISABLE_PIP_VERSION_CHECK",
        "VCPKG_ROOT",
        "VCPKG_DOWNLOADS",
        "VCPKG_DEFAULT_BINARY_CACHE",
        "VCPKG_INSTALLED_DIR",
        "VCPKG_DEFAULT_TRIPLET",
        "VCPKG_FEATURE_FLAGS",
        "CMAKE_TOOLCHAIN_FILE",
        "GMP_PROXY_MODE",
        "HTTP_PROXY",
        "HTTPS_PROXY",
        "ALL_PROXY",
        "http_proxy",
        "https_proxy",
        "all_proxy",
        "NO_PROXY",
        "no_proxy",
    )
    for name in protected_names:
        if name in base_env:
            activated[name] = base_env[name]
    return activated


def install_vcpkg_projects(manifest: dict) -> None:
    ensure_vcpkg_cache_directories()
    executable = BIN_DIR / "vcpkg" / "vcpkg.exe"
    if not executable.is_file():
        raise EnvironmentError("vcpkg.exe is missing")
    env = visual_studio_environment(private_environment())
    triplet = manifest["vcpkg"]["triplet"]
    install_root = BIN_DIR / "vcpkg_installed" / triplet
    install_root.mkdir(parents=True, exist_ok=True)
    projects = discover_vcpkg_projects(manifest)
    if not projects:
        raise EnvironmentError(f"No vcpkg projects were found by {SUITE_SIMULATE_GLOB}")
    for project in projects:
        relative = project.relative_to(GMP_ROOT).as_posix()
        print(f"[INSTALL] Restoring vcpkg manifest: {relative}")
        run(
            [
                executable,
                "install",
                f"--x-manifest-root={project}",
                f"--x-install-root={install_root}",
                f"--triplet={triplet}",
                "--disable-metrics",
            ],
            cwd=project,
            env=env,
        )


def configure_repository() -> None:
    python = BIN_DIR / "python" / "python.exe"
    env = private_environment()
    facilities = GMP_ROOT / "tools" / "facilities_generator"
    run([python, facilities / "gmp_fac_install_ccs_product.py"], cwd=facilities, env=env)
    run([python, facilities / "gmp_fac_generate_cfg_json.py"], cwd=facilities, env=env)
    source_manager = facilities / "src_mgr"
    run([python, source_manager / "framework_distribute_tools_v3.py"], cwd=source_manager, env=env)
    sdpe = GMP_ROOT / "tools" / "SDPE_v2"
    run([python, sdpe / "distribute_sdpe_mgr.py"], cwd=sdpe, env=env)
    run([python, INSTALLER_DIR / "distribute_project_gitignores.py"], cwd=INSTALLER_DIR, env=env)


def write_state(
    manifest: dict,
    mode: str,
    *,
    visual_studio_cpp: bool,
    vcpkg_packages_restored: bool,
) -> None:
    python = BIN_DIR / "python" / "python.exe"
    freeze = subprocess.run(
        [str(python), "-m", "pip", "freeze", "--all"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=private_environment(),
    )
    state = {
        "schema_version": 1,
        "environment_version": manifest["environment_version"],
        "deployed_by": mode,
        "python_version": manifest["python"]["version"],
        "applications": {item["name"]: item["version"] for item in manifest["applications"]},
        "vcpkg_repository_version": manifest["vcpkg"]["repository_version"],
        "vcpkg_projects": [
            project.relative_to(GMP_ROOT).as_posix()
            for project in discover_vcpkg_projects(manifest)
        ],
        "capabilities": {
            "visual_studio_cpp": visual_studio_cpp,
            "vcpkg_packages_restored": vcpkg_packages_restored,
        },
        "python_packages": freeze.stdout.splitlines() if freeze.returncode == 0 else [],
        "portable": True,
    }
    BIN_DIR.mkdir(parents=True, exist_ok=True)
    STATE_PATH.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def remove_completion_marker() -> None:
    COMPLETION_MARKER_PATH.unlink(missing_ok=True)


def create_completion_marker() -> None:
    temporary = COMPLETION_MARKER_PATH.with_suffix(".flag.tmp")
    temporary.write_text("GMP virtual environment installation completed.\n", encoding="ascii")
    temporary.replace(COMPLETION_MARKER_PATH)


def expected_vcpkg_ports(manifest: dict) -> set[str]:
    ports = set()
    for project in discover_vcpkg_projects(manifest):
        data = json.loads((project / "vcpkg.json").read_text(encoding="utf-8"))
        for dependency in data.get("dependencies", []):
            ports.add(dependency if isinstance(dependency, str) else dependency["name"])
    return ports


def doctor(
    manifest: dict,
    *,
    require_vcpkg_packages: bool | None = None,
    require_completion_marker: bool = True,
) -> bool:
    if require_vcpkg_packages is None:
        require_vcpkg_packages = visual_studio_is_available()
    failures = []
    if require_completion_marker and not COMPLETION_MARKER_PATH.is_file():
        failures.append(f"missing completion marker: {display_path(COMPLETION_MARKER_PATH)}")
    python = BIN_DIR / "python" / "python.exe"
    if not python.is_file():
        failures.append(f"missing {display_path(python)}")
    else:
        version = subprocess.run([str(python), "--version"], capture_output=True, text=True, env=private_environment())
        if manifest["python"]["version"] not in (version.stdout + version.stderr):
            failures.append(f"expected Python {manifest['python']['version']}, got {(version.stdout + version.stderr).strip()}")
        import_code = "; ".join(f"import {name}" for name in manifest["python"]["imports"])
        imports = subprocess.run([str(python), "-c", import_code], env=private_environment())
        if imports.returncode:
            failures.append("one or more required Python packages cannot be imported")

    for application in manifest["applications"]:
        if not application_is_ready(application):
            failures.append(f"missing or incomplete application: {application['name']}")
    if not (BIN_DIR / "vcpkg" / "vcpkg.exe").is_file():
        failures.append("missing vcpkg/vcpkg.exe")

    if require_vcpkg_packages:
        triplet = manifest["vcpkg"]["triplet"]
        share = BIN_DIR / "vcpkg_installed" / triplet / triplet / "share"
        for port in expected_vcpkg_ports(manifest):
            if not (share / port).is_dir():
                failures.append(f"missing vcpkg package for {triplet}: {port}")

    if failures:
        print("[DOCTOR] Environment validation failed:")
        for failure in failures:
            print(f"  - {failure}")
        return False
    print("[DOCTOR] GMP private environment is complete and relocatable.")
    return True


def print_plan(manifest: dict) -> None:
    print(f"GMP root: {GMP_ROOT}")
    print(f"Private environment: {BIN_DIR}")
    print(f"Python: {manifest['python']['version']}")
    print("Applications:")
    for item in manifest["applications"]:
        print(f"  - {item['name']} {item['version']} -> bin/{item['destination']}")
    print(f"vcpkg: {manifest['vcpkg']['repository_version']} ({manifest['vcpkg']['triplet']})")
    print(
        "Visual Studio C++: "
        + ("available; suite packages will be restored" if visual_studio_is_available() else "not detected; optional suite package restore will be skipped")
    )
    print(f"  - automatic rule: {SUITE_SIMULATE_GLOB}/vcpkg.json")
    for project in discover_vcpkg_projects(manifest):
        relative = project.relative_to(GMP_ROOT).as_posix()
        print(f"  - manifest: {relative}/vcpkg.json")


def print_vcpkg_project_paths(manifest: dict) -> None:
    """Print machine-readable repository-relative paths, one per line."""
    for project in discover_vcpkg_projects(manifest):
        print(project.relative_to(GMP_ROOT).as_posix())


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    online = subparsers.add_parser("online", help="download and build bin from zero")
    online.add_argument("--skip-vcpkg-packages", action="store_true")
    online.add_argument("--skip-project-setup", action="store_true")
    deploy = subparsers.add_parser("deploy", help="validate and deploy a copied bin folder")
    deploy.add_argument("--skip-project-setup", action="store_true")
    subparsers.add_parser("doctor", help="validate the current bin folder")
    subparsers.add_parser("plan", help="print the pinned environment plan without changing files")
    subparsers.add_parser(
        "list-vcpkg-projects",
        help="print convention-based and explicitly configured vcpkg project paths",
    )
    subparsers.add_parser(
        "configure-proxy",
        help="persist the selected proxy mode for future GMP and Visual Studio processes",
    )
    subparsers.add_parser(
        "restore-vcpkg",
        help="download/restore all discovered vcpkg tools and project dependencies",
    )
    subparsers.add_parser(
        "check-visual-studio",
        help="return success only when the Visual Studio x64 C++ workload is available",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    try:
        manifest = load_manifest()
        if args.command == "plan":
            print_plan(manifest)
            return 0
        if args.command == "list-vcpkg-projects":
            print_vcpkg_project_paths(manifest)
            return 0
        if args.command == "configure-proxy":
            write_proxy_configuration()
            return 0
        if args.command == "check-visual-studio":
            if visual_studio_is_available():
                print("[OK] Visual Studio x64 C++ tools are available.")
                return 0
            print_optional_visual_studio_warning()
            return 2
        if args.command == "restore-vcpkg":
            if not visual_studio_is_available():
                print_optional_visual_studio_warning()
                return 2
            install_vcpkg(manifest)
            install_vcpkg_projects(manifest)
            return 0 if doctor(manifest, require_completion_marker=False) else 1
        if args.command == "doctor":
            return 0 if doctor(manifest) else 1
        if args.command == "online":
            write_proxy_configuration()
            remove_completion_marker()
            install_python_packages(manifest)
            install_applications(manifest)
            install_vcpkg(manifest)
            visual_studio_cpp = visual_studio_is_available()
            vcpkg_packages_restored = False
            if not args.skip_vcpkg_packages and visual_studio_cpp:
                install_vcpkg_projects(manifest)
                vcpkg_packages_restored = True
            elif not args.skip_vcpkg_packages:
                print_optional_visual_studio_warning()
            if not args.skip_project_setup:
                configure_repository()
            write_state(
                manifest,
                "online",
                visual_studio_cpp=visual_studio_cpp,
                vcpkg_packages_restored=vcpkg_packages_restored,
            )
            if not doctor(
                manifest,
                require_vcpkg_packages=vcpkg_packages_restored,
                require_completion_marker=False,
            ):
                return 1
            if args.skip_vcpkg_packages or args.skip_project_setup:
                print("\n[WARNING] Partial installation completed; completion marker was not created.")
                return 0
            create_completion_marker()
        elif args.command == "deploy":
            write_proxy_configuration()
            remove_completion_marker()
            visual_studio_cpp = visual_studio_is_available()
            if not visual_studio_cpp:
                print_optional_visual_studio_warning()
            if not doctor(
                manifest,
                require_vcpkg_packages=visual_studio_cpp,
                require_completion_marker=False,
            ):
                return 1
            if not args.skip_project_setup:
                configure_repository()
            write_state(
                manifest,
                "portable-copy",
                visual_studio_cpp=visual_studio_cpp,
                vcpkg_packages_restored=visual_studio_cpp,
            )
            if args.skip_project_setup:
                print("\n[WARNING] Partial deployment completed; completion marker was not created.")
                return 0
            if not doctor(
                manifest,
                require_vcpkg_packages=visual_studio_cpp,
                require_completion_marker=False,
            ):
                return 1
            create_completion_marker()
        print("\n[SUCCESS] GMP private environment is ready. Run gmp_env.bat to enter it.")
        return 0
    except (EnvironmentError, OSError, json.JSONDecodeError, zipfile.BadZipFile) as error:
        print(f"\n[ERROR] {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
