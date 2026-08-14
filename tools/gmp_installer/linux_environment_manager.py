#!/usr/bin/env python3
"""Install and validate the repository-private GMP Linux environment."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import urllib.request
import zipfile
from pathlib import Path


INSTALLER_DIR = Path(__file__).resolve().parent
GMP_ROOT = INSTALLER_DIR.parents[1]
LINUX_ROOT = GMP_ROOT / "bin" / "linux"
VENV_DIR = LINUX_ROOT / "venv"
VCPKG_ROOT = LINUX_ROOT / "vcpkg"
VCPKG_INSTALL_ROOT = LINUX_ROOT / "vcpkg_installed"
CACHE_ROOT = LINUX_ROOT / "cache"
ACTIVATION_PATH = LINUX_ROOT / "activate_gmp.sh"
STATE_PATH = LINUX_ROOT / "gmp_environment.json"
COMPLETION_MARKER_PATH = LINUX_ROOT / "gmp_virtual_env_installed.flag"
WINDOWS_MANIFEST_PATH = INSTALLER_DIR / "environment_manifest.json"

HEADLESS_REQUIREMENTS = INSTALLER_DIR / "requirements-gmp-linux.txt"
GUI_REQUIREMENTS = INSTALLER_DIR / "requirements-gmp-linux-gui.txt"
HEADLESS_IMPORTS = ("jinja2", "matplotlib", "numpy", "serial", "symengine")
GUI_IMPORTS = ("PyQt5", "PyQt6", "PySide6", "pyqtgraph")


class LinuxEnvironmentError(RuntimeError):
    """Raised when the private Linux environment cannot be prepared safely."""


def run(command: list[str | Path], *, cwd: Path | None = None, env: dict[str, str] | None = None) -> None:
    printable = " ".join(str(item) for item in command)
    print(f"[RUN] {printable}", flush=True)
    result = subprocess.run([str(item) for item in command], cwd=cwd, env=env)
    if result.returncode != 0:
        raise LinuxEnvironmentError(f"Command failed with exit code {result.returncode}: {printable}")


def target_triplet(machine: str | None = None) -> str:
    """Return the vcpkg native Linux triplet for the current machine."""
    normalized = (machine or platform.machine()).lower()
    mapping = {
        "x86_64": "x64-linux",
        "amd64": "x64-linux",
        "aarch64": "arm64-linux",
        "arm64": "arm64-linux",
    }
    try:
        return mapping[normalized]
    except KeyError as error:
        raise LinuxEnvironmentError(f"Unsupported GMP Linux architecture: {normalized}") from error


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def download(url: str, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_file():
        print(f"[CACHE] {destination}")
        return
    temporary = destination.with_suffix(destination.suffix + ".part")
    temporary.unlink(missing_ok=True)
    request = urllib.request.Request(url, headers={"User-Agent": "GMP-Linux-Environment-Installer/1.0"})
    print(f"[DOWNLOAD] {url}")
    try:
        with urllib.request.urlopen(request, timeout=120) as response, temporary.open("wb") as output:
            shutil.copyfileobj(response, output, 1024 * 1024)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise
    temporary.replace(destination)


def load_windows_manifest() -> dict:
    return json.loads(WINDOWS_MANIFEST_PATH.read_text(encoding="utf-8"))


def discover_vcpkg_projects() -> list[Path]:
    projects = {
        path.parent.resolve()
        for path in GMP_ROOT.glob("ctl/suite/*/project/simulate/vcpkg.json")
        if path.is_file()
    }
    for relative in load_windows_manifest()["vcpkg"].get("projects", []):
        project = (GMP_ROOT / relative).resolve()
        if not (project / "vcpkg.json").is_file():
            raise LinuxEnvironmentError(f"Missing vcpkg manifest: {project / 'vcpkg.json'}")
        projects.add(project)
    return sorted(projects, key=lambda path: path.relative_to(GMP_ROOT).as_posix().lower())


def build_aggregate_vcpkg_manifest(projects: list[Path]) -> dict:
    """Merge all project manifests before using one shared install root."""
    dependencies: dict[str, str | dict] = {}
    overrides: dict[str, dict] = {}
    baseline: str | None = None
    for project in projects:
        document = json.loads((project / "vcpkg.json").read_text(encoding="utf-8"))
        project_name = project.relative_to(GMP_ROOT).as_posix()
        candidate_baseline = document.get("builtin-baseline")
        if candidate_baseline:
            if baseline is not None and baseline != candidate_baseline:
                raise LinuxEnvironmentError(f"Conflicting vcpkg baselines in {project_name}")
            baseline = candidate_baseline
        for dependency in document.get("dependencies", []):
            normalized = dependency["name"] if isinstance(dependency, dict) and set(dependency) == {"name"} else dependency
            name = normalized if isinstance(normalized, str) else normalized.get("name")
            if not name:
                raise LinuxEnvironmentError(f"Invalid vcpkg dependency in {project_name}")
            if name in dependencies and dependencies[name] != normalized:
                raise LinuxEnvironmentError(f"Conflicting vcpkg dependency declarations for {name}")
            dependencies[name] = normalized
        for override in document.get("overrides", []):
            name = override.get("name") if isinstance(override, dict) else None
            if not name:
                raise LinuxEnvironmentError(f"Invalid vcpkg override in {project_name}")
            if name in overrides and overrides[name] != override:
                raise LinuxEnvironmentError(f"Conflicting vcpkg override declarations for {name}")
            overrides[name] = override

    aggregate = {
        "name": "gmp-linux-environment-dependencies",
        "version-string": "1.0.0",
        "dependencies": [dependencies[name] for name in sorted(dependencies)],
    }
    if baseline:
        aggregate["builtin-baseline"] = baseline
    if overrides:
        aggregate["overrides"] = [overrides[name] for name in sorted(overrides)]
    return aggregate


def install_python_packages(with_gui: bool) -> None:
    requirements = GUI_REQUIREMENTS if with_gui else HEADLESS_REQUIREMENTS
    run([sys.executable, "-m", "pip", "install", "--disable-pip-version-check", "-r", requirements])
    run([sys.executable, "-m", "pip", "check"])


def install_vcpkg() -> None:
    manifest = load_windows_manifest()["vcpkg"]
    archive = CACHE_ROOT / Path(manifest["repository_url"]).name
    download(manifest["repository_url"], archive)

    version_marker = VCPKG_ROOT / ".gmp-version"
    expected_version = str(manifest["repository_version"])
    needs_bootstrap = False
    if not version_marker.is_file() or version_marker.read_text(encoding="utf-8").strip() != expected_version:
        extraction = CACHE_ROOT / "vcpkg-extract"
        if extraction.exists():
            shutil.rmtree(extraction)
        extraction.mkdir(parents=True)
        with zipfile.ZipFile(archive) as package:
            package.extractall(extraction)
        roots = [path for path in extraction.iterdir() if path.is_dir()]
        if len(roots) != 1:
            raise LinuxEnvironmentError("The vcpkg archive does not contain one root directory")
        if VCPKG_ROOT.exists():
            shutil.rmtree(VCPKG_ROOT)
        roots[0].replace(VCPKG_ROOT)
        shutil.rmtree(extraction)
        version_marker.write_text(expected_version + "\n", encoding="utf-8")
        needs_bootstrap = True

    bootstrap = VCPKG_ROOT / "bootstrap-vcpkg.sh"
    vcpkg_executable = VCPKG_ROOT / "vcpkg"
    if needs_bootstrap or not vcpkg_executable.is_file():
        bootstrap.chmod(bootstrap.stat().st_mode | 0o111)
        run([bootstrap, "-disableMetrics"], cwd=VCPKG_ROOT)
    else:
        print(f"[CACHE] {vcpkg_executable}")

    projects = discover_vcpkg_projects()
    aggregate_root = CACHE_ROOT / "vcpkg-manifest"
    aggregate_root.mkdir(parents=True, exist_ok=True)
    (aggregate_root / "vcpkg.json").write_text(
        json.dumps(build_aggregate_vcpkg_manifest(projects), indent=2) + "\n", encoding="utf-8"
    )
    triplet = target_triplet()
    environment = os.environ.copy()
    environment["VCPKG_DOWNLOADS"] = str(CACHE_ROOT / "vcpkg-downloads")
    environment["VCPKG_DEFAULT_BINARY_CACHE"] = str(CACHE_ROOT / "vcpkg-binary")
    environment["VCPKG_FORCE_SYSTEM_BINARIES"] = "1"
    Path(environment["VCPKG_DOWNLOADS"]).mkdir(parents=True, exist_ok=True)
    Path(environment["VCPKG_DEFAULT_BINARY_CACHE"]).mkdir(parents=True, exist_ok=True)
    run(
        [
            vcpkg_executable,
            "install",
            f"--x-manifest-root={aggregate_root}",
            f"--x-install-root={VCPKG_INSTALL_ROOT}",
            f"--triplet={triplet}",
            "--disable-metrics",
        ],
        cwd=aggregate_root,
        env=environment,
    )


def write_activation_script() -> None:
    content = """#!/usr/bin/env bash
# Generated by the GMP Linux installer. Source this file; do not execute it.
if [[ -z "${BASH_VERSION:-}" ]]; then
    echo "[GMP] activate_gmp.sh requires Bash." >&2
    return 1 2>/dev/null || exit 1
fi
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "[GMP] Use: source bin/linux/activate_gmp.sh" >&2
    exit 1
fi

_gmp_linux_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export GMP_PRO_LOCATION="$(cd -- "${_gmp_linux_dir}/../.." && pwd -P)"
export GMP_LINUX_ENV="${_gmp_linux_dir}"
export VIRTUAL_ENV="${_gmp_linux_dir}/venv"
export VCPKG_ROOT="${_gmp_linux_dir}/vcpkg"
export VCPKG_INSTALLED_DIR="${_gmp_linux_dir}/vcpkg_installed"
export VCPKG_DOWNLOADS="${_gmp_linux_dir}/cache/vcpkg-downloads"
export VCPKG_DEFAULT_BINARY_CACHE="${_gmp_linux_dir}/cache/vcpkg-binary"
export VCPKG_FORCE_SYSTEM_BINARIES=1
_gmp_triplet="$("${VIRTUAL_ENV}/bin/python" "${GMP_PRO_LOCATION}/tools/gmp_installer/linux_environment_manager.py" triplet)" || return 1
export VCPKG_DEFAULT_TRIPLET="${_gmp_triplet}"
export CMAKE_TOOLCHAIN_FILE="${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
export CMAKE_PREFIX_PATH="${VCPKG_INSTALLED_DIR}/${_gmp_triplet}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
case ":${PATH}:" in
    *":${VIRTUAL_ENV}/bin:"*) ;;
    *) export PATH="${VIRTUAL_ENV}/bin:${VCPKG_ROOT}:${PATH}" ;;
esac
unset _gmp_linux_dir _gmp_triplet
echo "[GMP] Linux environment active: ${GMP_PRO_LOCATION}"
"""
    ACTIVATION_PATH.parent.mkdir(parents=True, exist_ok=True)
    ACTIVATION_PATH.write_text(content, encoding="utf-8", newline="\n")
    ACTIVATION_PATH.chmod(ACTIVATION_PATH.stat().st_mode | 0o111)


def validate_imports(with_gui: bool) -> None:
    for module in HEADLESS_IMPORTS + (GUI_IMPORTS if with_gui else ()):
        __import__(module)
        print(f"[OK] Python import: {module}")


def validate_host_tools() -> None:
    """Verify the native tools required by Linux builds and vcpkg."""
    for executable, version_argument in (
        ("cmake", "--version"),
        ("ninja", "--version"),
        ("git", "--version"),
        ("g++", "--version"),
        ("bash", "--version"),
    ):
        resolved = shutil.which(executable)
        if resolved is None:
            raise LinuxEnvironmentError(f"Missing required Linux host tool: {executable}")
        print(f"[OK] Linux host tool: {executable} ({resolved})")
        run([resolved, version_argument])


def validate_repository() -> None:
    json.loads((GMP_ROOT / "tools/facilities_generator/src_mgr/gmp_framework_dic.json").read_text(encoding="utf-8"))
    run([sys.executable, GMP_ROOT / "tools/facilities_generator/src_mgr/facility_dependency_audit.py", "--repo", GMP_ROOT])
    for test_name in ("test_framework_registry.py", "test_facility_dependency_audit.py"):
        run(
            [
                sys.executable,
                "-m",
                "unittest",
                "discover",
                "-s",
                str(GMP_ROOT / "tools/facilities_generator/src_mgr"),
                "-p",
                test_name,
            ]
        )
    sdpe_environment = os.environ.copy()
    sdpe_root = GMP_ROOT / "tools/SDPE_v2"
    sdpe_environment["PYTHONPATH"] = (
        f"{sdpe_root}{os.pathsep}{sdpe_environment['PYTHONPATH']}"
        if sdpe_environment.get("PYTHONPATH")
        else str(sdpe_root)
    )
    for test_name in (
        "test_sdpe_v2.py",
        "test_project_requirements.py",
        "test_project_context.py",
        "test_normalize_suite_sdpe_layout.py",
    ):
        run(
            [
                sys.executable,
                "-m",
                "unittest",
                "discover",
                "-s",
                str(GMP_ROOT / "tools/SDPE_v2/tests"),
                "-p",
                test_name,
            ],
            env=sdpe_environment,
        )


def configure_repository() -> None:
    environment = os.environ.copy()
    environment["GMP_PRO_LOCATION"] = str(GMP_ROOT)
    environment["VIRTUAL_ENV"] = str(VENV_DIR)
    if not (GMP_ROOT / ".git").exists():
        discovery_git = CACHE_ROOT / "discovery-git"
        if not (discovery_git / "HEAD").is_file():
            discovery_git.parent.mkdir(parents=True, exist_ok=True)
            run(["git", "init", "--bare", discovery_git])
        environment["GIT_DIR"] = str(discovery_git)
        environment["GIT_WORK_TREE"] = str(GMP_ROOT)
        print("[INFO] Using a private Git metadata directory for archive-based project discovery")
    run(
        [
            sys.executable,
            GMP_ROOT / "tools/facilities_generator/src_mgr/framework_distribute_tools_v3.py",
            "--deploy-only",
        ],
        cwd=GMP_ROOT,
        env=environment,
    )


def doctor(with_gui: bool, require_vcpkg: bool = True) -> None:
    if sys.version_info < (3, 10):
        raise LinuxEnvironmentError("GMP Linux tools require Python 3.10 or newer")
    if Path(sys.executable).resolve() != (VENV_DIR / "bin/python").resolve():
        raise LinuxEnvironmentError(f"Doctor must run with {VENV_DIR / 'bin/python'}")
    validate_imports(with_gui)
    validate_host_tools()
    if require_vcpkg:
        triplet = target_triplet()
        for header in ("asio.hpp", "nlohmann/json.hpp", "fmt/core.h"):
            path = VCPKG_INSTALL_ROOT / triplet / "include" / header
            if not path.is_file():
                raise LinuxEnvironmentError(f"Missing Linux vcpkg header: {path}")
            print(f"[OK] vcpkg header: {header}")
    validate_repository()
    print("[OK] GMP Linux environment doctor passed")


def install(args: argparse.Namespace) -> None:
    COMPLETION_MARKER_PATH.unlink(missing_ok=True)
    install_python_packages(args.with_gui)
    os.environ["PATH"] = f"{VENV_DIR / 'bin'}{os.pathsep}{os.environ.get('PATH', '')}"
    if not args.skip_vcpkg:
        install_vcpkg()
    write_activation_script()
    configure_repository()
    doctor(args.with_gui, require_vcpkg=not args.skip_vcpkg)
    state = {
        "schema_version": 1,
        "platform": "linux",
        "architecture": platform.machine(),
        "python": platform.python_version(),
        "with_gui": args.with_gui,
        "vcpkg": not args.skip_vcpkg,
        "triplet": None if args.skip_vcpkg else target_triplet(),
    }
    STATE_PATH.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
    COMPLETION_MARKER_PATH.write_text("GMP Linux environment installation completed.\n", encoding="utf-8")
    print(f"[OK] Source this script to enter GMP: {ACTIVATION_PATH}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    install_parser = subparsers.add_parser("install", help="Install and validate the Linux environment")
    install_parser.add_argument("--with-gui", action="store_true", help="Install optional Qt GUI dependencies")
    install_parser.add_argument("--skip-vcpkg", action="store_true", help="Debug only: skip native C/C++ packages")
    doctor_parser = subparsers.add_parser("doctor", help="Validate an installed Linux environment")
    doctor_parser.add_argument("--with-gui", action="store_true", help="Require optional Qt GUI dependencies")
    doctor_parser.add_argument("--skip-vcpkg", action="store_true", help="Do not require native packages")
    subparsers.add_parser("triplet", help="Print the native Linux vcpkg triplet")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.command == "install":
            install(args)
        elif args.command == "doctor":
            doctor(args.with_gui, require_vcpkg=not args.skip_vcpkg)
        else:
            print(target_triplet())
        return 0
    except (LinuxEnvironmentError, OSError, ValueError, json.JSONDecodeError) as error:
        print(f"[ERROR] {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
