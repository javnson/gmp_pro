#!/usr/bin/env bash
# Install the GMP Linux toolchain without modifying the host Python or shell profiles.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
GMP_LINUX_ROOT="${SCRIPT_DIR}/bin/linux"
GMP_UV_DIR="${GMP_LINUX_ROOT}/uv"
GMP_UV_BIN="${GMP_UV_DIR}/uv"
GMP_VENV_DIR="${GMP_LINUX_ROOT}/venv"
GMP_CACHE_DIR="${GMP_LINUX_ROOT}/cache"
GMP_UV_VERSION="0.11.15"

case "$(uname -s)" in
    Linux) ;;
    *) echo "[ERROR] This installer supports Linux only." >&2; exit 1 ;;
esac

case "$(uname -m)" in
    x86_64|amd64)
        GMP_UV_TARGET="x86_64-unknown-linux-musl"
        GMP_UV_SHA256="200ccf2f351849c5d6698714e7e7eb9ead1e8c097dbdbb43730e1a4e059ceb87"
        ;;
    aarch64|arm64)
        GMP_UV_TARGET="aarch64-unknown-linux-musl"
        GMP_UV_SHA256="6505075cec3f551fad4fe9026922967ff9c895c9f513c97682b24e7a1c9becd3"
        ;;
    *) echo "[ERROR] Unsupported Linux architecture: $(uname -m)" >&2; exit 1 ;;
esac

mkdir -p "${GMP_UV_DIR}" "${GMP_CACHE_DIR}"
GMP_UV_ARCHIVE="${GMP_CACHE_DIR}/uv-${GMP_UV_VERSION}-${GMP_UV_TARGET}.tar.gz"
GMP_UV_URL="https://github.com/astral-sh/uv/releases/download/${GMP_UV_VERSION}/uv-${GMP_UV_TARGET}.tar.gz"

if [[ ! -x "${GMP_UV_BIN}" ]]; then
    echo "[GMP] Downloading the pinned private uv runtime."
    if [[ ! -f "${GMP_UV_ARCHIVE}" ]]; then
        if command -v curl >/dev/null 2>&1; then
            curl -fL --retry 3 --connect-timeout 20 -C - -o "${GMP_UV_ARCHIVE}.part" "${GMP_UV_URL}"
            mv "${GMP_UV_ARCHIVE}.part" "${GMP_UV_ARCHIVE}"
        elif command -v wget >/dev/null 2>&1; then
            wget -c -O "${GMP_UV_ARCHIVE}.part" "${GMP_UV_URL}"
            mv "${GMP_UV_ARCHIVE}.part" "${GMP_UV_ARCHIVE}"
        else
            echo "[ERROR] curl or wget is required to bootstrap the private environment." >&2
            exit 1
        fi
    fi
    echo "${GMP_UV_SHA256}  ${GMP_UV_ARCHIVE}" | sha256sum --check --status || {
        echo "[ERROR] The downloaded uv archive failed SHA-256 validation." >&2
        exit 1
    }
    GMP_UV_EXTRACT="$(mktemp -d "${GMP_CACHE_DIR}/uv-extract.XXXXXX")"
    trap 'rm -rf -- "${GMP_UV_EXTRACT}"' EXIT
    tar -xzf "${GMP_UV_ARCHIVE}" -C "${GMP_UV_EXTRACT}"
    GMP_UV_EXTRACTED="$(find "${GMP_UV_EXTRACT}" -type f -name uv -print -quit)"
    if [[ -z "${GMP_UV_EXTRACTED}" ]]; then
        echo "[ERROR] The uv archive does not contain the expected executable." >&2
        exit 1
    fi
    install -m 0755 "${GMP_UV_EXTRACTED}" "${GMP_UV_BIN}"
    rm -rf -- "${GMP_UV_EXTRACT}"
    trap - EXIT
fi

export GMP_PRO_LOCATION="${SCRIPT_DIR}"
export UV_CACHE_DIR="${GMP_CACHE_DIR}/uv"
export UV_PYTHON_INSTALL_DIR="${GMP_LINUX_ROOT}/python"

case "${GMP_VENV_DIR}" in
    "${SCRIPT_DIR}/bin/linux/venv") rm -rf -- "${GMP_VENV_DIR}" ;;
    *) echo "[ERROR] Refusing to replace an unexpected virtual-environment path." >&2; exit 1 ;;
esac

echo "[GMP] Creating the repository-private Python 3.12 virtual environment."
echo "[GMP] The first run downloads a managed Python runtime; slow links can take several minutes."
"${GMP_UV_BIN}" venv --managed-python --python 3.12 --relocatable "${GMP_VENV_DIR}"
"${GMP_UV_BIN}" pip install --python "${GMP_VENV_DIR}/bin/python" pip
export PATH="${GMP_VENV_DIR}/bin:${PATH}"

echo "[GMP] Installing and validating Linux GMP services."
"${GMP_VENV_DIR}/bin/python" \
    "${SCRIPT_DIR}/tools/gmp_installer/linux_environment_manager.py" install "$@"

echo
echo "[GMP] Linux environment installation completed successfully."
echo "[GMP] Enter it with: source \"${GMP_LINUX_ROOT}/activate_gmp.sh\""
