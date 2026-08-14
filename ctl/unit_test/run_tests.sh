#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${GMP_PRO_LOCATION:-}" || -z "${GMP_LINUX_ENV:-}" ]]; then
    echo "[ERROR] Source bin/linux/activate_gmp.sh before running CTL tests." >&2
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
BUILD_DIR="${SCRIPT_DIR}/out/linux"

cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}" \
    -DVCPKG_TARGET_TRIPLET="${VCPKG_DEFAULT_TRIPLET}" \
    -DVCPKG_MANIFEST_DIR="${GMP_LINUX_ENV}/cache/vcpkg-manifest" \
    -DVCPKG_INSTALLED_DIR="${VCPKG_INSTALLED_DIR}"
cmake --build "${BUILD_DIR}"
ctest --test-dir "${BUILD_DIR}" --output-on-failure
