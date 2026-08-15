#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${GMP_PRO_LOCATION:-}" ]]; then
    echo "[ERROR] GMP_PRO_LOCATION is not defined. Source the GMP Linux environment first." >&2
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PYTHON_BIN="${VIRTUAL_ENV:-${GMP_PRO_LOCATION}/bin/linux/venv}/bin/python"
if [[ ! -x "${PYTHON_BIN}" ]]; then
    echo "[ERROR] GMP Linux Python is unavailable. Run install_gmp_virtual_env.sh first." >&2
    exit 1
fi

cd -- "${SCRIPT_DIR}"
"${PYTHON_BIN}" "${GMP_PRO_LOCATION}/tools/facilities_generator/src_mgr/framework_sync_inc_v3.py"
