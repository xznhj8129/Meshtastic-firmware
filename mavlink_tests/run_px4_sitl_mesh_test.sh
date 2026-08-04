#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${PX4_MESH_TEST_ENV:-${SCRIPT_DIR}/px4_sitl_mesh_test.env}"
VENV_DIR="${PX4_MESH_TEST_VENV:-${REPO_ROOT}/.venv-px4-mesh-test}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

if [[ ! -f "${ENV_FILE}" ]]; then
    echo "Missing ${ENV_FILE}" >&2
    echo "Copy ${SCRIPT_DIR}/px4_sitl_mesh_test.env.example and fill in the three required values." >&2
    exit 2
fi

# shellcheck disable=SC1090
source "${ENV_FILE}"

: "${PX4_DIR:?Set PX4_DIR in ${ENV_FILE}}"
: "${AIR_UART:?Set AIR_UART in ${ENV_FILE}}"
: "${GROUND_HOST:?Set GROUND_HOST in ${ENV_FILE}}"

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
    "${PYTHON_BIN}" -m venv "${VENV_DIR}"
fi

"${VENV_DIR}/bin/python" -m pip install --disable-pip-version-check -q \
    -r "${SCRIPT_DIR}/requirements-px4-sitl-mesh.txt"

cd "${REPO_ROOT}"

exec "${VENV_DIR}/bin/python" "${SCRIPT_DIR}/px4_sitl_mesh_test.py" \
    --px4-dir "${PX4_DIR}" \
    --air-uart "${AIR_UART}" \
    --air-baud "${AIR_BAUD:-115200}" \
    --ground-host "${GROUND_HOST}" \
    --ground-port "${GROUND_PORT:-14550}" \
    --local-port "${LOCAL_PORT:-14600}" \
    --expected-sysid "${EXPECTED_SYSID:-1}" \
    --max-rate-bps "${MAX_RATE_BPS:-1000}" \
    --build-timeout "${BUILD_TIMEOUT:-1200}" \
    --test-timeout "${TEST_TIMEOUT:-240}" \
    --command-attempts "${COMMAND_ATTEMPTS:-3}" \
    --report-root "${REPORT_ROOT:-mavlink_tests/reports}" \
    "$@"
