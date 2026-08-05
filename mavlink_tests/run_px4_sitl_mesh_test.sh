#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${PX4_MESH_TEST_ENV:-${SCRIPT_DIR}/px4_sitl_mesh_test.env}"
VENV_DIR="${PX4_MESH_TEST_VENV:-${SCRIPT_DIR}/.venv-px4-mesh-test}"
PYTHON_BIN="${PYTHON_BIN:-python3}"

if [[ ! -f "${ENV_FILE}" ]]; then
    echo "Missing ${ENV_FILE}" >&2
    echo "Copy ${SCRIPT_DIR}/px4_sitl_mesh_test.env.example and fill in the required values." >&2
    exit 2
fi

# shellcheck disable=SC1090
source "${ENV_FILE}"

: "${PX4_DIR:?Set PX4_DIR in ${ENV_FILE}}"
: "${AIR_UART:?Set AIR_UART in ${ENV_FILE}}"
: "${GROUND_HOST:?Set GROUND_HOST in ${ENV_FILE}}"

AIR_UART_CONFIGURED="${AIR_UART}"
AIR_UART="$(readlink -f -- "${AIR_UART_CONFIGURED}")"
if [[ -z "${AIR_UART}" || ! -e "${AIR_UART}" ]]; then
    echo "AIR_UART does not resolve to an existing device: ${AIR_UART_CONFIGURED}" >&2
    exit 2
fi
printf 'Air UART: %s -> %s\n' "${AIR_UART_CONFIGURED}" "${AIR_UART}"

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
    "${PYTHON_BIN}" -m venv "${VENV_DIR}"
fi

"${VENV_DIR}/bin/python" -m pip install --disable-pip-version-check -q \
    -r "${SCRIPT_DIR}/requirements-px4-sitl-mesh.txt"

cd "${REPO_ROOT}"
export MAVLINK20=1

exec "${VENV_DIR}/bin/python" "${SCRIPT_DIR}/px4_sitl_mesh_test.py" \
    --px4-dir "${PX4_DIR}" \
    --air-uart "${AIR_UART}" \
    --air-baud "${AIR_BAUD:-57600}" \
    --ground-host "${GROUND_HOST}" \
    --ground-port "${GROUND_PORT:-14550}" \
    --local-port "${LOCAL_PORT:-14600}" \
    --target-sysid "${TARGET_SYSID:-${EXPECTED_SYSID:-1}}" \
    --target-compid "${TARGET_COMPID:-1}" \
    --max-rate-bps "${MAX_RATE_BPS:-1000}" \
    --build-timeout "${BUILD_TIMEOUT:-1200}" \
    --warmup "${WARMUP_SECONDS:-5}" \
    --probe-attempts "${PROBE_ATTEMPTS:-${COMMAND_ATTEMPTS:-10}}" \
    --probe-interval "${PROBE_INTERVAL_SECONDS:-3}" \
    --control-offset "${CONTROL_OFFSET_SECONDS:-1}" \
    --drain "${DRAIN_SECONDS:-8}" \
    --report-root "${REPORT_ROOT:-mavlink_tests/reports}" \
    "$@"
