#!/usr/bin/env bash
# Source this file before launching the multi-robot simulation to disable
# FastDDS Shared Memory transport and avoid port-lock errors on VMs / containers.
#
#   source <path_to_go2_config>/setup_dds.bash
#
# To make the setting permanent, paste the two export lines into your ~/.bashrc.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTDDS_XML="${SCRIPT_DIR}/config/fastdds.xml"

if [[ ! -f "${FASTDDS_XML}" ]]; then
    echo "[setup_dds] ERROR: fastdds.xml not found at ${FASTDDS_XML}" >&2
    return 1
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_XML}"
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

echo "[setup_dds] FastDDS SHM disabled. Using UDPv4 only."
echo "  FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo "  RMW_FASTRTPS_USE_QOS_FROM_XML=${RMW_FASTRTPS_USE_QOS_FROM_XML}"
