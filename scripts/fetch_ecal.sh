#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/third_party/ecal"
DEB="/tmp/ecal_mujoco_interface.deb"
URL="https://github.com/eclipse-ecal/ecal/releases/download/v6.1.1/ecal_6.1.1-jammy_amd64.deb"

if [[ -f "${DEST}/usr/lib/x86_64-linux-gnu/cmake/eCAL/eCALConfig.cmake" ]]; then
  echo "eCAL already present at ${DEST}"
  exit 0
fi

echo "Downloading eCAL..."
curl -fsSL -o "${DEB}" "${URL}"
mkdir -p "${DEST}"
dpkg-deb -x "${DEB}" "${DEST}"
echo "eCAL extracted to ${DEST}"
