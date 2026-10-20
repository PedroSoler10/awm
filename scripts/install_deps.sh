#!/usr/bin/env bash
# Installs the apt packages for every dependency listed in package.xml.
#
# Usage:
#   scripts/install_deps.sh [--print] [path/to/package.xml]
#
# --print lists the apt packages that would be installed, without installing them.

set -euo pipefail

PRINT_ONLY=0
if [ "${1:-}" = "--print" ]; then
  PRINT_ONLY=1
  shift
fi
PKG_XML="${1:-$(dirname "$0")/../package.xml}"

# ROS package names whose apt name is not simply "ros-noetic-" + the name with "-" instead of "_"
declare -A APT_NAME=(
  [gazebo_ros]=gazebo-ros-pkgs
)

# Unique dependency names from package.xml (the catkin build tool is provided by ros-base)
mapfile -t DEPS < <(python3 - "$PKG_XML" <<'EOF'
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
seen = set()
for tag in ("depend", "build_depend", "exec_depend"):
    for element in root.findall(tag):
        name = element.text.strip()
        if name not in seen:
            seen.add(name)
            print(name)
EOF
)

PACKAGES=()
for dep in "${DEPS[@]}"; do
  apt_name="${APT_NAME[$dep]:-${dep//_/-}}"
  PACKAGES+=("ros-noetic-$apt_name")
done

if [ "$PRINT_ONLY" -eq 1 ]; then
  printf '%s\n' "${PACKAGES[@]}"
  exit 0
fi

SUDO=""
if [ "$(id -u)" -ne 0 ]; then
  SUDO="sudo"
fi

$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends "${PACKAGES[@]}"
