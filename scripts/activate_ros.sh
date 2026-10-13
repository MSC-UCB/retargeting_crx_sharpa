#!/usr/bin/env bash
# Source this file in Bash; ROS Jazzy requires a Python 3.12 virtualenv.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Use: source scripts/activate_ros.sh" >&2
    exit 1
fi

_retargeting_activate_ros() {
    local repo site cmeel
    repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
    if [[ ! -f "$repo/.venv/bin/activate" ]]; then
        echo "Missing .venv; complete Install and Test first." >&2
        return 1
    fi
    env -u PYTHONPATH -u LD_LIBRARY_PATH "$repo/.venv/bin/python" -c \
        'import sys; assert sys.version_info[:2] == (3, 12), "ROS Jazzy requires Python 3.12"' || return 1
    site="$repo/.venv/lib/python3.12/site-packages"
    cmeel="$site/cmeel.prefix"
    if [[ ! -d "$cmeel/lib/python3.12/site-packages/pinocchio" ]]; then
        echo "Missing virtualenv Pinocchio; complete Install and Test first." >&2
        return 1
    fi
    source /opt/ros/jazzy/setup.bash || return 1
    source "$HOME/ws_fanuc/install/setup.bash" || return 1
    source "$repo/.venv/bin/activate" || return 1
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
    export PYTHONPATH="$cmeel/lib/python3.12/site-packages:$site${PYTHONPATH:+:$PYTHONPATH}"
    export LD_LIBRARY_PATH="$cmeel/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    echo "Retargeting venv active; ROS_DOMAIN_ID=$ROS_DOMAIN_ID (must match drivers)."
}

if _retargeting_activate_ros; then
    unset -f _retargeting_activate_ros
else
    unset -f _retargeting_activate_ros
    return 1
fi
