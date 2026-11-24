#!/usr/bin/env bash
set -e
search_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
search_args=("$@")
set --
unset PYTHONPATH PYTHONNOUSERSITE
source /opt/ros/melodic/setup.bash
source "$HOME/piper_ws/devel/setup.bash" --extend
export PYTHONPATH="$HOME/librealsense-2.50.0/build/wrappers/python${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 "$search_dir/main.py" "${search_args[@]}"
