#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
export PYTHONDONTWRITEBYTECODE=1
for package in "$PWD"/ros2_ws/src/*; do
    export PYTHONPATH="$package${PYTHONPATH:+:$PYTHONPATH}"
done
python3 --version
python3 scripts/test_launcher_offline.py
if [[ -d tests ]]; then
    python3 -m unittest discover -s tests -v
fi
for package in ros2_ws/src/*; do
    if [[ -d "$package/tests" ]]; then
        python3 -m unittest discover -s "$package/tests" -v
    fi
done
