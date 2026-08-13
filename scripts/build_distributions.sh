#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${PYTHON:-python3}"

for package in "${workspace_root}"/packages/*; do
  "${python_bin}" -m build "${package}"
done
