#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf -- "$build_dir"' EXIT

cmake -S "$repo_root/holistic_fusion" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DNO_CLANG_TOOLING=ON
cmake --build "$build_dir" --parallel 2
ctest --test-dir "$build_dir" --output-on-failure --no-tests=error
