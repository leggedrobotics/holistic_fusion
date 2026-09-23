#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf -- "$build_dir"' EXIT

cmake -S "$repo_root/graph_msf" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DNO_CLANG_TOOLING=ON
cmake --build "$build_dir" --target graph_config_test --parallel 2
ctest --test-dir "$build_dir" --output-on-failure --no-tests=error
