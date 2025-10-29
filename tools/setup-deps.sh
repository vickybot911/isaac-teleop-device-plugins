#!/usr/bin/env bash
set -euo pipefail

CF_BIN="clang-format-14"

if ! command -v "$CF_BIN" >/dev/null 2>&1; then
  echo "[setup-deps] Installing clang-format-14..."
  if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update -y
    sudo apt-get install -y clang-format-14
  elif command -v dnf >/dev/null 2>&1; then
    sudo dnf install -y clang-tools-extra || sudo dnf install -y clang
  elif command -v yum >/dev/null 2>&1; then
    sudo yum install -y clang-tools-extra || sudo yum install -y clang
  elif command -v pacman >/dev/null 2>&1; then
    sudo pacman -Sy --noconfirm clang
  else
    echo "No supported package manager found. Please install clang-format manually." >&2
    exit 1
  fi
else
  echo "[setup-deps] clang-format-14 already installed: $(command -v "$CF_BIN")"
fi

echo "[setup-deps] Done."


