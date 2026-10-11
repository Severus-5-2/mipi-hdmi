#!/usr/bin/env bash
# conn_label.v 一键位级验证：生成图样 → iverilog 编译 → vvp 仿真 → Python 逐项对拍
set -e
cd "$(dirname "$0")"
PY="${PY:-python}"
[ -x "/c/Users/lenovo/.workbuddy/binaries/python/versions/3.13.12/python.exe" ] && \
  PY="/c/Users/lenovo/.workbuddy/binaries/python/versions/3.13.12/python.exe"
"$PY" verify_conn_label.py
