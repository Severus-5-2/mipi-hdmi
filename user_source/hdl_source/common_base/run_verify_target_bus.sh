#!/usr/bin/env bash
# target_bus 全链路一键位级验证：
#   conn_label + conn_features + target_pack → 384bit 总线逐位对拍
set -e
cd "$(dirname "$0")"
PY="${PY:-python}"
[ -x "/c/Users/lenovo/.workbuddy/binaries/python/versions/3.13.12/python.exe" ] && \
  PY="/c/Users/lenovo/.workbuddy/binaries/python/versions/3.13.12/python.exe"
"$PY" verify_target_bus.py
