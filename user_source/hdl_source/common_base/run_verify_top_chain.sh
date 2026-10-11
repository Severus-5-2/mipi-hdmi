#!/usr/bin/env bash
# 顶层新增链路一键位级验证：
#   三通道 mask → 颜色编码（红>蓝>绿） → conn_label → conn_features → 384bit 总线
set -e
cd "$(dirname "$0")"
PY="${PY:-python}"
[ -x "/c/Users/lenovo/.workbuddy/binaries/python/versions/3.13.12/python.exe" ] && \
  PY="/c/Users/lenovo/.workbuddy/binaries/python/versions/3.13.12/python.exe"
"$PY" verify_top_color_chain.py
