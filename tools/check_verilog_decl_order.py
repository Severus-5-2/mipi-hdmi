#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_verilog_decl_order.py —— Verilog「声明必须先于使用」静态检查

===== 为什么需要它（2026-10-12 现场血泪）=====
Verilog 标准允许"先用后声明"（net 可以前向引用），**iverilog 也能正确解析**，
所以 TB 全过、仿真看不出任何问题。

但 **TD / Synplify 会为先出现的标识符建一个隐式 1-bit 网** ——
那个网没有驱动、恒为 0。现场后果：

    mask_despeckle.v 里 `.I_wen(A_de)` 写在 `wire A_de = ...` 之前
    ⇒ A_de 恒 0 ⇒ 行缓存从未写入 ⇒ 垂直投票永远凑不满票数
    ⇒ **整片掩膜被清空**（屏幕上绿点和框全部消失）

这类 bug **仿真抓不到、只有上板才暴露**，代价是一个综合+下板周期。
所以做这个静态检查，把它拦在提交前。

===== ⚠ 重要前提：本工具只作【人工复查提示】，不要当硬性门禁 =====
2026-10-12 实测：**TD 能正确解析 Verilog 的前向引用** ——
本项目 `conn_label.v` / `conn_features.v` 里本来就有「先用后声明」
（如 `do_swap` 用在 268 行、声明在 544 行），而这些模块在板子上**工作正常**。
所以本工具报出的项**大概率是误报**，它的价值只是：
    · 提醒你复查新写的代码有没有把声明顺序搞乱；
    · 万一遇到"仿真全过、上板全崩"的怪现象，可以拿它当一条排查线索。

===== 用法 =====
    python check_verilog_decl_order.py <file.v> [file2.v ...]
    退出码：0 = 干净；非 0 = 发现"先用后声明"

    python check_verilog_decl_order.py --quiet mask_despeckle.v
"""

import re
import sys

# 声明关键字（含带赋值形式的 `wire x = ...`）
DECL_KW = r'(?:input|output|inout|wire|reg|integer|parameter|localparam|genvar)'
# 去掉字符串与注释后，找 "关键字 [可选位宽/类型] 名字列表 ;"
DECL_RE = re.compile(
    r'\b' + DECL_KW + r'\b'          # 关键字
    r'(?:\s+(?:signed|unsigned|integer|\[[^\];]*\]|wire|reg|logic))*'  # 可选位宽/类型
    r'\s*([A-Za-z_]\w*(?:\s*,\s*[A-Za-z_]\w*)*)'                      # 名字表
    r'\s*(?:=[^;,]*)?[;,]',           # 可选初值 + 分隔符
    re.S)

ID_RE = re.compile(r'\b[A-Za-z_]\w*\b')

# 这些是 Verilog 关键字 / 常用系统函数，不算标识符
KEYWORDS = set("""
module endmodule input output inout wire reg integer parameter localparam genvar
assign always initial begin end if else case endcase default for while repeat
posedge negedge or and not xor nand nor xnor buf bufif0 bufif1
signed unsigned real time realtime event task function endtask endfunction
generate endgenerate localparam defparam specify endspecify primitive endprimitive
forever forever disable wait fork join join_any join_none
""".split())


def strip_comments(src):
    """把注释与字符串替换成等长空白，保持字符偏移不变。"""
    out = list(src)
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if c == '/' and i + 1 < n and src[i + 1] == '/':
            j = src.find('\n', i)
            j = n if j < 0 else j
            for k in range(i, j):
                out[k] = ' '
            i = j
        elif c == '/' and i + 1 < n and src[i + 1] == '*':
            j = src.find('*/', i + 2)
            j = n if j < 0 else j + 2
            for k in range(i, j):
                if src[k] != '\n':
                    out[k] = ' '
            i = j
        elif c == '"':
            j = i + 1
            while j < n and src[j] != '"':
                j += 2 if src[j] == '\\' else 1
            j = min(j + 1, n)
            for k in range(i, j):
                if src[k] != '\n':
                    out[k] = ' '
            i = j
        else:
            i += 1
    return ''.join(out)


def collect_decls(code):
    """返回 {name: 最早声明位置}"""
    decls = {}
    for m in DECL_RE.finditer(code):
        names = [x.strip() for x in m.group(1).split(',')]
        for nm in names:
            if nm and nm not in KEYWORDS and nm not in decls:
                decls[nm] = m.start()
    return decls


def check_file(path, quiet=False):
    with open(path, 'r', encoding='utf-8', errors='replace') as fp:
        raw = fp.read()
    code = strip_comments(raw)
    decls = collect_decls(code)

    bad = []
    for name, dpos in decls.items():
        # 该名字在声明之前是否出现过（作为独立标识符；跳过 `MACRO 形式的宏引用）
        for m in ID_RE.finditer(code, 0, dpos):
            if m.group(0) != name:
                continue
            if m.start() > 0 and code[m.start() - 1] == '`':
                continue          # `` `LABEL_MAX `` 是宏引用，不是信号 —— 跳过
            line = code.count('\n', 0, m.start()) + 1
            bad.append((name, line, code.count('\n', 0, dpos) + 1))
            break

    if bad:
        print("!! [先用后声明] %s" % path)
        for name, uline, dline in sorted(bad, key=lambda x: x[1]):
            print("     - '%s' 在第 %d 行被使用，声明却在第 %d 行" % (name, uline, dline))
        print("   提示：Verilog 允许前向引用，实测 TD 也能正确解析（本仓库 conn_label.v")
        print("         里就有这类写法且上板工作正常）⇒ 多数情况是**误报**，仅供人工复查。")
        return 1
    if not quiet:
        print("   ok  %s：未发现「先用后声明」" % path)
    return 0


def main():
    args = [a for a in sys.argv[1:] if a != '--quiet']
    quiet = '--quiet' in sys.argv
    if not args:
        print(__doc__)
        return 2
    rc = 0
    for p in args:
        rc |= check_file(p, quiet=quiet)
    return rc


if __name__ == '__main__':
    sys.exit(main())
