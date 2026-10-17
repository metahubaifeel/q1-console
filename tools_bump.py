#!/usr/bin/env python3
"""守护进程改动后必须 bump 版本，否则 ensure_live_daemon 不会重传（静默失效）。
三处必须一起改：live_daemon.py 的 VERSION、server.py 的 DAEMON_VER、
server.py boot 一行里 grep 的 '"v": N'。用法: python3 tools_bump.py"""
import re
import sys
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent
d = ROOT / "live_daemon.py"
s = ROOT / "server.py"

dt, st = d.read_text(), s.read_text()
m = re.search(r"^VERSION = (\d+)$", dt, flags=re.M)
if not m:
    sys.exit("live_daemon.py 里找不到 VERSION")
old, new = int(m.group(1)), int(m.group(1)) + 1

dt, k1 = re.subn(r"^VERSION = \d+$", f"VERSION = {new}", dt, flags=re.M)
st, k2 = re.subn(r"^DAEMON_VER = \d+$", f"DAEMON_VER = {new}", st, flags=re.M)
st, k3 = re.subn(r"""grep -q '\\"v\\": \d+'""", f"""grep -q '\\"v\\": {new}'""", st)

if (k1, k2, k3) != (1, 1, 1):
    sys.exit(f"改到了 {k1}/{k2}/{k3} 处，应该各 1 处 —— 中止，未写入")
d.write_text(dt)
s.write_text(st)
print(f"守护进程版本 {old} -> {new}（三处已同步）")
