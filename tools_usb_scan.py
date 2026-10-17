#!/usr/bin/env python3
"""找出 Q1 直连（USB 网卡）时对面到底在哪个地址。

为什么不用 ping：机器人在很多网络配置下不答 ICMP，`nmap -sn` 非 root 时
也做不了 ARP 扫描，结果就是「机器明明在，扫出来一片空」。这里改用 TCP
connect（不需要 root），在已配置的每个 /24 上敲 22 / 80 / 443 / 8080 / 8888。

用法：插好线之后直接跑，不用参数。
  python3 tools_usb_scan.py
"""

from __future__ import annotations

import concurrent.futures
import ipaddress
import socket
import subprocess
import sys

PORTS = (22, 80, 443, 8080, 8888)
TIMEOUT = 0.35
WORKERS = 128


def usb_interfaces() -> list[tuple[str, str]]:
    """返回 [(网卡名, ipv4/掩码)]，只保留 enx*/usb* 这类 USB 网卡。"""
    out: list[tuple[str, str]] = []
    try:
        raw = subprocess.run(
            ["ip", "-4", "-o", "addr", "show"],
            capture_output=True, text=True, timeout=5, check=True,
        ).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        sys.exit(f"跑不了 ip 命令：{exc}")

    for line in raw.splitlines():
        parts = line.split()
        if len(parts) < 4 or parts[2] != "inet":
            continue
        name, cidr = parts[1], parts[3]
        if not (name.startswith("enx") or name.startswith("usb")):
            continue
        out.append((name, cidr))
    return out


def probe(host: str) -> list[int]:
    hits = []
    for port in PORTS:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(TIMEOUT)
            try:
                sock.connect((host, port))
                hits.append(port)
            except OSError:
                pass
    return hits


def main() -> int:
    ifaces = usb_interfaces()
    if not ifaces:
        print("没看到 USB 网卡。先确认线插着、`ip -br link` 里有 enx* 并且 UP。")
        return 1

    targets: list[str] = []
    for name, cidr in ifaces:
        net = ipaddress.ip_network(cidr, strict=False)
        ours = str(ipaddress.ip_interface(cidr).ip)
        print(f"{name}: {cidr}（本机 {ours}）")
        targets += [str(ip) for ip in net.hosts() if str(ip) != ours]

    print(f"\n扫 {len(targets)} 个地址 × {len(PORTS)} 个端口（TCP，不需要 root）…")
    found: list[tuple[str, list[int]]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=WORKERS) as pool:
        for host, hits in zip(targets, pool.map(probe, targets)):
            if hits:
                found.append((host, hits))

    if not found:
        print("没有主机应答。说明对面不在这些网段，或者整条线根本没通。")
        return 2

    print("\n有应答：")
    for host, hits in sorted(found):
        print(f"  {host}  端口 {', '.join(map(str, hits))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
