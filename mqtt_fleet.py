"""机群状态：直接订阅机器人运控板自带的 MQTT broker，不走 SSH。

为什么要有这条路（而不是全都走 SSH）：
- broker 是**常驻的**（conduit_router → mosquitto，`0.0.0.0:1883`），机器人侧零改动，
  不用往它上面推任何守护进程
- 不占 SSH 通道；WiFi 断了会自己重连，SSH 那条路只留给「发命令」
- Q1 和 T1 的 topic 与 payload 格式一致（同一套 conduit_router），不用按机型分支

只实现 MQTT 3.1.1 的一个子集：CONNECT / SUBSCRIBE / 收 PUBLISH / PINGREQ。
纯标准库，不引 paho —— 不值得为一条只读的旁路加依赖。

协议细节上踩过的坑，都写在下面各自的函数里。
"""

from __future__ import annotations

import json
import os
import random
import re
import socket
import struct
import threading
import time

# 连上之后每隔多久发一次 PINGREQ。比 broker 的 keepalive 短很多，
# 保证即使服务器在上一个周期刚过就认为超时，我们也能续上。
PING_EVERY = 15.0
# 告诉 broker 的 keepalive（秒）。
KEEPALIVE = 60
# 这么久**一个包都没收到**就判定连接已半开（NAT 掉表、WiFi 悄悄断开时，
# socket 本身不会报错，recv 只会一直阻塞）。必须靠这个兜底。
SILENCE_LIMIT = 40.0
BACKOFF_MIN = 1.0
BACKOFF_MAX = 60.0
CONNECT_TIMEOUT = 6.0
# 这么久没收到消息就算离线（broker 心跳很密，正常是每秒好几条）。
QUIET_AFTER = 10.0

TOPIC_FILTER = "/channel/robot_state/#"


def _parse_endpoint(text: str) -> tuple[str, int] | None:
    text = (text or "").strip()
    if not text:
        return None
    host, sep, port = text.rpartition(":")
    if not sep:
        return text, 1883
    try:
        return host, int(port)
    except ValueError:
        return None


def load_fleet() -> dict[str, tuple[str, int]]:
    """机群配置。

    `FLEET_MQTT="T1=192.168.112.178:1883,Q1=192.168.112.222:1883"` 一行加完；
    也可以单独用 `FLEET_T1_MQTT` 之类的变量覆盖某一台。
    两个都不给就不启用（服务空转，`/api/fleet` 返回空机群）。
    """
    out: dict[str, tuple[str, int]] = {}
    for item in (os.environ.get("FLEET_MQTT") or "").split(","):
        name, _, endpoint = item.partition("=")
        name = name.strip().upper()
        if not name:
            continue
        ep = _parse_endpoint(endpoint)
        if ep:
            out[name] = ep
    # 单个覆盖 / 补充
    for key, val in os.environ.items():
        if key.startswith("FLEET_") and key.endswith("_MQTT") and key != "FLEET_MQTT":
            ep = _parse_endpoint(val)
            if ep:
                out[key[len("FLEET_"):-len("_MQTT")].upper()] = ep
    return out


# ---------------------------------------------------------------- MQTT 编码


def _enc_len(n: int) -> bytes:
    """MQTT 变长剩余长度。"""
    out = bytearray()
    while True:
        b = n % 128
        n //= 128
        out.append(b | (0x80 if n > 0 else 0))
        if n == 0:
            return bytes(out)


def _mqtt_str(text: str) -> bytes:
    raw = text.encode()
    return struct.pack("!H", len(raw)) + raw


def _connect_packet(client_id: str) -> bytes:
    # 协议名 MQTT + 级别 4 + 连接标志（clean session）+ keepalive
    vh = _mqtt_str("MQTT") + bytes([4, 0x02]) + struct.pack("!H", KEEPALIVE)
    payload = _mqtt_str(client_id)
    return bytes([0x10]) + _enc_len(len(vh) + len(payload)) + vh + payload


def _subscribe_packet(packet_id: int, topic: str) -> bytes:
    vh = struct.pack("!H", packet_id)
    payload = _mqtt_str(topic) + b"\x00"  # QoS 0
    return bytes([0x82]) + _enc_len(len(vh) + len(payload)) + vh + payload


def _read_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("broker 断开（读到 EOF）")
        buf += chunk
    return buf


def _read_packet(sock: socket.socket) -> tuple[int, bytes]:
    """读一整个 MQTT 包，返回 (类型, 剩余部分)。

    **必须按帧读满**，不能「读 4 个字节就当 SUBACK」。SUBACK 是
    2 字节包标识 + 1 字节返回码，加上固定头正好 5 字节；少读一个字节，
    后面整条流就错位，表现为「不停重连」——很难查，因为看起来像网络问题。
    """
    head = _read_exact(sock, 1)
    mult, length = 1, 0
    for _ in range(4):
        b = _read_exact(sock, 1)[0]
        length += (b & 0x7F) * mult
        mult *= 128
        if not (b & 0x80):
            break
    else:
        raise ValueError("剩余长度字段超过 4 字节")
    return head[0] >> 4, _read_exact(sock, length)


# ------------------------------------------------------------- payload 解析

_STATE_RE = re.compile(rb"/aima/[A-Za-z0-9_/]+")


def _balanced_json(buf: bytes, start: int) -> bytes | None:
    depth = 0
    in_str = False
    esc = False
    for i in range(start, len(buf)):
        ch = buf[i:i + 1]
        if in_str:
            if esc:
                esc = False
            elif ch == b"\\":
                esc = True
            elif ch == b'"':
                in_str = False
            continue
        if ch == b'"':
            in_str = True
        elif ch == b"{":
            depth += 1
        elif ch == b"}":
            depth -= 1
            if depth == 0:
                return buf[start:i + 1]
    return None


def extract_states(payload: bytes) -> dict:
    """从 AimRT 二进制 payload 里捞出 `/aima/xxx` → JSON 状态。

    不去解析 AimRT 的 protobuf 封装：那层只是「话题名 + 序列化类型 + 数据」。
    JSON 本身就是完整的，格式一变这里最多少几个字段，不会崩。
    """
    out: dict = {}
    for match in _STATE_RE.finditer(payload):
        name = match.group(0).decode()
        if name in out:
            continue
        rest = payload[match.end():match.end() + 4200]
        brace = rest.find(b"{")
        if brace < 0 or brace > 64:
            continue
        raw = _balanced_json(rest, brace)
        if raw is None:
            continue
        try:
            out[name] = json.loads(raw.decode("utf-8", "replace"))
        except (ValueError, UnicodeDecodeError):
            continue
    return out


def _dig(obj, *names, depth: int = 4):
    """在嵌套 dict 里找第一个匹配 `names` 的键的值。"""
    if depth <= 0:
        return None
    if isinstance(obj, dict):
        for key in names:
            if key in obj:
                return obj[key]
        for val in obj.values():
            found = _dig(val, *names, depth=depth - 1)
            if found is not None:
                return found
    elif isinstance(obj, list):
        for val in obj:
            found = _dig(val, *names, depth=depth - 1)
            if found is not None:
                return found
    return None


# ------------------------------------------------------------------- 服务


class FleetService:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._robots: dict[str, dict] = {}
        self._started = False

    # -- 生命周期

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        for name, (host, port) in load_fleet().items():
            with self._lock:
                self._robots.setdefault(name, {
                    "host": host,
                    "port": port,
                    "online": False,
                    "msgs": 0,
                    "last_msg": 0.0,
                    "last_error": "",
                    "states": {},
                })
            thread = threading.Thread(
                target=self._worker, args=(name, host, port),
                daemon=True, name=f"fleet-{name}",
            )
            thread.start()
            self._threads.append(thread)

    def stop(self) -> None:
        self._stop.set()

    # -- 单台机器人的连接循环

    def _worker(self, name: str, host: str, port: int) -> None:
        backoff = BACKOFF_MIN
        while not self._stop.is_set():
            try:
                self._run_once(name, host, port)
                backoff = BACKOFF_MIN
            except Exception as exc:  # 任何异常都只影响这一台
                self._fail(name, f"{type(exc).__name__}: {exc}")
            if self._stop.wait(backoff * random.uniform(0.8, 1.2)):
                break
            backoff = min(backoff * 2, BACKOFF_MAX)

    def _run_once(self, name: str, host: str, port: int) -> None:
        sock = socket.create_connection((host, port), timeout=CONNECT_TIMEOUT)
        try:
            sock.sendall(_connect_packet(f"q1-console-{name.lower()}-{os.getpid()}"))
            kind, body = _read_packet(sock)
            if kind != 2:                      # CONNACK
                raise ConnectionError(f"期望 CONNACK，收到类型 {kind}")
            if len(body) >= 2 and body[1] != 0:
                raise ConnectionError(f"broker 拒绝连接，返回码 {body[1]}")

            sock.sendall(_subscribe_packet(1, TOPIC_FILTER))
            kind, _ = _read_packet(sock)
            if kind != 9:                      # SUBACK
                raise ConnectionError(f"期望 SUBACK，收到类型 {kind}")

            self._set_online(name, True)
            last_ping = last_rx = time.monotonic()
            while not self._stop.is_set():
                now = time.monotonic()
                if now - last_rx > SILENCE_LIMIT:
                    raise TimeoutError(f"{SILENCE_LIMIT:.0f}s 无入包，判定连接已半开")
                if now - last_ping >= PING_EVERY:
                    sock.sendall(b"\xc0\x00")
                    last_ping = now
                sock.settimeout(1.0)
                try:
                    kind, body = _read_packet(sock)
                except socket.timeout:
                    continue
                last_rx = time.monotonic()
                if kind == 3:                  # PUBLISH
                    self._handle_publish(name, body)
        finally:
            sock.close()
            self._set_online(name, False)

    def _handle_publish(self, name: str, body: bytes) -> None:
        if len(body) < 2:
            return
        topic_len = struct.unpack("!H", body[:2])[0]
        # QoS 0 的 PUBLISH 没有包标识，payload 紧跟 topic
        payload = body[2 + topic_len:]
        states = extract_states(payload)
        with self._lock:
            robot = self._robots.get(name)
            if robot is None:
                return
            robot["msgs"] += 1
            robot["last_msg"] = time.time()
            robot["online"] = True
            if states:
                robot["states"].update(states)

    # -- 状态写入

    def _set_online(self, name: str, online: bool) -> None:
        with self._lock:
            robot = self._robots.get(name)
            if robot is not None:
                robot["online"] = online

    def _fail(self, name: str, message: str) -> None:
        with self._lock:
            robot = self._robots.get(name)
            if robot is not None:
                robot["online"] = False
                robot["last_error"] = message

    # -- 对外快照

    def snapshot(self) -> dict:
        now = time.time()
        fleet = {}
        with self._lock:
            for name, robot in self._robots.items():
                states = robot["states"]
                bms = states.get("/aima/hal/bms/state") or {}
                fleet[name] = {
                    "host": robot["host"],
                    "port": robot["port"],
                    "online": bool(robot["last_msg"]) and now - robot["last_msg"] < QUIET_AFTER,
                    "msgs": robot["msgs"],
                    "last_msg": round(robot["last_msg"], 1),
                    "age": round(now - robot["last_msg"], 1) if robot["last_msg"] else None,
                    "last_error": robot["last_error"],
                    "robot_name": _dig(bms, "robot_name"),
                    "battery": _dig(bms, "electricity"),
                    "temperature": _dig(bms, "temperature"),
                    "charging": _dig(bms, "is_charging"),
                    "action": _dig(states, "action", "action_name"),
                    "form": _dig(states, "form", "form_state"),
                    "mode": _dig(states, "mode"),
                    "states": sorted(states),
                }
        return {"updated": round(now, 1), "fleet": fleet}


fleet = FleetService()
