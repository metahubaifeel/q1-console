#!/usr/bin/env python3
"""Q1 local console: browser -> this PC -> SSH -> onboard ROS2."""

from __future__ import annotations

import asyncio
import json
import os
import secrets
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import anyio
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

import mqtt_fleet
import people as facebook

ROOT = Path(__file__).resolve().parent
ENV_FILE = ROOT / ".env"


def _load_env() -> None:
    if not ENV_FILE.exists():
        return
    for line in ENV_FILE.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, _, v = line.partition("=")
        os.environ.setdefault(k.strip(), v.strip())


_load_env()

HOST = os.environ.get("Q1_HOST", "192.168.50.181")
LAN_HOST = os.environ.get("Q1_LAN_HOST", HOST)
TS_HOST = os.environ.get("Q1_TS_HOST", "q1-shanghai")
USER = os.environ.get("Q1_USER", "run")
PASS = os.environ.get("Q1_PASS", "")
LAN_IFACE = os.environ.get("Q1_LAN_IFACE", "")
# 机器人不在这台机器的网段上时（比如插在别的工作站上），可以走本地 SSH 隧道：
# 隧道把机器人 22/2554/8799 映射到 127.0.0.1，这里只需要指定 SSH 的本地端口。
SSH_PORT = os.environ.get("Q1_SSH_PORT", "")
# 免密模式：配好 ssh key（ssh-copy-id run@<机器人>）就不用 Q1_PASS。
# Windows 上没有 sshpass，走这条路。
KEY_MODE = os.environ.get("Q1_SSH_KEY", "")
# RTSP 路径名随机型不同：Q1 是单目 live_head_monocular_centra，T1 是左右目
# live_head_stereo_left / _right（见机器人上的 rtsp_server_t1_v2d0_sub.yaml）。
RTSP_PATH = os.environ.get("Q1_RTSP_PATH", "live_head_monocular_centra")
TS_BIN = os.environ.get(
    "TAILSCALE_BIN",
    str(Path.home() / ".local/tailscale/tailscale_1.102.4_amd64/tailscale"),
)
TS_SOCK = os.environ.get(
    "TAILSCALE_SOCKET",
    str(Path.home() / ".local/tailscale/tailscaled.sock"),
)
TS_HTTP_LOCAL = 18799
TS_RTSP_LOCAL = 12554

ROS_BASH = r"""
export LD_LIBRARY_PATH=/robot/software/cloud_proxy/lib:/robot/software/aimrt_agent/lib:/opt/ros/humble/lib:${LD_LIBRARY_PATH}
export PYTHONPATH=/robot/software/cloud_proxy/local/lib/python3.10/dist-packages:/robot/software/aimrt_agent/local/lib/python3.10/dist-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:${PYTHONPATH}
export AMENT_PREFIX_PATH=/robot/software/cloud_proxy:/robot/software/aimrt_agent:/opt/ros/humble
export ROS_LOCALHOST_ONLY=0
unset ROS_DOMAIN_ID
# Every robot-side process runs with this FastDDS profile (see
# /robot/software/*/bin/em_run.sh). Without it our participant still discovers
# most services, but NOT the ones served from the profiled participant -- PlayTts
# shows up in "ros2 service list" yet call_async never matches, so SAY silently
# reports is_success=False and the robot stays mute. Verified A/B/A/B: toggling
# only this variable flips PlayTts service_is_ready False/True/False/True.
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/robot/software/entry/cfg/standard/ros_dds_configuration.xml}
[ -f /opt/ros/humble/setup.bash ] && . /opt/ros/humble/setup.bash
"""

REMOTE_PY_PREFIX = r"""
import os, sys
_pp = '/robot/software/cloud_proxy/local/lib/python3.10/dist-packages:/robot/software/aimrt_agent/local/lib/python3.10/dist-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages'
os.environ['LD_LIBRARY_PATH'] = '/robot/software/cloud_proxy/lib:/robot/software/aimrt_agent/lib:/opt/ros/humble/lib:' + os.environ.get('LD_LIBRARY_PATH','')
os.environ['PYTHONPATH'] = _pp + ':' + os.environ.get('PYTHONPATH','')
os.environ['AMENT_PREFIX_PATH'] = '/robot/software/cloud_proxy:/robot/software/aimrt_agent:/opt/ros/humble'
os.environ['ROS_LOCALHOST_ONLY'] = '0'
os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', '/robot/software/entry/cfg/standard/ros_dds_configuration.xml')
os.environ.pop('ROS_DOMAIN_ID', None)
for p in _pp.split(':'):
    if p and p not in sys.path:
        sys.path.insert(0, p)
"""

app = FastAPI(title="Q1 Console")
app.mount("/static", StaticFiles(directory=str(ROOT / "static")), name="static")


@app.on_event("startup")
def _on_start():
    try:
        facebook.sweep_junk()
    except Exception:
        pass
    _start_ts_forward(TS_HTTP_LOCAL, DAEMON_PORT)
    _start_ts_forward(TS_RTSP_LOCAL, 2554)
    start_snapper()
    threading.Thread(target=live_pump, daemon=True).start()
    threading.Thread(target=lambda: ensure_cmd(), daemon=True).start()
    # 机群状态走 MQTT 旁路，和上面那条 SSH 通道并行，互不依赖。
    mqtt_fleet.fleet.start()


@app.on_event("shutdown")
def _on_stop():
    mqtt_fleet.fleet.stop()


class SpeakIn(BaseModel):
    text: str = Field(min_length=1, max_length=200)


class EmotionIn(BaseModel):
    emotion_id: int


class LightIn(BaseModel):
    mode: int
    r: int = 0
    g: int = 0
    b: int = 0
    period: int = 0


class MotionIn(BaseModel):
    motion_id: int


class CustomMotionIn(BaseModel):
    rtype: int = Field(ge=0, le=2)
    name: str = Field(min_length=1, max_length=200)


class MuteIn(BaseModel):
    mute: bool


class VolumeIn(BaseModel):
    volume: int = Field(ge=0, le=20)


class ActionIn(BaseModel):
    action_id: int


class FollowIn(BaseModel):
    on: bool
    kind: str = "follow"
    target_id: int = 0
    watch: bool = False


class MapCmdIn(BaseModel):
    cmd: str
    name: str = ""
    watch: bool = False


class NaviIn(BaseModel):
    cmd: str
    name: str = ""
    x: float = 0
    y: float = 0
    theta: float = 0
    watch: bool = False


class LocIn(BaseModel):
    on: bool
    map_id: str = ""
    watch: bool = False


class PointIn(BaseModel):
    map_id: str = ""
    name: str = "这里"
    watch: bool = False


class DriveIn(BaseModel):
    fwd: float = Field(ge=-0.4, le=0.4)
    lat: float = Field(ge=-0.3, le=0.3)
    yaw: float = Field(ge=-0.8, le=0.8)


class PersonIn(BaseModel):
    id: str = ""
    name: str = ""
    image: str = ""
    x: int = 0
    y: int = 0
    w: int = 0
    h: int = 0


_drive_lock = threading.Lock()
_drive_proc: subprocess.Popen | None = None
_drive_log: list[str] = []
_drive_log_ev = threading.Event()
_cmd_lock = threading.Lock()
_cmd_proc: subprocess.Popen | None = None
_cmd_log: list[str] = []
_cmd_ev = threading.Event()


_peer_lock = threading.Lock()
_peer = {"host": LAN_HOST, "via_ts": False, "t": 0.0}


def _lan_onlink() -> bool:
    if os.environ.get("Q1_LAN_MODE") == "1":
        return True
    try:
        out = subprocess.check_output(["ip", "-4", "-o", "addr"], text=True, timeout=1)
    except (subprocess.SubprocessError, OSError):
        # macOS 没有 ip 命令，退回 ifconfig
        try:
            out = subprocess.check_output(["ifconfig"], text=True, timeout=1)
        except (subprocess.SubprocessError, OSError):
            return False
    # 按 LAN_HOST 自己推导 /24，别再写死某一段：机器人直连时是 10.1.1.x，
    # 走家里路由器时是 192.168.50.x，写死哪一段另一次就连不上。
    prefix = ".".join(LAN_HOST.split(".")[:3]) + "."
    return prefix in out


def _route() -> tuple[str, bool]:
    now = time.time()
    with _peer_lock:
        if now - float(_peer["t"]) < 6:
            return str(_peer["host"]), bool(_peer["via_ts"])
    via_ts = not _lan_onlink()
    host = TS_HOST if via_ts else LAN_HOST
    with _peer_lock:
        _peer.update(host=host, via_ts=via_ts, t=now)
    return host, via_ts


def _ssh_argv(remote: str) -> list[str]:
    host, via_ts = _route()
    if PASS:
        cmd = [
            "sshpass",
            "-e",
            "ssh",
            "-o",
            "PreferredAuthentications=password",
            "-o",
            "PubkeyAuthentication=no",
            "-o",
            "IdentitiesOnly=yes",
            "-o",
            "NumberOfPasswordPrompts=1",
        ]
    else:
        # 免密模式：配好 ssh key（ssh-copy-id run@<机器人>）就不用 sshpass
        cmd = ["ssh"]
    cmd += [
        "-o",
        "ConnectTimeout=8",
        "-o",
        "StrictHostKeyChecking=accept-new",
    ]
    if via_ts:
        cmd += [
            "-o",
            f"ProxyCommand={TS_BIN} --socket={TS_SOCK} nc %h %p",
        ]
    elif LAN_IFACE:
        cmd += ["-o", f"BindInterface={LAN_IFACE}"]
    if SSH_PORT and not via_ts:
        cmd += ["-p", SSH_PORT]
    cmd += [f"{USER}@{host}", remote]
    return cmd


def _ssh_env() -> dict:
    env = os.environ.copy()
    if PASS:
        env["SSHPASS"] = PASS
    return env


def _start_ts_forward(local_port: int, remote_port: int) -> None:
    def handle(client: socket.socket) -> None:
        try:
            proc = subprocess.Popen(
                [TS_BIN, f"--socket={TS_SOCK}", "nc", TS_HOST, str(remote_port)],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except OSError:
            client.close()
            return

        def uplink() -> None:
            try:
                assert proc.stdin is not None
                while True:
                    chunk = client.recv(65536)
                    if not chunk:
                        break
                    proc.stdin.write(chunk)
                    proc.stdin.flush()
            except OSError:
                pass
            finally:
                # 客户端半关写端 ≠ 会话结束：如果是 POST，回执还要从这条隧道回来。
                # 以前这里直接 proc.terminate()，把回程连同还没转发出去的缓冲一起杀了。
                # GET 路径永远不会半关，所以这个 bug 一直没露头。
                try:
                    if proc.stdin:
                        proc.stdin.close()
                except OSError:
                    pass

                def reap() -> None:
                    try:
                        proc.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        proc.terminate()

                threading.Thread(target=reap, daemon=True).start()

        def downlink() -> None:
            try:
                assert proc.stdout is not None
                while True:
                    chunk = proc.stdout.read(65536)
                    if not chunk:
                        break
                    client.sendall(chunk)
            except OSError:
                pass
            finally:
                try:
                    client.shutdown(socket.SHUT_WR)
                except OSError:
                    pass

        threading.Thread(target=uplink, daemon=True).start()
        downlink()
        client.close()

    def serve() -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("127.0.0.1", local_port))
        srv.listen(32)
        while True:
            conn, _ = srv.accept()
            threading.Thread(target=handle, args=(conn,), daemon=True).start()

    threading.Thread(target=serve, daemon=True).start()


def _kill_remote_drive() -> None:
    subprocess.run(
        _ssh_argv(
            "python3 - <<'PY'\n"
            "import os, signal\n"
            "for line in os.popen('ps -eo pid,cmd'):\n"
            "    if 'q1_drive_loop.py' in line:\n"
            "        pid = int(line.split(None, 1)[0])\n"
            "        if pid != os.getpid():\n"
            "            try: os.kill(pid, signal.SIGTERM)\n"
            "            except OSError: pass\n"
            "print('drive_killed')\n"
            "PY"
        ),
        timeout=8,
        env=_ssh_env(),
        capture_output=True,
        text=True,
    )


def _eat_drive(proc: subprocess.Popen) -> None:
    assert proc.stdout is not None
    for line in proc.stdout:
        s = (line or "").rstrip()
        if not s:
            continue
        with _drive_lock:
            _drive_log.append(s)
            if len(_drive_log) > 50:
                del _drive_log[:-30]
        _drive_log_ev.set()


def _require_auth() -> None:
    """有密码，或者明确说了走免密（Q1_SSH_KEY），才继续。

    以前这里一律要求 Q1_PASS，Windows 上没有 sshpass，等于整个控制台不可用。
    """
    if not PASS and not KEY_MODE:
        raise HTTPException(500, "Q1_PASS missing（或设 Q1_SSH_KEY=1 走免密）")


def ensure_drive() -> subprocess.Popen:
    global _drive_proc
    _require_auth()
    with _drive_lock:
        if _drive_proc is not None and _drive_proc.poll() is None:
            return _drive_proc
        _kill_remote_drive()
        up = subprocess.run(
            _ssh_argv("cat > /tmp/q1_drive_loop.py"),
            input=(ROOT / "drive_loop.py").read_text(),
            text=True,
            timeout=12,
            env=_ssh_env(),
            capture_output=True,
        )
        if up.returncode != 0:
            raise HTTPException(502, (up.stderr or up.stdout or "upload drive loop failed")[-800:])
        proc = subprocess.Popen(
            _ssh_argv(f"{ROS_BASH}\npython3 -u /tmp/q1_drive_loop.py"),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=_ssh_env(),
            bufsize=1,
        )
        ready = False
        assert proc.stdout is not None
        deadline = time.time() + 20
        while time.time() < deadline:
            line = proc.stdout.readline()
            if "DRIVE_READY" in (line or ""):
                ready = True
                break
            if proc.poll() is not None:
                rest = proc.stdout.read() if proc.stdout else ""
                raise HTTPException(502, ((line or "") + rest)[-800:])
        if not ready:
            proc.kill()
            raise HTTPException(504, "drive loop did not start")
        threading.Thread(target=_eat_drive, args=(proc,), daemon=True).start()
        _drive_proc = proc
        return proc


def drive_send(fwd: float, lat: float, yaw: float) -> bool:
    """发一帧速度。**故意不在这里 ensure_drive()。**

    原来这里是 ensure_drive()，字面意思就是「按一下就拉起一个远端发布器」。
    而浏览器那边的保活是 8.3 Hz（padTimer=120ms）—— 只要那个标签页还开着、
    「遥控」还是开着的状态，控制台这边无论发生什么（重启服务、进程被杀），
    下一个 120 毫秒它就会把机器人重新接管回来。2026-09-14 实测就是这么发生的：
    重启服务 → _kill_remote_drive() 杀掉了发布器 → 陈旧标签页的保活立刻重建 →
    drive_loop 模块级的 arm_now() 抢下 priority=80 的速度源 → 机器人从趴着站了起来。

    所以挂速度源必须是**显式**的（/api/drive/arm 走 drive_line → ensure_drive），
    保活只能沿用已经存在的通道；通道没了就当这帧没发，什么都不做。
    这样「谁在控制机器人」永远对应一次真实的用户操作，而不是某个忘了关的标签页。
    """
    global _drive_proc
    with _drive_lock:
        proc = _drive_proc
    if proc is None or proc.poll() is not None or proc.stdin is None:
        return False
    try:
        proc.stdin.write(f"{fwd:.3f} {lat:.3f} {yaw:.3f}\n")
        proc.stdin.flush()
        return True
    except BrokenPipeError as exc:
        with _drive_lock:
            _drive_proc = None
        raise HTTPException(502, "drive loop died") from exc


def _kill_remote_cmd() -> None:
    subprocess.run(
        _ssh_argv(
            "python3 - <<'PY'\n"
            "import os, signal\n"
            "for line in os.popen('ps -eo pid,cmd'):\n"
            "    if 'q1_cmd_loop.py' in line:\n"
            "        pid = int(line.split(None, 1)[0])\n"
            "        if pid != os.getpid():\n"
            "            try: os.kill(pid, signal.SIGTERM)\n"
            "            except OSError: pass\n"
            "print('cmd_killed')\n"
            "PY"
        ),
        timeout=8,
        env=_ssh_env(),
        capture_output=True,
        text=True,
    )


def _eat_cmd(proc: subprocess.Popen) -> None:
    assert proc.stdout is not None
    for line in proc.stdout:
        s = (line or "").rstrip()
        if not s:
            continue
        with _cmd_lock:
            _cmd_log.append(s)
            if len(_cmd_log) > 80:
                del _cmd_log[:-40]
        _cmd_ev.set()


def ensure_cmd() -> subprocess.Popen:
    global _cmd_proc
    _require_auth()
    with _cmd_lock:
        if _cmd_proc is not None and _cmd_proc.poll() is None:
            return _cmd_proc
        _kill_remote_cmd()
        up = subprocess.run(
            _ssh_argv("cat > /tmp/q1_cmd_loop.py"),
            input=(ROOT / "cmd_loop.py").read_text(),
            text=True,
            timeout=12,
            env=_ssh_env(),
            capture_output=True,
        )
        if up.returncode != 0:
            raise HTTPException(502, (up.stderr or up.stdout or "upload cmd loop failed")[-800:])
        proc = subprocess.Popen(
            _ssh_argv(f"{ROS_BASH}\npython3 -u /tmp/q1_cmd_loop.py"),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=_ssh_env(),
            bufsize=1,
        )
        ready = False
        assert proc.stdout is not None
        deadline = time.time() + 20
        while time.time() < deadline:
            line = proc.stdout.readline()
            if "CMD_READY" in (line or ""):
                ready = True
                break
            if proc.poll() is not None:
                rest = proc.stdout.read() if proc.stdout else ""
                raise HTTPException(502, ((line or "") + rest)[-800:])
        if not ready:
            proc.kill()
            raise HTTPException(504, "cmd loop did not start")
        threading.Thread(target=_eat_cmd, args=(proc,), daemon=True).start()
        _cmd_proc = proc
        return proc


def cmd_exec(line: str, timeout: float = 6.0) -> str:
    proc = ensure_cmd()
    if proc.stdin is None:
        raise HTTPException(500, "cmd stdin closed")
    _cmd_ev.clear()
    with _cmd_lock:
        before = len(_cmd_log)
    try:
        proc.stdin.write(line.rstrip() + "\n")
        proc.stdin.flush()
    except BrokenPipeError as exc:
        global _cmd_proc
        with _cmd_lock:
            _cmd_proc = None
        raise HTTPException(502, "cmd loop died") from exc
    deadline = time.time() + timeout
    while time.time() < deadline:
        with _cmd_lock:
            new = _cmd_log[before:]
        hits = [x for x in new if x.startswith("CMD_OK") or x.startswith("CMD_ERR")]
        if hits:
            return hits[-1]
        _cmd_ev.wait(0.15)
        _cmd_ev.clear()
    raise HTTPException(504, "指令已进常驻通道，机器人这几秒没回")


def drive_line(line: str) -> None:
    proc = ensure_drive()
    if proc.stdin is None:
        raise HTTPException(500, "drive stdin closed")
    try:
        proc.stdin.write(line.rstrip() + "\n")
        proc.stdin.flush()
    except BrokenPipeError as exc:
        global _drive_proc
        with _drive_lock:
            _drive_proc = None
        raise HTTPException(502, "drive loop died") from exc


def ssh(remote: str, timeout: int = 20) -> subprocess.CompletedProcess:
    _require_auth()
    return subprocess.run(
        _ssh_argv(remote),
        capture_output=True,
        text=True,
        timeout=timeout,
        env=_ssh_env(),
    )


def ssh_ok() -> bool:
    try:
        p = ssh("echo OK", timeout=10)
        return p.returncode == 0 and "OK" in (p.stdout or "")
    except Exception:
        return False


def ros_exec(body: str, timeout: int = 25) -> str:
    script = REMOTE_PY_PREFIX + "\n" + body
    try:
        p = ssh(f"{ROS_BASH}\npython3 - <<'PY'\n{script}\nPY", timeout=timeout)
    except subprocess.TimeoutExpired:
        raise HTTPException(
            504,
            f"已经发出去了，SSH 等了 {timeout} 秒机器人没回。不是没点到。",
        )
    out = ((p.stdout or "") + (p.stderr or "")).strip()
    if p.returncode != 0:
        raise HTTPException(502, out[-2000:] or "ssh/ros failed")
    return out


DAEMON_PORT = 8799
DAEMON_VER = 17
_snap_lock = threading.Lock()
_snap_started = False
_daemon_lock = threading.Lock()
_daemon_boot_t = 0.0
_live_lock = threading.Lock()
_live_cache: dict = {"online": False}
_live_ev = threading.Event()
_latest_jpg = b""
_latest_n = 0
_latest_lock = threading.Lock()
_latest_tick = threading.Event()
_cam_proc: subprocess.Popen | None = None
_JPEG_SOI = b"\xff\xd8"
_JPEG_EOI = b"\xff\xd9"


def _io_timeout(fast: float, slow: float) -> float:
    return slow if _route()[1] else fast


def _daemon_get(path: str, timeout: float | None = None) -> dict:
    _host, via_ts = _route()
    dest = f"127.0.0.1:{TS_HTTP_LOCAL}" if via_ts else f"{LAN_HOST}:{DAEMON_PORT}"
    url = f"http://{dest}{path}"
    wait = _io_timeout(1.2, 4.0) if timeout is None else timeout
    with urllib.request.urlopen(url, timeout=wait) as resp:
        return json.loads(resp.read().decode())


def _daemon_fresh() -> bool:
    try:
        h = _daemon_get("/health", timeout=_io_timeout(0.3, 2.5))
        return bool(h.get("ok")) and int(h.get("v") or 0) >= DAEMON_VER
    except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError, TypeError, ValueError):
        return False


def ensure_live_daemon() -> None:
    global _daemon_boot_t
    if _daemon_fresh():
        return
    with _daemon_lock:
        if _daemon_fresh():
            return
        now = time.time()
        if now - _daemon_boot_t < 60:
            return
        _daemon_boot_t = now
        _require_auth()
        for name in ("live_probe.py", "live_daemon.py", "mic_pump.py"):
            up = subprocess.run(
                _ssh_argv(f"cat > /tmp/{name}"),
                input=(ROOT / name).read_text(),
                text=True,
                timeout=25,
                env=_ssh_env(),
                capture_output=True,
            )
            if up.returncode != 0:
                raise HTTPException(502, (up.stderr or up.stdout or f"upload {name} failed")[-800:])
        boot = (
            "code=$(curl -sS -m 1 http://127.0.0.1:8799/health 2>/dev/null || true); "
            # 版本号从 DAEMON_VER 插值，别再手写死 —— 写死过一次，改版本时漏了这行，
            # 结果守护进程每次短暂掉线都会被「重启一遍」而不是认出它已经是对的。
            f"echo \"$code\" | grep -q '\"v\": {DAEMON_VER}' && echo ALREADY && exit 0; "
            "fuser -k 8799/tcp >/dev/null 2>&1 || true; sleep 0.3; "
            f"{ROS_BASH}\n"
            "cd /tmp && nohup python3 -u /tmp/live_daemon.py >/tmp/q1_live_daemon.log 2>&1 & "
            "echo STARTED $!\n"
        )
        subprocess.run(_ssh_argv(boot), env=_ssh_env(), timeout=20, capture_output=True, text=True)
        deadline = time.time() + 18
        while time.time() < deadline:
            if _daemon_fresh():
                return
            time.sleep(0.4)
        raise HTTPException(504, "live daemon did not start")


def _decorate_live(data: dict) -> dict:
    if "heard" not in data and data.get("source") != "daemon":
        data["online"] = False
        data["host"] = _route()[0]
        return data
    data["faces"] = facebook.attach(data.get("faces") or [])
    data["online"] = True
    data["host"] = _route()[0]
    data["filled"] = live_to_filled(data, data.get("meta") or {})
    return data


def _set_live(data: dict) -> None:
    with _live_lock:
        _live_cache.clear()
        _live_cache.update(data)
    _live_ev.set()


def live_pump() -> None:
    while True:
        try:
            if not _daemon_fresh():
                ensure_live_daemon()
            raw = _daemon_get("/live", timeout=_io_timeout(0.5, 4.0))
            _set_live(_decorate_live(raw))
        except Exception:
            with _live_lock:
                _live_cache["online"] = False
            _live_ev.set()
        time.sleep(0.08)


def collect_live() -> dict:
    with _live_lock:
        return dict(_live_cache)


def _rtsp_url() -> str:
    _host, via_ts = _route()
    if via_ts:
        return f"rtsp://127.0.0.1:{TS_RTSP_LOCAL}/{RTSP_PATH}"
    return f"rtsp://{LAN_HOST}:2554/{RTSP_PATH}"


def _publish_jpg(jpg: bytes) -> None:
    global _latest_jpg, _latest_n
    if len(jpg) < 20:
        return
    with _latest_lock:
        _latest_jpg = jpg
        _latest_n += 1
    _latest_tick.set()
    # overwrite one fallback file at ~1 Hz, never pile frames on disk.
    # 12 是跟着相机 12 fps 写的；相机现在是源帧率直通（实测约 30），
    # 不改这行就变成 2.5 Hz 白写盘。
    if _latest_n % 30 == 1:
        try:
            (ROOT / ".cache" / "head.jpg").write_bytes(jpg)
        except OSError:
            pass


def _kill_stale_ffmpeg() -> None:
    global _cam_proc
    if _cam_proc is not None and _cam_proc.poll() is None:
        _cam_proc.kill()
        _cam_proc = None
    # Windows 上没有 pkill。以前这里直接 subprocess.run(["pkill", ...]) 会抛
    # FileNotFoundError，而它是 _cam_loop 的第一句，摄像头线程就此死掉——
    # 表现为「页面能开、数据在刷、就是没有画面」。
    if os.name == "nt":
        cmd = ["taskkill", "/F", "/IM", "ffmpeg.exe"]
    else:
        cmd = ["pkill", "-f", f"{RTSP_PATH} -frames:v 1"]
    try:
        subprocess.run(cmd, capture_output=True, check=False, timeout=5)
    except (OSError, subprocess.SubprocessError):
        pass


def _cam_loop() -> None:
    global _cam_proc
    (ROOT / ".cache").mkdir(exist_ok=True)
    _kill_stale_ffmpeg()
    while True:
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-rtsp_transport",
            "tcp",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-i",
            _rtsp_url(),
            "-an",
            # 原本这里硬写着 "-r", "12"。12 是当初随手定的上限，不是源的上限。
            # 转码在**本机**做（RTSP 从隧道那头拉过来），不占机器人 CPU，
            # 所以放宽它不必向那台负载 20+ 的机器要预算。先不加 -r 直通，
            # 量出源真实帧率再决定卡在多少 —— 卡在源之上只会复制帧、白费带宽。
            "-q:v",
            "6",
            "-f",
            "image2pipe",
            "-vcodec",
            "mjpeg",
            "-",
        ]
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
            _cam_proc = proc
            assert proc.stdout is not None
            buf = b""
            while True:
                chunk = proc.stdout.read(8192)
                if not chunk:
                    break
                buf += chunk
                while True:
                    start = buf.find(_JPEG_SOI)
                    if start < 0:
                        buf = buf[-1:] if buf else b""
                        break
                    end = buf.find(_JPEG_EOI, start + 2)
                    if end < 0:
                        buf = buf[start:]
                        break
                    _publish_jpg(buf[start : end + 2])
                    buf = buf[end + 2 :]
            proc.wait(timeout=2)
        except Exception:
            pass
        time.sleep(0.5)


def start_snapper() -> None:
    global _snap_started
    with _snap_lock:
        if _snap_started:
            return
        threading.Thread(target=_cam_loop, daemon=True).start()
        _snap_started = True


def live_to_filled(live: dict, meta: dict) -> dict:
    bat = live.get("battery") or {}
    aud = live.get("audio") or {}
    pb = live.get("playback") or {}
    imu = live.get("imu") or {}
    act = live.get("action") or {}
    led = live.get("led") or {}
    faces = live.get("faces") or []
    mc = live.get("mc") or {}
    body = live.get("body") or {}
    vel = live.get("vel") or {}
    heard = set(live.get("heard") or [])

    bat_txt = (
        f"{bat['percent']}%　{bat['voltage_v']} V　{bat['current_a']} A　{bat['temp_c']} °C　循环 {bat['cycles']}"
        if bat
        else "这几秒没刷到"
    )
    aud_txt = (
        f"{aud['sample_rate']/1000:.0f} kHz　{aud['channels']} 声道　麦 RMS {aud['rms']}"
        if aud
        else "这几秒没刷到"
    )
    pb_txt = f"{pb.get('pkg') or '扬声器'} RMS {pb['rms']}" if pb else "这几秒没在播"
    face_txt = (
        f"{len(faces)} 张　" + "；".join((f.get("name") or f.get("nickname") or "未命名") for f in faces[:4])
        if faces
        else "0 张（头朝地/没人时正常）"
    )
    imu_txt = f"滚转 {imu['roll']}°　俯仰 {imu['pitch']}°　偏航 {imu['yaw']}°" if imu else "这几秒没刷到"
    led_txt = f"模式 {led.get('mode')}　RGB {led.get('r')},{led.get('g')},{led.get('b')}" if led else "这几秒没刷到"
    emo = (live.get("display") or {}).get("emoticon_id")
    mc_txt = (
        f"{'摔倒' if mc.get('fallen') else '没摔'}　{'在动' if mc.get('moving') else '静止'}　{mc.get('action') or ''}　步 {mc.get('steps')}"
        if mc
        else "这几秒没刷到"
    )
    return {
        "link": {
            "SSH": "已接通",
            "Wi‑Fi": (live.get("net") or {}).get("ssid") or meta.get("WIFI") or "未知",
            "Wi‑Fi信号": (live.get("net") or {}).get("signal"),
            "地址": meta.get("IP") or _route()[0],
            "负载": meta.get("LOAD") or "",
            "模式": (meta.get("MODE") or "").replace("export RUNNING_MODE=", "").strip() or "未知",
            "固件": meta.get("FW") or "未知",
            "序列号": meta.get("SN") or "未知",
        },
        "power": {"电池": bat_txt},
        "motion": {
            "当前 Action": f"{act.get('name') or mc.get('action') or '未读到'}（{act.get('id', mc.get('action_id', '?'))}）",
            "运控": mc_txt,
            "身体": body or "这几秒没刷到",
            "速度": vel or "这几秒没刷到",
            "躯干姿态": imu_txt,
            "里程计": live.get("odom") or "趴着时可能不刷",
            "轨迹点": len(live.get("traj") or []),
            "上身关节": f"{len(live.get('joints') or [])} 个",
            "形态舵机": live.get("servo") or "这几秒没刷到",
        },
        "sense": {
            "头摄": "已截一帧，见上方画面",
            "麦克风": aud_txt,
            "扬声器": pb_txt,
            "摸头": live.get("touch") or "没摸到就不发",
            "防摔区": live.get("fall"),
        },
        "see": {
            "人脸": face_txt,
            "相机模块": live.get("camera") or "这几秒没刷到",
            "读不出内容的": "；".join(live.get("blocked") or []),
        },
        "nav": {
            "跟随": (live.get("follow") or {}).get("label") or "没开任务，话题空着",
            "跟随候选": len(live.get("candidates") or []),
            "SLAM": (live.get("slam") or {}).get("msg") or "没开建图/定位，话题空着",
            "地图": (live.get("map") or {}).get("current") or "还没存图",
            "导航": (live.get("navi") or {}).get("label") or "空闲/没刷",
            "导航点": len(live.get("points") or []),
            "避障": (live.get("fusion") or {}).get("label") or "没刷到",
            "充电桩": (live.get("charge") or {}).get("label") or "没刷到",
            "Insta360": live.get("insta") or "节点在，没任务",
        },
        "talk": {
            "音量": live.get("volume"),
            "静音": "是" if live.get("mute") else "否",
            "脸屏表情": emo,
            "灯语": led_txt,
            "正在说": (live.get("tts") or {}).get("text") or "没有",
            "字幕": (live.get("show") or {}).get("text") or "没有",
            "对话态": live.get("conv") or "官方对话关着",
            "Agent 意图": live.get("intent") or "空",
        },
        "task": {
            "task_engine": live.get("task") or "这几秒没任务包",
            "智能动作": live.get("imot") or "这几秒没刷到",
            "交互": live.get("ix") or "这几秒没刷到",
            "告警": f"{len(live.get('alerts') or [])} 条",
            "诊断": f"{len(live.get('diags') or [])} 条",
        },
        "sys": {
            "系统": live.get("sm") or "这几秒没刷到",
            "机器人开关": live.get("robot") or "这几秒没刷到",
            "场景": live.get("scene") or "",
            "事件": live.get("events"),
            "模块": (meta.get("APPS") or "")[:240],
        },
    }


@app.get("/")
def index():
    return FileResponse(
        ROOT / "static" / "index.html",
        headers={"Cache-Control": "no-store"},
    )


@app.get("/api/health")
def health():
    with _live_lock:
        online = bool(_live_cache.get("online"))
    return {
        "online": online,
        "host": _route()[0],
        "user": USER,
        "rtsp": _rtsp_url(),
        "need_dev_mode_for_this_gui": False,
        "live_source": "pump" if online else "down",
    }


CATALOG = [
    {
        "id": "link",
        "title": "连接 / 模式",
        "items": ["SSH", "Wi‑Fi 地址", "开发者模式", "固件", "序列号"],
    },
    {
        "id": "power",
        "title": "电源",
        "items": ["电池 / BMS", "充电", "保护位"],
    },
    {
        "id": "motion",
        "title": "运控",
        "items": ["当前 Action", "是否可切动作", "步态", "关节摘要", "形态舵机"],
    },
    {
        "id": "sense",
        "title": "本体传感",
        "items": ["头摄 RTSP / 单帧 JPEG", "麦克风码流（在不在）", "里程计 odom / 躯干", "IMU", "摸头"],
    },
    {
        "id": "see",
        "title": "视觉算法",
        "items": ["跟随时状态 / 候选", "人脸 face_info", "YOLOv8（模块在机上）", "Insta360 节点"],
    },
    {
        "id": "nav",
        "title": "建图 / 定位 / 导航",
        "items": ["SLAM 状态", "房间底图 PNG", "导航点", "定位任务", "避障方向", "充电桩识别"],
    },
    {
        "id": "talk",
        "title": "交互",
        "items": ["音量 / 静音", "TTS", "表情 / 脸屏", "灯语", "官方 Agent 意图"],
    },
    {
        "id": "task",
        "title": "任务",
        "items": ["task_engine", "跟随任务", "导航任务"],
    },
    {
        "id": "sys",
        "title": "系统",
        "items": ["模块列表", "网络 / 热点", "健康告警"],
    },
]


@app.get("/api/catalog")
def catalog():
    return {"sections": CATALOG}


WATCH_TOPICS = [
    "/aima/hal/audio/capture",
    "/odom",
    "/odom_torso",
    "/torso_imu_orientation",
    "/aima/hal/touch/state",
    "/aima/hal/bms/state",
    "/aima/algo/follow/state",
    "/perception/follow_candidates",
    "/aima/agent/face_info",
    "/slam_state",
    "/aima/map/state",
    "/aima/algo/navi/state",
    "/aima/display_state",
    "/aima/hal/rgb/state",
    "/aima/agent/semantic_intent",
    "/aima/task_engine/task_status",
]


@app.get("/api/status")
def status():
    try:
        live = collect_live()
    except Exception:
        return {"online": False, "volume": None, "action": None, "switch_ready": None, "filled": {}}
    meta = live.get("meta") or {}
    action = live.get("action")
    return {
        "online": True,
        "volume": live.get("volume"),
        "action": action,
        "switch_ready": (action or {}).get("switch_ready"),
        "filled": live_to_filled(live, meta),
        "live": live,
    }


@app.get("/api/live")
def live():
    return collect_live()


@app.websocket("/ws/live")
async def live_ws(ws: WebSocket):
    await ws.accept()
    last = ""
    try:
        while True:
            await asyncio.to_thread(_live_ev.wait, 0.12)
            _live_ev.clear()
            with _live_lock:
                payload = dict(_live_cache)
            blob = json.dumps(payload, ensure_ascii=False)
            if blob != last:
                await ws.send_text(blob)
                last = blob
    except WebSocketDisconnect:
        return


@app.get("/api/people")
def people_list():
    return {"ok": True, "people": facebook.load()}


@app.post("/api/people")
def people_keep(body: PersonIn):
    pid = (body.id or "").strip() or f"face-{secrets.token_hex(4)}"
    row = facebook.upsert(
        pid,
        name=body.name if body.name else None,
        image_b64=body.image,
        box={"x": body.x, "y": body.y, "w": body.w, "h": body.h} if body.w and body.h else None,
    )
    return {"ok": True, "person": row}


@app.post("/api/people/{pid}/name")
def people_name(pid: str, body: PersonIn):
    facebook.upsert(pid, name=body.name)
    row = next((r for r in facebook.load() if r.get("id") == pid), None)
    return {"ok": True, "person": row}


@app.delete("/api/people/{pid}")
def people_delete(pid: str):
    if not facebook.delete(pid):
        raise HTTPException(404, "没有这个人")
    return {"ok": True}


@app.get("/api/people/{pid}.jpg")
def people_thumb(pid: str):
    path = facebook.thumb_path(pid)
    if path is None:
        raise HTTPException(404, "还没留下这张脸")
    return FileResponse(path, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/api/snapshot.jpg")
def snapshot():
    start_snapper()
    path = ROOT / ".cache" / "head.jpg"
    deadline = time.time() + 6
    while time.time() < deadline and not path.exists():
        time.sleep(0.1)
    if not path.exists():
        raise HTTPException(504, "头摄还没出第一帧")
    return FileResponse(path, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/api/cam.mjpg")
def cam_mjpeg():
    start_snapper()
    boundary = "q1frame"
    deadline = time.time() + 8
    while time.time() < deadline:
        with _latest_lock:
            if _latest_jpg:
                break
        time.sleep(0.05)

    def gen():
        last_n = -1
        while True:
            with _latest_lock:
                n = _latest_n
                cur = _latest_jpg
            if cur and n != last_n:
                last_n = n
                header = (
                    f"--{boundary}\r\nContent-Type: image/jpeg\r\n"
                    f"Content-Length: {len(cur)}\r\n\r\n"
                ).encode()
                yield header + cur + b"\r\n"
            else:
                _latest_tick.wait(timeout=0.6)
                _latest_tick.clear()

    return StreamingResponse(
        gen(),
        media_type=f"multipart/x-mixed-replace; boundary={boundary}",
        headers={"Cache-Control": "no-store, no-cache", "X-Accel-Buffering": "no"},
    )


@app.post("/api/speak")
def speak(body: SpeakIn):
    text = body.text.replace("\n", " ").strip()
    out = cmd_exec("SAY " + text, timeout=8)
    return {"ok": "CMD_OK SAY True" in out, "detail": out}


def _daemon_conn(path: str, method: str):
    """连机器人守护进程的一个裸 socket（走 tailscale 转发口或局域网直连）。"""
    _host, via_ts = _route()
    dest = ("127.0.0.1", TS_HTTP_LOCAL) if via_ts else (LAN_HOST, DAEMON_PORT)
    sock = socket.create_connection(dest, timeout=8.0)
    sock.sendall(
        ("%s %s HTTP/1.0\r\nHost: q1\r\nConnection: close\r\n\r\n" % (method, path)).encode()
    )
    return sock


def _read_http_json(sock: socket.socket, timeout: float = 6.0) -> dict:
    """从这个裸 socket 上读一个 JSON 响应。

    必须按 Content-Length 把正文读满：正文和响应头分两个包到时，
    只切到第一个 \\r\\n\\r\\n 会拿到半截 JSON，json.loads 抛错就被当成「没有回执」，
    症状是「明明送到了却显示没送到」。测试里就是这么复现的。
    """
    sock.settimeout(timeout)
    buf = b""
    while b"\r\n\r\n" not in buf and len(buf) < 8192:
        part = sock.recv(4096)
        if not part:
            return {}
        buf += part
    head, _, body = buf.partition(b"\r\n\r\n")
    need = 0
    for line in head.split(b"\r\n"):
        if line.lower().startswith(b"content-length:"):
            try:
                need = int(line.split(b":", 1)[1])
            except ValueError:
                need = 0
    while len(body) < need:
        part = sock.recv(4096)
        if not part:
            break
        body += part
    try:
        return json.loads(body.decode() or "{}")
    except ValueError:
        return {}


@app.websocket("/ws/talk")
async def talk_ws(ws: WebSocket):
    """实时对讲。一条 WebSocket 同时走两个方向：

      浏览器 → 二进制帧 → 上行 PCM（24 kHz 单声道 S16LE）→ 机器人喇叭
      机器人麦克风 → 二进制帧 → 浏览器

    为什么是 WebSocket 而不是 fetch 流式 POST：**浏览器根本不支持流式请求体**。
    `duplex:"half"` 这个成员存在、也不报参数错，但请求一律以
    `TypeError: Failed to fetch` 失败 —— 在 Chromium 148 / Chrome 151 / Edge 152 上
    都实测过。于是改成每条按住开一条 WS，二进制帧直接 ws.send()，各浏览器都支持。

    上行一次按住 = 一条到守护进程 /play.pcm 的长连接 = 一个 token。
    token 必须整段按住保持不变：变了会清 HAL 的播放缓存，把机器人当前说的话打断。
    """
    await ws.accept()
    up = None
    mic_task = None
    stop = asyncio.Event()
    sent = 0

    async def end_uplink():
        """收尾这一轮按住：补 0 长度结束帧、读守护进程回执、关连接。
        返回 (发出字节数, 回执)。下一次按住会重开连接 = 换 token = 打断机器人当前说话。
        """
        nonlocal up, sent
        if up is None:
            return 0, {}
        sock, up = up, None
        mine, sent = sent, 0
        reply: dict = {}
        try:
            await asyncio.to_thread(sock.sendall, b"\x00\x00\x00\x00")
            # 超时给 30 秒：实测网络降速时（Tailscale 抖动），3 块 0.3 秒的 PCM
            # 可能花 6 秒多才到齐。6 秒就放弃会提前关 socket，把还在路上的
            # 播放截断 —— HAL 日志里只有第一块播出来（实测 3 丢 2）。
            reply = await asyncio.to_thread(_read_http_json, sock, 30.0)
        except (OSError, RuntimeError):
            reply = {}
        finally:
            try:
                sock.close()
            except OSError:
                pass
        return mine, reply

    async def pump_mic(on: bool):
        """开关下行。"""
        nonlocal mic_task
        if on and mic_task is None:
            stop.clear()
            mic_task = asyncio.create_task(mic_pump())
            print("TALK listen on", flush=True)
        elif not on and mic_task is not None:
            stop.set()
            mic_task.cancel()
            mic_task = None
            print("TALK listen off", flush=True)

    async def mic_pump():
        """下行：守护进程 /mic.pcm 的分帧流 → 原样转成二进制帧发给浏览器。

        注意开头那段 HTTP 响应头要先吃掉：这是个裸 socket，不是 requests。
        不剥头就会把 "HTTP/1.0 200 OK" 当成长度前缀，于是永远在等一个天文数字的负载。
        （第一版就是这么写的，症状是「守护进程明明在推、浏览器一块都收不到」。）
        """
        sock = None
        try:
            sock = await asyncio.to_thread(_daemon_conn, "/mic.pcm", "GET")

            def _read_head() -> bytes:
                sock.settimeout(8.0)
                head = b""
                while b"\r\n\r\n" not in head and len(head) < 8192:
                    part = sock.recv(4096)
                    if not part:
                        break
                    head += part
                return head.partition(b"\r\n\r\n")[2]

            buf = await asyncio.to_thread(_read_head)
            fwd = 0
            while not stop.is_set():
                while len(buf) >= 4:
                    n = int.from_bytes(buf[:4], "big")
                    if len(buf) < 4 + n:
                        break
                    blk, buf = buf[4:4 + n], buf[4 + n:]
                    await ws.send_bytes(blk)
                    fwd += 1
                part = await asyncio.to_thread(sock.recv, 8192)
                if not part:
                    break
                buf += part
            print("TALK down end: fwd=%d blocks" % fwd, flush=True)
        except (OSError, RuntimeError, WebSocketDisconnect):
            print("TALK down error", flush=True)
        finally:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
            stop.set()

    try:
        while True:
            msg = await ws.receive()
            if msg.get("type") == "websocket.disconnect":
                break
            data = msg.get("bytes")
            if data:
                if up is None:
                    try:
                        up = await asyncio.to_thread(_daemon_conn, "/play.pcm", "POST")
                    except OSError as exc:
                        await ws.send_json({"err": "连不上机器人对讲口 — %s" % exc})
                        break
                # 4 字节大端长度 + 负载。隔着 tailscale nc，它不透传 TCP 半关，
                # 所以不能用关连接定界，收尾要显式发一个 0 长度帧。
                await asyncio.to_thread(
                    up.sendall, len(data).to_bytes(4, "big") + data
                )
                sent += len(data)
                continue
            txt = msg.get("text")
            if not txt:
                continue
            try:
                cmd = json.loads(txt)
            except ValueError:
                continue
            if "listen" in cmd:
                print("TALK cmd listen=%s" % cmd["listen"], flush=True)
                await pump_mic(bool(cmd["listen"]))
            elif cmd.get("end"):
                # 松开说话。**不关 WebSocket** —— 收听可能还开着，
                # 关掉会把下行一起掐断（一开始就是这么写的，会留一个空洞）。
                n, reply = await end_uplink()
                print("TALK up end: bytes=%d reply=%s" % (n, reply), flush=True)
                try:
                    await ws.send_json({"sent": n, "robot": reply})
                except (WebSocketDisconnect, RuntimeError):
                    break
    except WebSocketDisconnect:
        pass
    finally:
        stop.set()
        if mic_task is not None:
            mic_task.cancel()
        await end_uplink()


@app.post("/api/mute")
def mute(body: MuteIn):
    out = cmd_exec("MUTE 1" if body.mute else "MUTE 0", timeout=6)
    parts = out.split()
    now = None
    if len(parts) >= 3 and parts[2] in ("True", "False"):
        now = parts[2] == "True"
    return {"ok": out.startswith("CMD_OK"), "mute": now if now is not None else body.mute, "detail": out}


@app.post("/api/stop_talk")
def stop_talk():
    out = cmd_exec("HUSH", timeout=5)
    return {"ok": out.startswith("CMD_OK"), "detail": out}


@app.post("/api/volume")
def volume(body: VolumeIn):
    out = cmd_exec(f"VOL {int(body.volume)}", timeout=5)
    return {"ok": out.startswith("CMD_OK"), "detail": out}


@app.post("/api/emotion")
def emotion(body: EmotionIn):
    out = cmd_exec(f"EMO {int(body.emotion_id)}", timeout=5)
    return {"ok": out.startswith("CMD_OK"), "detail": out}


@app.post("/api/light")
def light(body: LightIn):
    out = cmd_exec(
        f"LIGHT {int(body.mode)} {int(body.r)} {int(body.g)} {int(body.b)} {int(body.period)}",
        timeout=5,
    )
    return {"ok": out.startswith("CMD_OK"), "detail": out}


@app.post("/api/motion")
def motion(body: MotionIn):
    out = cmd_exec(f"MOTION {int(body.motion_id)}", timeout=8)
    return {"ok": out.startswith("CMD_OK MOTION 0"), "detail": out}


@app.post("/api/motion/custom")
def motion_custom(body: CustomMotionIn):
    """放自录动作。跟 /api/motion 不是一条路：自录动作没有内置编号，
    走 PlayCustomMotion（resource_type + resource_key）。"""
    out = cmd_exec(f"CMOTION {int(body.rtype)} {body.name}", timeout=8)
    return {"ok": "CMD_OK CMOTION 0" in out, "detail": out}


@app.post("/api/mc_action")
def mc_action(body: ActionIn):
    out = cmd_exec(f"ACTION {int(body.action_id)}", timeout=10)
    return {"ok": "CMD_OK ACTION True" in out, "detail": out}


@app.post("/api/follow")
def follow(body: FollowIn):
    if body.on and not body.watch:
        raise HTTPException(403, "跟人走要先勾「我站在旁边」。")
    if body.kind == "perc":
        out = cmd_exec(f"FPERC {'1' if body.on else '0'}", timeout=6)
        return {"ok": out.startswith("CMD_OK FPERC"), "detail": out}
    tid = max(0, int(body.target_id))
    out = cmd_exec(f"FOLLOW {'1' if body.on else '0'} {tid}", timeout=8)
    return {"ok": out.startswith("CMD_OK FOLLOW"), "detail": out}


@app.post("/api/map")
def map_cmd(body: MapCmdIn):
    cmd = (body.cmd or "").strip().lower()
    if cmd not in ("start", "startauto", "save", "cancel"):
        raise HTTPException(400, "地图命令只能是 start / startauto / save / cancel")
    if cmd in ("start", "startauto") and not body.watch:
        raise HTTPException(403, "建图要先勾「我站在旁边」。")
    name = (body.name or "").strip().replace(" ", "_")[:40]
    out = cmd_exec(f"MAP {cmd} {name}".strip(), timeout=10)
    return {"ok": out.startswith("CMD_OK MAP"), "detail": out}


@app.post("/api/navi")
def navi_cmd(body: NaviIn):
    cmd = (body.cmd or "").strip().lower()
    if cmd in ("name", "xy", "charge") and not body.watch:
        raise HTTPException(403, "去一个地方要先勾「我站在旁边」。")
    if cmd == "name":
        name = (body.name or "").strip()[:40]
        if not name:
            raise HTTPException(400, "导航点名字是空的")
        out = cmd_exec(f"NAVI name {name}", timeout=10)
    elif cmd == "xy":
        out = cmd_exec(f"NAVI xy {body.x:.3f} {body.y:.3f} {body.theta:.3f}", timeout=10)
    elif cmd in ("stop", "cancel", "resume", "charge"):
        out = cmd_exec(f"NAVI {cmd}", timeout=8)
    else:
        raise HTTPException(400, "导航命令不对")
    return {"ok": out.startswith("CMD_OK NAVI"), "detail": out}


@app.post("/api/locate")
def locate_cmd(body: LocIn):
    if body.on and not body.watch:
        raise HTTPException(403, "开始定位要先勾「我站在旁边」。")
    mid = (body.map_id or "").strip()
    if body.on and not mid:
        raise HTTPException(400, "定位要一张图的 id")
    out = cmd_exec(f"LOC {'start' if body.on else 'stop'} {mid}".strip(), timeout=10)
    return {"ok": out.startswith("CMD_OK LOC"), "detail": out}


@app.post("/api/navi_point")
def navi_point(body: PointIn):
    if not body.watch:
        raise HTTPException(403, "打点要先勾「我站在旁边」。")
    mid = (body.map_id or "").strip()
    if not mid:
        raise HTTPException(400, "打点要先有一张图")
    name = (body.name or "这里").strip().replace(" ", "_")[:24]
    out = cmd_exec(f"POINT {mid} {name}", timeout=8)
    return {"ok": out.startswith("CMD_OK POINT"), "detail": out}


@app.get("/api/map.png")
def map_png():
    try:
        url = f"http://{'127.0.0.1:' + str(TS_HTTP_LOCAL) if _route()[1] else LAN_HOST + ':' + str(DAEMON_PORT)}/map.png"
        with urllib.request.urlopen(url, timeout=1.5) as resp:
            data = resp.read()
            code = resp.status
    except Exception:
        return StreamingResponse(iter([b""]), media_type="image/png", status_code=204)
    return StreamingResponse(iter([data]), media_type="image/png", status_code=code if data else 204)


@app.post("/api/drive")
def drive(body: DriveIn):
    # sent=False 只表示「这会儿没有速度通道」（没遥控过 / 已经被 disarm），
    # 不是错误：这个口同时也是 8.3 Hz 的保活，通道不在就是没事可做。
    sent = drive_send(body.fwd, body.lat, body.yaw)
    return {"ok": True, "sent": sent, "fwd": body.fwd, "lat": body.lat, "yaw": body.yaw}


@app.post("/api/drive/arm")
def drive_arm():
    _drive_log_ev.clear()
    drive_line("ARM")
    deadline = time.time() + 12
    last = ""
    while time.time() < deadline:
        with _drive_lock:
            hits = [x for x in _drive_log if x.startswith("INPUT_SRC")]
        if hits:
            last = hits[-1]
            ok = last.startswith("INPUT_SRC 0")
            return {"ok": ok, "armed": ok, "detail": last}
        _drive_log_ev.wait(0.25)
        _drive_log_ev.clear()
    raise HTTPException(504, last or "速度源没挂上")


@app.post("/api/drive/disarm")
def drive_disarm():
    """交还速度源并结束机器人那头的过程。

    这里**故意不用** drive_line()/drive_send()：那两个都走 ensure_drive()，
    而 ensure_drive() 是「没有就拉起一个」——
      · 先 DISARM 再 drive_send(0)：DISARM 刚让远端退出，这一句又把它拉回来，
        等于没关（原来就是这个问题）；
      · 冷启动时直接 post 一个 disarm：机器人上会平白多出一个 50 Hz 速度发布器。
    所以自己拿锁看一眼，活着才写；没活着就什么都不做。

    先落零速度再 DISARM：顺序反了的话，从收到 DISARM 到进程真正退出之间
    还可能发出去一帧非零速度。
    """
    with _drive_lock:
        proc = _drive_proc
    if proc is not None and proc.poll() is None and proc.stdin is not None:
        try:
            proc.stdin.write("0.000 0.000 0.000\nDISARM\n")
            proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass
    return {"ok": True, "armed": False}


@app.websocket("/ws/drive")
async def drive_ws(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            data = await ws.receive_json()
            def clamp(v, lo, hi):
                return max(lo, min(hi, float(v)))

            sent = await asyncio.to_thread(
                drive_send,
                clamp(data.get("fwd", 0), -0.4, 0.4),
                clamp(data.get("lat", 0), -0.3, 0.3),
                clamp(data.get("yaw", 0), -0.8, 0.8),
            )
            # 通道没了就直接断开，别让对面以为自己在开车。
            # 前端 onclose 会亮「速度通道断 · 重开遥控」—— 这才是实情：
            # 要重新控制机器人必须**重新按一次遥控**（显式挂速度源），
            # 而不是靠这个保活把通道悄悄续回来。
            if not sent:
                await ws.close(code=4001, reason="not armed")
                break
    except WebSocketDisconnect:
        try:
            drive_send(0.0, 0.0, 0.0)
        except Exception:
            pass


@app.get("/api/fleet")
def api_fleet():
    """全机群快照：在线、消息数、电量、当前动作。"""
    return mqtt_fleet.fleet.snapshot()


@app.websocket("/ws/fleet")
async def fleet_ws(ws: WebSocket):
    """机群状态推送。只在内容变化时发，30 秒没话说就发一次心跳。"""
    await ws.accept()
    last = None
    last_ping = time.monotonic()
    try:
        while True:
            snap = mqtt_fleet.fleet.snapshot()
            body = json.dumps(snap, ensure_ascii=False, sort_keys=True)
            now = time.monotonic()
            if body != last:
                await ws.send_text(body)
                last = body
                last_ping = now
            elif now - last_ping >= 30:
                await ws.send_text('{"ping":1}')
                last_ping = now
            await asyncio.sleep(1.0)
    except WebSocketDisconnect:
        pass


if __name__ == "__main__":
    import uvicorn

    try:
        _kill_remote_drive()
    except Exception:
        pass
    uvicorn.run("server:app", host="0.0.0.0", port=8788, reload=False)
