"""Always-on ROS cache + HTTP on :8799. /live is a prebuilt snapshot, never blocks."""
import collections
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, "/tmp")
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from aimdk_msgs.msg import AudioPlayback
from live_probe import (
    attach,
    build_payload,
    poll_maps,
    poll_motions,
    poll_room,
    poll_services,
    rms16,
)

PORT = 8799
VERSION = 17
# 上行对讲：浏览器 PCM → /aima/hal/audio/playback。HAL 订阅这个口是 RELIABLE，
# 用默认 QoS 发出去会静默丢弃。24 kHz 单声道 S16LE，浏览器按 4800 字节（0.1 秒）一块发。
TALK_RATE = 24000

# 下行对讲：机器人麦克风 → 浏览器。采集口的订阅不在本进程 —— 见 mic_pump.py
# 顶部注释：进程内订阅被主循环和 DDS 收包线程饿死（3.2 块/秒 vs 独立进程 17.5）。
# 本进程只拉 mic_pump 子进程、读它的 stdout 转发。24 kHz 单声道 S16LE。
_mic_lock = threading.Lock()
_mic_cond = threading.Condition(_mic_lock)
_mic_ring = collections.deque(maxlen=64)  # (seq, bytes)，64 块 = 2.56 秒
_mic_seq = 0
_mic_listeners = 0  # 没人在听就让泵停着，别白烧机器人 CPU
_pump = None  # mic_pump.py 子进程
_pump_state = "stop"

got = {}
svc = {"vol": None, "mute": None, "act": None}
lock = threading.Lock()
_pub = None
_node = None
_pub_lock = threading.Lock()
_snap = b'{"ok":true,"online":false}'
_snap_t = 0.0
_snap_build_lock = threading.Lock()
# 比本机 live_pump 的 0.08 秒略快一点点就够：再快也没人看，纯属白烧
SNAP_TTL = 0.07
_meta = {}
_meta_t = 0.0
_map_png = b""


def meta():
    global _meta, _meta_t
    now = time.time()
    if _meta and now - _meta_t < 5:
        return _meta

    def rd(path):
        try:
            return open(path, encoding="utf-8", errors="ignore").read().strip()
        except OSError:
            return ""

    mode = ""
    try:
        for line in open("/robot/software/entry/cfg/mode/mode_config.env.sh", encoding="utf-8"):
            if "RUNNING_MODE" in line:
                mode = line.strip()
                break
    except OSError:
        pass
    ip = _meta.get("IP", "")
    if now - _meta_t > 15 or not ip:
        try:
            import subprocess

            ip = subprocess.check_output(
                "ip -4 -br addr show wlan0 | awk '{print $3}'", shell=True, text=True
            ).strip()
        except Exception:
            ip = ip or ""
    _meta = {
        "SN": _meta.get("SN") or rd("/robot/protected/sn") or rd("/robot/info/sn"),
        "FW": _meta.get("FW") or rd("/robot/software/firmware_version"),
        "MODE": mode or _meta.get("MODE", ""),
        "IP": ip,
        "WIFI": os.environ.get("Q1_WIFI", ""),
    }
    _meta_t = now
    return _meta


def rebuild():
    global _snap, _snap_t, _map_png
    with lock:
        payload = build_payload(got, svc["vol"], svc["mute"], svc["act"])
        m = got.get("mapb")
        if m is not None and m.png_data:
            _map_png = bytes(m.png_data)
    payload["online"] = True
    payload["meta"] = meta()
    payload["source"] = "daemon"
    payload["v"] = VERSION
    _snap = json.dumps(payload, ensure_ascii=False).encode()
    _snap_t = time.time()


def snap() -> bytes:
    """拿当前快照，过期就重建 —— 按需，不是定时。

    以前是在 ROS 循环里每 0.05 秒无条件 rebuild()，可唯一的消费者
    （本机 server.py 的 live_pump）是每 0.08 秒来取一次：20 Hz 里有一半是白烧的，
    而且没人开控制台的时候也在烧。这台机器人 6 个核、负载已经 30+，
    平衡控制器 mc_main 常年在 99% —— 我们从它嘴里抢的每一分 CPU 都是它站不稳的理由。
    """
    if _snap_t and time.time() - _snap_t < SNAP_TTL:
        return _snap
    with _snap_build_lock:  # 并发请求别各自重建一遍
        if _snap_t and time.time() - _snap_t < SNAP_TTL:
            return _snap
        try:
            rebuild()
        except Exception as exc:
            print("REBUILD_ERR", str(exc)[:200], flush=True)
    return _snap


def pump_reader():
    """读 mic_pump 的 stdout：一行 JSON 头 + 无限 [4 字节大端长度 + PCM 帧]。
    帧进 _mic_ring，电平进 got["audio"]。泵死了：有人在听就重拉，没人听就随它去。"""
    global _mic_seq
    while True:
        p = _pump
        if p is None or p.stdout is None or p.poll() is not None:
            time.sleep(0.5)
            if _mic_listeners > 0:
                pump_ctl("go")  # 正在听的时候泵死了，自救重拉
            continue
        try:
            line = p.stdout.readline()
            if not line:
                continue
            info = json.loads(line)
            with lock:
                got["audio"] = {
                    "sample_rate": int(info.get("sr") or 48000),
                    "channels": int(info.get("ch") or 8),
                    "format": str(info.get("fmt") or "S16LE pcm"),
                    "bytes": int(info.get("msg_bytes") or 30720),
                    "rms": 0,
                }
            while True:
                hdr = p.stdout.read(4)
                if len(hdr) < 4:
                    break
                n = int.from_bytes(hdr, "big")
                if n <= 0:
                    break
                blk = p.stdout.read(n)
                if len(blk) < n:
                    break
                with lock:
                    got["audio"]["rms"] = rms16(blk)
                with _mic_cond:
                    _mic_seq += 1
                    _mic_ring.append((_mic_seq, blk))
                    _mic_cond.notify_all()
        except Exception:
            time.sleep(0.5)


def pump_ctl(state: str) -> None:
    """拉/控 mic_pump 子进程。GO = 建订阅开始流（有人听才烧 CPU），STOP = 毁订阅。
    泵死了就重拉。"""
    global _pump, _pump_state
    if state == _pump_state and _pump is not None and _pump.poll() is None:
        return
    if _pump is None or _pump.poll() is not None:
        _pump = subprocess.Popen(
            ["python3", "-u", "/tmp/mic_pump.py"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
    try:
        _pump.stdin.write(("GO\n" if state == "go" else "STOP\n").encode())
        _pump.stdin.flush()
        _pump_state = state
    except OSError:
        pass


def mic_blocks(next_seq: int, stop: threading.Event):
    """下游客户端用：从 next_seq 开始取块，落后太多就跳到最旧一块（丢帧好过卡死）。

    产出 (seq, bytes)；没数据就等条件变量，别空转。
    """
    while not stop.is_set():
        with _mic_cond:
            if not _mic_ring:
                _mic_cond.wait(0.5)
                continue
            first_seq = _mic_ring[0][0]
            if next_seq < first_seq:
                next_seq = first_seq  # 追不上了，丢掉中间那些
            hit = None
            for seq, blk in _mic_ring:
                if seq >= next_seq:
                    hit = (seq, blk)
                    break
            if hit is None:
                _mic_cond.wait(0.5)
                continue
            next_seq = hit[0] + 1
        yield hit


def play_pcm(data: bytes, token: str) -> bool:
    """把一块 PCM 推进播放环。token 变化会清播放缓存，用于打断机器人当前的说话。"""
    with _pub_lock:
        pub, node = _pub, _node
    if pub is None or node is None or not data:
        return False
    m = AudioPlayback()
    m.stamps = node.get_clock().now().to_msg()
    m.info.channels = 1
    m.info.sample_rate = TALK_RATE
    m.info.size = len(data)
    m.info.sample_format = "S16LE"
    m.info.coding_format = "pcm"
    m.data.data = list(data)
    m.pkg_name = "console"
    m.token_id = token
    pub.publish(m)
    return True


def ros_loop():
    global _map_png, _pub, _node
    rclpy.init()
    print("ROS_OK", rclpy.ok(), flush=True)
    n = Node("q1_live_daemon")
    print("NODE_OK", flush=True)
    attach(n, got)
    with _pub_lock:
        _node = n
        _pub = n.create_publisher(
            AudioPlayback,
            "/aima/hal/audio/playback",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
    # 采集口订阅不在本进程：mic_pump.py 独立进程，理由见它顶部注释。
    print("SUB_OK", flush=True)
    last_svc = 0.0
    last_maps = 0.0
    last_mots = 0.0
    # timeout 是**空闲**上限，不是处理节拍：有消息就立刻返回，所以调大不会加延迟。
    # 实测（6 核 q1，空节点纯 spin）：0.02 → 16.6% CPU，0.2 → 1.3%。
    # DDS 的等待粒度约 30 ms，所以 ≤0.05 的值彼此没区别，都是每秒 28~40 次空转。
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
        now = time.time()
        if now - last_svc > 4.0:
            try:
                vol, mute, act = poll_services(n)
                with lock:
                    svc["vol"], svc["mute"], svc["act"] = vol, mute, act
            except Exception:
                pass
            last_svc = now
        # 动作库是静态的，拿到之后两分钟问一次足够 —— 这台机器 6 个核、负载常年 20+，
        # 每次服务调用都是在跟平衡控制器抢 CPU。
        # 但**没拿到之前要勤问**：刚起进程时图谱还没建好（近 400 个服务），
        # 第一次 wait_for_service 一定是 False —— 实测就是空手而归，然后干等两分钟，
        # 用户那边「预设动作」就一直是写死的那 6 个。8 秒一轮，问到为止。
        if now - last_mots > (120.0 if got.get("_motions") else 8.0):
            try:
                mots = poll_motions(n)
                if mots and (mots["builtin"] or mots["custom"]):
                    with lock:
                        got["_motions"] = mots
            except Exception:
                pass
            last_mots = now
        if now - last_maps > 8.0:
            try:
                rows = poll_maps(n)
                current = ""
                st = got.get("mapst")
                if st is not None:
                    current = getattr(st, "current_map", "") or ""
                if rows:
                    hit = next((x for x in rows if x.get("id") == current or x.get("name") == current), None)
                    if hit:
                        current = hit.get("id") or current
                    elif not current:
                        current = rows[0].get("id") or ""
                info, points, png = poll_room(n, current) if current else (None, [], None)
                with lock:
                    got["_maps"] = rows
                    if info:
                        got["_map_info"] = info
                    got["_navi_points"] = points
                    if png:
                        _map_png = png
            except Exception:
                pass
            last_maps = now


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        if self.path.startswith("/mic.pcm"):
            self.stream_mic()
            return
        if self.path.startswith("/map.png"):
            body = _map_png or b""
            self.send_response(200 if body else 204)
            self.send_header("Content-Type", "image/png")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            if body:
                self.wfile.write(body)
            return
        if self.path in ("/health", "/"):
            body = json.dumps(
                {
                    "ok": True,
                    "v": VERSION,
                    "talk": _pub is not None,
                    "mic": _mic_listeners,
                    "pump": _pump is not None and _pump.poll() is None,
                }
            ).encode()
        elif self.path.startswith("/live"):
            body = snap()
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except BrokenPipeError:
            return

    def stream_mic(self):
        """下行对讲：把麦克风 PCM 推给下游，直到对面断开。

        分帧 = 4 字节大端长度 + 负载，和上行同一个约定：中间隔着 tailscale nc，
        它不透传 TCP 半关，所以不能靠关连接让对面知道「这块完了」。

        不发 Content-Length：BaseHTTPRequestHandler 默认 HTTP/1.0，
        没有长度就是「读到连接关闭为止」，正是无限流要的语义（别去改 protocol_version，
        /live 和 /map.png 都靠现有行为）。
        """
        global _mic_listeners
        with _mic_lock:
            next_seq = _mic_seq  # 从最新一块开始，不补播积压
            _mic_listeners += 1
            pump_ctl("go")  # 有人听了，拉泵开流
        stop = threading.Event()
        try:
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            for _seq, blk in mic_blocks(next_seq, stop):
                self.wfile.write(len(blk).to_bytes(4, "big"))
                self.wfile.write(blk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass  # 对面关了，正常收尾
        finally:
            with _mic_lock:
                _mic_listeners -= 1
                if _mic_listeners <= 0:
                    pump_ctl("stop")  # 没人听了，泵停流省 CPU
                    with lock:
                        a = got.get("audio")
                        if a:
                            a["rms"] = 0  # 电平条跟着归零，别冻在上一次
            stop.set()
            self.close_connection = True

    def do_POST(self):
        """上行对讲：正文是分帧的 24 kHz 单声道 S16LE PCM，边收边推进播放环。

        帧 = 4 字节大端长度 + 负载；长度为 0 的帧表示结束。
        用显式分帧而不是「关连接定界」，是因为中间隔着 tailscale nc，
        它不透传 TCP 半关 —— 靠 EOF 判断结束会永远等下去。
        """
        if not self.path.startswith("/play.pcm"):
            self.send_error(404)
            return
        token = "talk_%d" % int(time.time() * 1000)
        total = 0
        chunks = 0
        try:
            while True:
                hdr = self.rfile.read(4)
                if len(hdr) < 4:
                    break
                n = int.from_bytes(hdr, "big")
                if n == 0:
                    break
                blk = self.rfile.read(n)
                if len(blk) < n:
                    break
                if play_pcm(blk, token):
                    total += len(blk)
                    chunks += 1
        except OSError:
            pass
        body = json.dumps(
            {"ok": chunks > 0, "bytes": total, "chunks": chunks, "token": token}
        ).encode()
        try:
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except OSError:
            pass


if __name__ == "__main__":
    class Server(ThreadingHTTPServer):
        allow_reuse_address = True

    httpd = Server(("0.0.0.0", PORT), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    threading.Thread(target=pump_reader, daemon=True, name="pump_reader").start()
    pump_ctl("stop")  # 泵出生即建订阅（DDS 发现要 ~10 秒，先跑着），但先不发
    print("HTTP_OK", PORT, flush=True)
    try:
        ros_loop()
    except Exception:
        import traceback

        traceback.print_exc()
        raise
