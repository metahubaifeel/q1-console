"""独立麦克风泵：自己的进程、自己的 DDS participant，把采集口抽成 24 kHz 单声道
ch0 块，从 stdout 流给 live_daemon。daemon 只做字节转发。

为什么独立进程（2026-09-14 实测定案）：
机器人负载 30+ 之后，daemon 进程内的 mic 线程被饿到 3.2 块/秒（成簇到达，
p90 间隔 892 ms），而同一时刻独立进程同机同 QoS 实测 17.5 条/秒、中位间隔 24 ms。
原因不在回调体 —— daemon 的 Fast-DDS 收包线程要给全部 12 个订阅反序列化
（采集口 30 KB × 17.5/s 是最大头），mic 线程还要跟主循环抢 GIL。
分进程后各自轻量：本进程只订采集口一个 topic，收包线程几乎没活干。

协议（stdout 给 daemon，stdin 收 daemon）：
  stdout 第 1 行 = JSON 头 {"sr":48000,"ch":8,"fmt":"S16LE pcm","msg_bytes":30720}
  之后       = 无限个 [4 字节大端长度 + PCM 帧]，帧是 24 kHz 单声道 S16LE
  stdin  "GO"  = 建订阅开始流（没人在听就停着，别白烧机器人 CPU）
  stdin  "STOP" = 毁订阅停流
  进程在 stdin EOF 或 stdout 写失败时退出，daemon 负责重拉。
"""
import array
import json
import select
import sys

sys.path.insert(0, "/tmp")
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from aimdk_msgs.msg import AudioCapture

# 与 live_daemon 原 MIC_QOS 一致：BEST_EFFORT + depth=30（实测 17 条/秒，
# 天花板是机器人自己的发布率；depth=4 只有 13.2，depth=200 也不再多）。
MIC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=30,
)


def emit(blk: bytes) -> None:
    # 必须走 .buffer：stdout 是文本包装，.write(bytes) 会抛 TypeError
    #（第一版就死在这，异常又被 feed 的 except 吞掉，帧全丢还看不出来）
    sys.stdout.buffer.write(len(blk).to_bytes(4, "big"))
    sys.stdout.buffer.write(blk)
    sys.stdout.buffer.flush()


def main() -> None:
    rclpy.init()
    node = Node("q1_mic_pump")
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    feeding = False

    def feed(msg) -> None:
        if not feeding:
            return
        try:
            raw = bytes(msg.data.data)
            a = array.array("h")
            a.frombytes(raw[: len(raw) // 2 * 2])
            # 交错布局是 [帧0的ch0..ch7][帧1的ch0..ch7]...，取 ch0 就是每隔 8 个取一个
            ch0 = a[0 :: msg.info.channels or 8]
            # ÷2 抽取：相邻两个求平均当低通，比直接丢掉一个少一点混叠
            out = array.array(
                "h", [(ch0[i] + ch0[i + 1]) >> 1 for i in range(0, len(ch0) - 1, 2)]
            )
            if out:
                emit(out.tobytes())
        except Exception as exc:
            # 单条坏消息不该弄死泵，但也不能一声不吭
            print("FEED_ERR", type(exc).__name__, file=sys.stderr, flush=True)

    # 订阅出生就建、从不销毁：GO/STOP 只闸「发不发」。DDS 发现在这台过载的
    # 机器上要 ~10 秒（实测首块延迟 10.4s），每次收听才建订阅的话用户点下
    # 「收听」要先听 10 秒静音，看着像坏了。常驻订阅的代价是 ~10% CPU 收包，
    # 但这笔账原来就在 daemon 头上（它常驻订了采集口），分进程后只是挪了地方。
    node.create_subscription(AudioCapture, "/aima/hal/audio/capture", feed, MIC_QOS)

    sys.stdout.write(
        json.dumps({"sr": 48000, "ch": 8, "fmt": "S16LE pcm", "msg_bytes": 30720}) + "\n"
    )
    sys.stdout.flush()
    print("PUMP_OK", file=sys.stderr, flush=True)
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        if r:
            line = sys.stdin.readline()
            if not line:
                return
            cmd = line.strip().upper()
            if cmd == "GO" and not feeding:
                feeding = True
                print("PUMP_GO", file=sys.stderr, flush=True)
            elif cmd == "STOP" and feeding:
                feeding = False
                print("PUMP_STOP", file=sys.stderr, flush=True)
        ex.spin_once(timeout_sec=0.05)


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        pass
    except Exception as exc:
        print("PUMP_DIED", str(exc)[:200], file=sys.stderr, flush=True)
        sys.exit(1)
