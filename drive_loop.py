"""Hold-to-drive publisher. Stdin: 'fwd lat yaw' or ARM / DISARM. Deadman 0.4s."""
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import CommonRequest, McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource

SRC = "q1_console"
# 没人说话多久之后自己退出。为什么必须有，见主循环前那段。
IDLE_EXIT = 60.0
cmd = [0.0, 0.0, 0.0]
stamp = time.time()
want = None
eof = False
lock = threading.Lock()


def reader():
    global stamp, want, eof
    for line in sys.stdin:
        with lock:
            stamp = time.time()  # 还活着，别被空闲看门狗收走
        parts = line.strip().split()
        if not parts:
            continue
        if parts[0] == "QUIT":
            break
        if parts[0] == "ARM":
            with lock:
                want = "arm"
            continue
        if parts[0] == "DISARM":
            with lock:
                want = "disarm"
                cmd[:] = [0.0, 0.0, 0.0]
                stamp = time.time()
            continue
        try:
            fwd, lat, yaw = float(parts[0]), float(parts[1]), float(parts[2])
        except (ValueError, IndexError):
            continue
        with lock:
            cmd[:] = [fwd, lat, yaw]
            stamp = time.time()
    with lock:
        eof = True  # stdin 断了 = 本机 server 没了，没必要再赖着


def call_source(n, cli, action, name):
    req = SetMcInputSource.Request()
    req.action.value = action
    req.input_source.name = name
    req.input_source.priority = 80
    req.input_source.timeout = 1000
    req.request = CommonRequest()
    req.request.header.stamp = n.get_clock().now().to_msg()
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5)
    return fut.result() if fut.done() else None


def arm_now(n, cli):
    if not cli.wait_for_service(timeout_sec=4):
        print("INPUT_SRC no-svc", flush=True)
        return
    call_source(n, cli, 1003, SRC)
    r = call_source(n, cli, 1001, SRC)
    code = r.response.header.code if r else None
    print("INPUT_SRC", code, flush=True)


def release(n, cli):
    """交还速度源。1003 = 放手，和 arm_now 里那句是同一个动作。"""
    try:
        if cli.service_is_ready():
            call_source(n, cli, 1003, SRC)
            print("INPUT_SRC off", flush=True)
    except Exception:
        pass


rclpy.init()
n = Node("q1_drive")
pub = n.create_publisher(McLocomotionVelocity, "/aima/mc/locomotion/velocity", 10)
src_cli = n.create_client(SetMcInputSource, "/aimdk_5Fmsgs/srv/SetMcInputSource")
threading.Thread(target=reader, daemon=True).start()
print("DRIVE_READY", flush=True)
arm_now(n, src_cli)

# ── 为什么这个循环必须会自己结束 ──────────────────────────────────────
# 这个循环每 0.02 秒发一条 source="q1_console" 的速度指令，**armed 与否都发**
# （没按键就是零速度，但 source 字段一直在刷新那个 priority=80 的速度源占用）。
#
# 于是「遥控关了」在这台机器上等于没关：本机 /api/drive/disarm 确实把 DISARM
# 送进来了、也确实调了 1003 交还，但**进程不退**，50 Hz 的速度流立刻又把速度源
# 抢了回去。后果三条，实测都撞上了：
#   1. 只要开过一次遥控，控制台就一直攥着机器人的速度源，机器人自带 App /
#      Agent 再也拿不回去，直到 systemctl restart 整个服务；
#   2. 白烧 11% CPU —— 这台 6 核机器常年负载 20~36，mc_main（平衡控制器）常年
#      99%，从它嘴里抢的每一分都是它站不稳的理由；
#   3. 2026-09-14 重启服务时实测：_kill_remote_drive() 把进程杀了，但浏览器那边
#      「遥控」还开着、8.3 Hz 的保活立刻把它重新拉起来，模块级的 arm_now() 又去
#      抢速度源 —— 机器人当场从趴着站了起来。控制台不该有这种「松手了还攥着」
#      的状态。
#
# 所以：DISARM 要真的走人，stdin 断了要走人，没人理它一分钟也要走人。
# 走人不心疼 —— 本机 ensure_drive() 是懒加载单例，下次按键重新拉起来就是了
# （代价是一次 SSH 上传 + 1~2 秒，正是 arm 那条路本来就要付的）。
try:
    while rclpy.ok():
        with lock:
            fwd, lat, yaw = cmd
            age = time.time() - stamp
            gone = eof
            job = want
            want = None
        if job == "arm":
            arm_now(n, src_cli)
        elif job == "disarm":
            release(n, src_cli)
            break
        if age > 0.4:
            fwd = lat = yaw = 0.0
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = n.get_clock().now().to_msg()
        msg.source = SRC
        msg.forward_velocity = float(fwd)
        msg.lateral_velocity = float(lat)
        msg.angular_velocity = float(yaw)
        msg.pitch_velocity = 0.0
        msg.level = 0.0
        pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(0.02)
        if gone or time.time() - stamp > IDLE_EXIT:
            print("DRIVE_EXIT idle", flush=True)
            break
finally:
    release(n, src_cli)
    n.destroy_node()
    rclpy.shutdown()
