import array
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from aimdk_msgs.msg import (
    AgentConversationState,
    AgentShowText,
    AlertCodeArray,
    AmStatus,
    AudioCapture,
    AudioPlayback,
    BiologicalEntryState,
    Bms,
    ChargePointRecognitionState,
    CommonRequest,
    CoreonFusionState,
    DiagnosticInfoArray,
    DisplaySate,
    EventProbeInfoArray,
    FaceInfoArray,
    FollowState,
    Insta360TaskState,
    IntelligentMotionTaskState,
    InteractionState,
    JointInfoArray,
    KeyValue,
    MapBuildState,
    MapState,
    McBodyPose,
    McCommonState,
    McLocomotionVelocity,
    ModuleInfo,
    NaviState,
    NetworkState,
    PerceptionFollow,
    PlayStateChange,
    RGBState,
    RobotState,
    SemanticIntent,
    ServoStatus,
    SlamState,
    SmSystemState,
    TouchState,
    TtsStatus,
)
from aimdk_msgs.srv import (
    GetCustomMotionList,
    GetMapData,
    GetMapList,
    GetMcAction,
    GetMotionList,
    GetMute,
    GetNaviPoints,
    GetVolume,
)

QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=4,
)


def rpy_deg(x, y, z, w):
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return [round(math.degrees(a), 1) for a in (roll, pitch, yaw)]


try:  # 机器人上有 numpy（1.26）；没有就退回纯 Python，不影响正确性
    import numpy as _np
except ImportError:  # pragma: no cover
    _np = None


def rms16(data):
    """音频电平。每次重建快照都要跑两遍（采集口 + 播放口），
    4000 个样本的 Python 生成式在 12.5 Hz 下就是每秒十万次解释器迭代 —— 用 numpy 摊平。"""
    raw = bytes(data[:8000])
    if len(raw) < 2:
        return 0
    if _np is not None:
        a = _np.frombuffer(raw[: len(raw) // 2 * 2], dtype="<i2")
        if not a.size:
            return 0
        return int(float((a.astype("f8") ** 2).mean()) ** 0.5)
    a = array.array("h")
    a.frombytes(raw[: len(raw) // 2 * 2])
    if not a:
        return 0
    return int((sum(v * v for v in a) / len(a)) ** 0.5)


def xyz(msg):
    p = msg.pose.pose.position
    return {"x": round(p.x, 3), "y": round(p.y, 3), "z": round(p.z, 3)}


def take(store, key):
    def cb(msg):
        store[key] = msg
        store["_t_" + key] = time.time()

    return cb


SUBS = [
    (Bms, "/aima/hal/bms/state", "bms"),
    # 采集口**故意不在这里**。它 25 条/秒 × 30 KB，在这儿再订一份等于把同样大的
    # 消息在同一个进程里反序列化两遍，而这一份纯粹是给界面电平表用的。
    # 现在电平表由 live_daemon.feed_mic 顺手算（它本来就要拆这一帧），
    # got["audio"] 由那边写成同样的形状。加回来之前先读 live_daemon.mic_loop 的注释。
    (AudioPlayback, "/aima/hal/audio/playback", "playbk"),
    (FaceInfoArray, "/aima/agent/face_info", "face"),
    (TouchState, "/aima/hal/touch/state", "touch"),
    (Odometry, "/odom", "odom"),
    (Odometry, "/odom_torso", "torso"),
    (Odometry, "/torso_imu_orientation", "imu"),
    (Path, "/odom/trajectory", "traj"),
    (DisplaySate, "/aima/display_state", "disp"),
    (RGBState, "/aima/hal/rgb/state", "rgb"),
    (SemanticIntent, "/aima/agent/semantic_intent", "intent"),
    (AmStatus, "/aima/task_engine/task_status", "task"),
    (PlayStateChange, "/aima/hal/audio/play_state", "play"),
    (JointInfoArray, "/aima/hal/joint/info", "jinfo"),
    (JointState, "/upper_body_joint_states", "jstate"),
    (McBodyPose, "/aima/mc/body_pose", "body"),
    (McCommonState, "/aima/mc/common/state", "mc"),
    (ServoStatus, "/aima/hal/servo/state", "servo"),
    (AlertCodeArray, "/aima/hds/alert_code_list", "alert"),
    (DiagnosticInfoArray, "/aima/hds/diagnostic_list", "diag"),
    (RobotState, "/aima/app_proxy/robot/state", "rstate"),
    (McLocomotionVelocity, "/aima/mc/locomotion/velocity", "vel"),
    (McLocomotionVelocity, "/aima/core/locomotion/velocity", "cvel"),
    (InteractionState, "/aima/interaction/state", "ix"),
    (AgentConversationState, "/aima/agent/interaction_state", "conv"),
    (AgentShowText, "/aima/agent_show_text", "show"),
    (BiologicalEntryState, "/aima/agent/biologic_state", "bio"),
    (TtsStatus, "/interaction/tts_status", "tts"),
    (ModuleInfo, "/aima/hal/camera/module_info", "caminfo"),
    (SmSystemState, "/aima/sm/system_state", "sm"),
    (IntelligentMotionTaskState, "/aima/motion/intelligent_task_state", "imot"),
    (EventProbeInfoArray, "/robot/trace/events", "ev"),
    (PoseArray, "/fall_guard/danger_regions", "fall"),
    (KeyValue, "/sys_guard/active_scene", "scene"),
    (FollowState, "/aima/algo/follow/state", "follow"),
    (PerceptionFollow, "/perception/follow_candidates", "fcand"),
    (SlamState, "/slam_state", "slam"),
    (MapState, "/aima/map/state", "mapst"),
    (MapBuildState, "/aima/map/build_state", "mapb"),
    (NaviState, "/aima/algo/navi/state", "navi"),
    (Insta360TaskState, "/insta360/task_state", "insta"),
    (NetworkState, "/aima/hal/network/state", "net"),
    (CoreonFusionState, "/aima/coreon_fusion/state", "fusion"),
    (ChargePointRecognitionState, "/aima/algo/charge_point/recognition_state", "charge"),
]
def attach(n, store):
    for cls, topic, key in SUBS:
        try:
            n.create_subscription(cls, topic, take(store, key), QOS)
            print("SUB", key, flush=True)
        except Exception as exc:
            print("SUB_FAIL", key, topic, str(exc)[:120], flush=True)


def call_srv(n, cls, name, timeout=0.08):
    c = n.create_client(cls, name)
    if not c.wait_for_service(timeout_sec=timeout):
        return None
    fut = c.call_async(cls.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=0.2)
    return fut.result() if fut.done() else None


def poll_maps(n):
    c = n.create_client(GetMapList, "/aimdk_5Fmsgs/srv/GetMapList")
    if not c.wait_for_service(timeout_sec=0.2):
        return []
    req = GetMapList.Request()
    req.header = CommonRequest()
    req.header.header.stamp = n.get_clock().now().to_msg()
    fut = c.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=1.2)
    r = fut.result() if fut.done() else None
    if r is None:
        return []
    return [
        {"id": m.map_id, "name": m.map_name, "created": int(m.create_utc_time)}
        for m in r.map_meta_infos
    ]


def poll_motions(n):
    """机器人到底会哪些动作 —— 问它，别照着手册写死。

    控制台的「预设动作」里写死了 6 个（挥手/握手/碰拳/比心/拥抱/跳舞A）。
    实测 GetMotionList 是**真的能调通**的（服务列表里有名字是一回事，
    wait_for_service 返回 True 才算数 —— GetMapList 就是反面教材）：
    这台机器内置 16 个动作，从挥手到打拳、街舞都有，写死的那 6 个只是其中三分之一。
    另外 GetCustomMotionList 里已经躺着用户自己录的一条 24 秒动作。

    返回 {"builtin": [{id,name}], "custom": [{name,path,secs}]}；调不通就是空，
    前端退回写死那 6 个 —— 这不是「失败了」，是这台机器可能没加载动作库。
    """
    out = {"builtin": [], "custom": []}
    c = n.create_client(GetMotionList, "/aimdk_5Fmsgs/srv/GetMotionList")
    if c.wait_for_service(timeout_sec=0.3):
        req = GetMotionList.Request()
        req.header = _cr(n)
        req.motion_type = 0        # 0 = 全部
        req.motion_list_type = 0   # 0 = 全部分类
        fut = c.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=1.5)
        r = fut.result() if fut.done() else None
        if r is not None:
            out["builtin"] = [
                {"id": int(m.motion_id), "name": m.motion_name} for m in r.motion_list
            ]
    c2 = n.create_client(GetCustomMotionList, "/aimdk_5Fmsgs/srv/GetCustomMotionList")
    if c2.wait_for_service(timeout_sec=0.3):
        req = GetCustomMotionList.Request()
        req.header = _cr(n)
        fut = c2.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=1.5)
        r = fut.result() if fut.done() else None
        if r is not None:
            out["custom"] = [
                {
                    "name": m.resource_key,
                    "path": m.resource_path,
                    "secs": round(float(m.duration), 1),
                    # PlayCustomMotion 要 resource_type（1=灵创 / 2=示教），
                    # 报上来就别丢，前端放按钮时原样传回去。
                    "type": int(m.resource_type),
                }
                for m in r.motion_list
            ]
    return out


def _cr(n):
    h = CommonRequest()
    h.header.stamp = n.get_clock().now().to_msg()
    return h


def poll_room(n, map_id):
    info, points, png = None, [], None
    if not map_id:
        return info, points, png
    dc = n.create_client(GetMapData, "/aimdk_5Fmsgs/srv/GetMapData")
    if dc.wait_for_service(timeout_sec=0.25):
        req = GetMapData.Request()
        req.header = _cr(n)
        req.map_id = map_id
        fut = dc.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=2.4)
        r = fut.result() if fut.done() else None
        if r is not None:
            mi = r.map_info
            og = mi.origin
            info = {
                "id": getattr(r.map_meta_info, "map_id", "") or map_id,
                "name": getattr(r.map_meta_info, "map_name", "") or "",
                "resolution": float(mi.resolution),
                "w": int(mi.width),
                "h": int(mi.height),
                "origin": {"x": float(og.x), "y": float(og.y), "yaw": float(og.yaw)},
            }
            if r.png_data:
                png = bytes(r.png_data)
    pc = n.create_client(GetNaviPoints, "/aimdk_5Fmsgs/srv/GetNaviPoints")
    if pc.wait_for_service(timeout_sec=0.2):
        req = GetNaviPoints.Request()
        req.header = _cr(n)
        req.map_id = map_id
        fut = pc.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=1.6)
        r = fut.result() if fut.done() else None
        if r is not None:
            for p in r.points[:40]:
                pose = p.pixel_pose
                points.append({
                    "id": p.point_id or "",
                    "name": p.point_name or "",
                    "type": int(p.point_type),
                    "x": float(pose.x),
                    "y": float(pose.y),
                    "yaw": float(pose.yaw),
                })
    return info, points, png


def poll_services(n):
    vol = call_srv(n, GetVolume, "/aimdk_5Fmsgs/srv/GetVolume")
    mute = None
    mc = n.create_client(GetMute, "/aimdk_5Fmsgs/srv/GetMute")
    if mc.wait_for_service(timeout_sec=0.08):
        req = GetMute.Request()
        req.request = CommonRequest()
        req.request.header.stamp = n.get_clock().now().to_msg()
        fut = mc.call_async(req)
        rclpy.spin_until_future_complete(n, fut, timeout_sec=0.2)
        mute = fut.result() if fut.done() else None
    act = call_srv(n, GetMcAction, "/aimdk_5Fmsgs/srv/GetMcAction")
    return vol, mute, act


def build_payload(got, vol=None, mute=None, act=None):
    now = time.time()
    payload = {
        "heard": sorted(k for k in got if not k.startswith("_")),
        "volume": getattr(vol, "audio_volume", None) if vol else None,
        "mute": getattr(mute, "is_mute", None) if mute else None,
        "action": None,
        "battery": None,
        "audio": None,
        "playback": None,
        "faces": [],
        "touch": None,
        "odom": None,
        "torso": None,
        "imu": None,
        "traj": [],
        "display": None,
        "led": None,
        "intent": None,
        "task": None,
        "play": None,
        "joints": [],
        "joint_names": [],
        "body": None,
        "mc": None,
        "servo": None,
        "alerts": [],
        "diags": [],
        "robot": None,
        "vel": None,
        "ix": None,
        "conv": None,
        "show": None,
        "bio": None,
        "tts": None,
        "camera": None,
        "sm": None,
        "imot": None,
        "events": 0,
        "fall": 0,
        "scene": None,
        "follow": None,
        "candidates": [],
        "slam": None,
        "map": None,
        "navi": None,
        "insta": None,
        "maps": got.get("_maps") or [],
        "points": got.get("_navi_points") or [],
        "map_info": got.get("_map_info"),
        "net": None,
        "fusion": None,
        "charge": None,
        "dangers": [],
        "blocked": [],
    }

    if act and getattr(act, "info", None):
        payload["action"] = {
            "id": act.info.current_action.value,
            "name": act.info.action_desc,
            "switch_ready": bool(act.info.switch_ready),
        }

    if "bms" in got:
        m = got["bms"]
        payload["battery"] = {
            "percent": int(m.remaining_capacity_percentage),
            "voltage_v": round(m.voltage / 1000.0, 2),
            "current_a": round(m.current / 1000.0, 2),
            "power_w": round(m.power / 1000.0, 1),
            "temp_c": round(float(m.temperature), 1),
            "cycles": int(m.cycle_count),
            "status": int(m.status),
        }

    if "audio" in got:
        # 采集口那条订阅不在 SUBS 里了，这个键由 live_daemon.feed_mic 直接写成
        # 已经算好的 dict（它 25 次/秒都在拆同一帧，顺手算电平不要钱）。
        payload["audio"] = got["audio"]

    if "playbk" in got:
        m = got["playbk"]
        payload["playback"] = {
            "pkg": m.pkg_name,
            "sample_rate": int(m.info.sample_rate),
            "channels": int(m.info.channels),
            "rms": rms16(m.data.data),
        }

    if "face" in got:
        payload["faces"] = [
            {
                "name": f.name or "",
                "nickname": f.nickname or "",
                "id": f.global_id or f.face_uid or "",
                "x": int(f.x),
                "y": int(f.y),
                "w": int(f.width),
                "h": int(f.height),
                "behavior": int(f.behavior),
                "gesture": int(f.gesture),
            }
            for f in got["face"].list
        ]

    if "touch" in got:
        payload["touch"] = {"sensor": int(got["touch"].sensor_id), "event": int(got["touch"].event_type)}

    if "odom" in got:
        o = got["odom"].pose.pose.orientation
        payload["odom"] = {
            **xyz(got["odom"]),
            "yaw": rpy_deg(o.x, o.y, o.z, o.w)[2],
        }
    if "torso" in got:
        payload["torso"] = xyz(got["torso"])
    if "imu" in got:
        o = got["imu"].pose.pose.orientation
        payload["imu"] = dict(zip(("roll", "pitch", "yaw"), rpy_deg(o.x, o.y, o.z, o.w)))

    if "traj" in got:
        payload["traj"] = [
            {"x": round(p.pose.position.x, 3), "y": round(p.pose.position.y, 3)}
            for p in got["traj"].poses[-80:]
        ]

    if "disp" in got:
        payload["display"] = {"emoticon_id": int(got["disp"].emoticon_id)}

    if "rgb" in got:
        m = got["rgb"]
        payload["led"] = {"mode": int(m.display_mode), "r": int(m.r), "g": int(m.g), "b": int(m.b), "period": int(m.period)}

    if "intent" in got:
        payload["intent"] = {"type": got["intent"].type, "value": got["intent"].value}

    if "task" in got:
        payload["task"] = {
            "name": got["task"].function_name,
            "status": int(got["task"].function_status),
            "json": (got["task"].json_data or "")[:240],
        }

    if "play" in got:
        payload["play"] = {"pkg": got["play"].pkg_name, "state": int(got["play"].state.value)}

    if "jstate" in got:
        m = got["jstate"]
        payload["joints"] = [
            {"name": name, "pos": round(float(pos), 3)}
            for name, pos in zip(list(m.name), list(m.position))
        ]

    if "jinfo" in got:
        payload["joint_names"] = [j.name for j in got["jinfo"].joints]

    # 动作库：守护进程每两分钟问一次机器人（这是静态数据，问那么勤没意义）。
    # 拿不到就是 None，前端退回写死的那 6 个。
    if got.get("_motions"):
        payload["motions"] = got["_motions"]

    if "body" in got:
        m = got["body"]
        payload["body"] = {
            "stand_height": round(m.stand_height, 3),
            "bend_forward": round(m.bend_forward, 3),
            "bend_sideways": round(m.bend_sideways, 3),
            "twist_waist": round(m.twist_waist, 3),
            "raise_head": round(m.raise_head, 3),
            "turn_head": round(m.turn_head, 3),
        }

    if "mc" in got:
        m = got["mc"]
        payload["mc"] = {
            "fallen": bool(m.is_fallen),
            "moving": bool(m.is_moving),
            "action": m.action_info.action_desc,
            "action_id": m.action_info.current_action.value,
            "switch_ready": bool(m.action_info.switch_ready),
            "fsm": int(m.fsm_state.current_state),
            "form": int(m.form_state.current_form),
            "posture": {
                "roll": round(m.posture.roll, 2),
                "pitch": round(m.posture.pitch, 2),
                "yaw": round(m.posture.yaw, 2),
            },
            "motion": m.motion_status.motion,
            "steps": int(m.motion_status.current_steps),
        }

    if "servo" in got:
        payload["servo"] = {"position": round(float(got["servo"].position), 3)}

    if "alert" in got:
        payload["alerts"] = [
            {"code": a.code, "level": int(a.level), "text": a.text, "light": a.light_msg}
            for a in got["alert"].list[:20]
        ]

    if "diag" in got:
        payload["diags"] = [
            {"module": int(d.module_id), "code": int(d.code), "info": d.info}
            for d in got["diag"].diagnostics[:20]
        ]

    if "rstate" in got:
        m = got["rstate"]
        payload["robot"] = {
            "mic": int(m.microphone),
            "camera": int(m.camera),
            "avoid": int(m.obstacle_avoidance),
            "mode": int(m.mode),
            "fwd": round(m.forward_speed, 3),
            "back": round(m.backward_speed, 3),
            "rot": round(m.rotation_speed, 3),
            "lat": round(m.lateral_speed, 3),
        }

    vel = got.get("vel") or got.get("cvel")
    if vel:
        payload["vel"] = {
            "fwd": round(vel.forward_velocity, 3),
            "lat": round(vel.lateral_velocity, 3),
            "yaw": round(vel.angular_velocity, 3),
            "pitch": round(vel.pitch_velocity, 3),
            "source": getattr(vel, "source", "") or "",
        }

    if "ix" in got:
        payload["ix"] = {
            "mode": int(got["ix"].mode),
            "action": got["ix"].action_info.action_desc,
            "form": int(got["ix"].form_state.current_form),
        }

    if "conv" in got:
        payload["conv"] = {
            "main": int(got["conv"].main_state.value),
            "sub": int(got["conv"].sub_state.value),
            "event": got["conv"].event_id,
        }

    if "show" in got:
        payload["show"] = {"type": int(got["show"].msg_type), "text": got["show"].text[:200]}

    if "bio" in got:
        payload["bio"] = {
            "stage": int(got["bio"].stage),
            "state": int(got["bio"].state),
            "reason": got["bio"].reason,
        }

    if "tts" in got:
        payload["tts"] = {
            "text": got["tts"].text[:120],
            "status": int(got["tts"].tts_status.value),
            "domain": got["tts"].domain,
            "err": got["tts"].error_message,
        }

    if "caminfo" in got:
        payload["camera"] = {
            "id": got["caminfo"].module_id,
            "kv": {kv.key: kv.value for kv in got["caminfo"].info[:20]},
        }

    if "sm" in got:
        payload["sm"] = {"state": got["sm"].cur_state, "status": int(got["sm"].cur_status.value)}

    if "imot" in got:
        m = got["imot"]
        payload["imot"] = {
            "id": int(m.task_id),
            "type": int(m.action_type),
            "running": bool(m.task_running),
            "done": bool(m.task_finished),
            "fail": bool(m.task_failed),
        }

    if "ev" in got:
        payload["events"] = len(got["ev"].list) if hasattr(got["ev"], "list") else 0
    if "fall" in got:
        payload["fall"] = len(got["fall"].poses)
        payload["dangers"] = [
            {
                "x": round(p.position.x, 3),
                "y": round(p.position.y, 3),
                "yaw": rpy_deg(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)[2],
            }
            for p in got["fall"].poses[:16]
        ]
    try:
        if "net" in got:
            w = got["net"].connect_wifi_info
            payload["net"] = {
                "ssid": (w.ssid or "")[:64],
                "ip": w.ip or "",
                "signal": int(w.signal_strength),
                "connected": bool(w.is_connected),
                "state": int(w.connect_state),
            }
    except Exception:
        pass
    try:
        if "fusion" in got:
            code = int(got["fusion"].obstacle_info.obstacle)
            labels = {
                0: "没挡",
                1: "前有障碍", 2: "左有障碍", 3: "右有障碍", 4: "后有障碍",
                5: "前要跌", 6: "左要跌", 7: "右要跌", 8: "后要跌",
                9: "前倾斜", 10: "左倾斜", 11: "右倾斜", 12: "后倾斜",
            }
            payload["fusion"] = {
                "avoid": bool(got["fusion"].is_avoidance),
                "source": got["fusion"].source or "",
                "code": code,
                "label": labels.get(code, str(code)),
            }
    except Exception:
        pass
    try:
        if "charge" in got:
            st = int(got["charge"].state)
            payload["charge"] = {
                "state": st,
                "label": {0: "未知", 1: "在找桩", 2: "找到了", 3: "已记下", 4: "失败", 5: "取消"}.get(st, str(st)),
                "msg": got["charge"].message or "",
                "has": bool(got["charge"].has_result),
            }
    except Exception:
        pass
    if "scene" in got:
            payload["scene"] = {"key": got["scene"].key, "value": got["scene"].value}

    FOLLOW = {
        0: "空闲", 1: "没看见人", 2: "跟着", 3: "跟丢了",
        4: "跟错人", 5: "太远", 6: "太近", 7: "被挡住",
    }
    NAVI = {
        0: "空闲", 1: "在初始化", 2: "在走过去", 3: "暂停",
        4: "到了", 5: "失败", 6: "取消了", 7: "电不够", 8: "初始化失败",
    }
    BUILD_STAGE = {0: "未知", 1: "在建图", 2: "在存图", 3: "在取消"}
    BUILD_STATE = {0: "未知", 1: "进行中", 2: "成功", 3: "失败"}

    if "follow" in got:
        st = int(got["follow"].state)
        payload["follow"] = {"state": st, "label": FOLLOW.get(st, str(st))}
    if "fcand" in got:
        payload["candidates"] = [
            {"id": int(c.id), "x": int(c.x), "y": int(c.y), "w": int(c.w), "h": int(c.h)}
            for c in got["fcand"].perception_list[:12]
        ]
    if "slam" in got:
        payload["slam"] = {"msg": got["slam"].state_msg or ""}
    if "mapst" in got or "mapb" in got:
        b = got.get("mapb")
        payload["map"] = {
            "current": getattr(got.get("mapst"), "current_map", "") or "",
            "stage": int(b.stage) if b else None,
            "stage_label": BUILD_STAGE.get(int(b.stage), str(b.stage)) if b else "",
            "state": int(b.state) if b else None,
            "state_label": BUILD_STATE.get(int(b.state), str(b.state)) if b else "",
            "type": int(b.mapping_type) if b else None,
            "task_id": (b.task_id or "") if b else "",
            "w": int(b.map_info.width) if b else 0,
            "h": int(b.map_info.height) if b else 0,
            "has_png": bool(b and b.png_data),
            "png_bytes": len(b.png_data) if b and b.png_data else 0,
        }
    if "navi" in got:
        st = int(got["navi"].state)
        payload["navi"] = {"state": st, "label": NAVI.get(st, str(st)), "desc": got["navi"].description or ""}
    if "insta" in got:
        payload["insta"] = {
            "mode": int(got["insta"].task_mode),
            "state": int(got["insta"].task_state),
        }

    payload["ages"] = {
        k[3:]: round(now - got[k], 1)
        for k in got
        if k.startswith("_t_")
    }
    payload["fresh_s"] = round(now - max((got[k] for k in got if k.startswith("_t_")), default=now), 2)
    return payload


def run_once():
    rclpy.init()
    n = Node("q1_live")
    got = {}
    attach(n, got)
    end = time.time() + 3.8
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    vol, mute, act = poll_services(n)
    payload = build_payload(got, vol, mute, act)
    print("LIVE", json.dumps(payload, ensure_ascii=False))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_once()
