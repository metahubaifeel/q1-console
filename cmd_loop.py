"""Persistent ROS command node. Stdin: MUTE 1 | HUSH | VOL 20 | SAY text | EMO 3003 | LIGHT 17 10 40 200 | MOTION 3004 | CMOTION 2 action_1 | ACTION 1007"""
import select
import sys
import time

import rclpy
from rclpy.node import Node
from aimdk_msgs.msg import (
    AgentPropertiesValue,
    AgentPropertyIdType,
    CommonRequest,
    McControlArea,
    NaviPointInfo,
    TokenCommon,
    TtsPriorityLevel,
)
from aimdk_msgs.srv import (
    LedStripCommand,
    MapCommand,
    MapLocalizationTask,
    NaviTaskCtrl,
    SetNaviPoint,
    PlayCustomMotion,
    PlayEmotion,
    PlayTts,
    SetAgentPropertiesRequest,
    SetFollowPerceptionTask,
    SetFollowTaskBiped,
    SetMcAction,
    SetMcPresetMotion,
    SetMute,
    SetVolume,
    StopAudioPlay,
    StopTtsTraceId,
    GetAudioStatus,
    GetMute,
)

SRC = "q1_console"


def cr(n):
    h = CommonRequest()
    h.header.stamp = n.get_clock().now().to_msg()
    return h


def call(n, cli, req, timeout=3.0):
    if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=1.5):
        return None
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=timeout)
    return fut.result() if fut.done() else None


def main():
    rclpy.init()
    n = Node("q1_cmd")
    mute_cli = n.create_client(SetMute, "/aimdk_5Fmsgs/srv/SetMute")
    vol_cli = n.create_client(SetVolume, "/aimdk_5Fmsgs/srv/SetVolume")
    tts_cli = n.create_client(PlayTts, "/aimdk_5Fmsgs/srv/PlayTts")
    stop_tts = n.create_client(StopTtsTraceId, "/aimdk_5Fmsgs/srv/StopTtsTraceId")
    stop_aud = n.create_client(StopAudioPlay, "/aimdk_5Fmsgs/srv/StopAudioPlay")
    emo_cli = n.create_client(PlayEmotion, "/aimdk_5Fmsgs/srv/PlayEmotion")
    led_cli = n.create_client(LedStripCommand, "/aimdk_5Fmsgs/srv/LedStripCommand")
    mot_cli = n.create_client(SetMcPresetMotion, "/aimdk_5Fmsgs/srv/SetMcPresetMotion")
    cmot_cli = n.create_client(PlayCustomMotion, "/aimdk_5Fmsgs/srv/PlayCustomMotion")
    act_cli = n.create_client(SetMcAction, "/aimdk_5Fmsgs/srv/SetMcAction")
    fol_cli = n.create_client(SetFollowTaskBiped, "/aimdk_5Fmsgs/srv/SetFollowTaskBiped")
    fpe_cli = n.create_client(SetFollowPerceptionTask, "/aimdk_5Fmsgs/srv/SetFollowPerceptionTask")
    map_cli = n.create_client(MapCommand, "/aimdk_5Fmsgs/srv/MapCommand")
    navi_cli = n.create_client(NaviTaskCtrl, "/aimdk_5Fmsgs/srv/NaviTaskCtrl")
    loc_cli = n.create_client(MapLocalizationTask, "/aimdk_5Fmsgs/srv/MapLocalizationTask")
    point_cli = n.create_client(SetNaviPoint, "/aimdk_5Fmsgs/srv/SetNaviPoint")
    agent_cli = n.create_client(SetAgentPropertiesRequest, "/aimdk_5Fmsgs/srv/SetAgentPropertiesRequest")
    get_mute = n.create_client(GetMute, "/aimdk_5Fmsgs/srv/GetMute")
    aud_st = n.create_client(GetAudioStatus, "/aimdk_5Fmsgs/srv/GetAudioStatus")
    print("CMD_READY", flush=True)
    want_mute = False
    last_agent = 0.0
    last_hush = 0.0

    def hush():
        if stop_aud.wait_for_service(timeout_sec=0.8):
            q = StopAudioPlay.Request()
            q.request = cr(n)
            call(n, stop_aud, q, 2.0)
        if stop_tts.wait_for_service(timeout_sec=0.6):
            q = StopTtsTraceId.Request()
            q.header = cr(n)
            q.trace_id = ""
            call(n, stop_tts, q, 2.0)

    def set_props(pairs):
        req = SetAgentPropertiesRequest.Request()
        req.header = cr(n)
        items = []
        for key, val in pairs:
            item = AgentPropertiesValue()
            item.key.value = key
            item.value = val
            items.append(item)
        req.contents.properties = items
        return call(n, agent_cli, req, 3.0)

    def agent_voice(on: bool):
        if on:
            return set_props([
                (AgentPropertyIdType.AGENT_PROPERTY_RUN_MODE, "normal"),
            ])
        return set_props([
            (AgentPropertyIdType.AGENT_PROPERTY_RUN_MODE, "no_voice"),
            (AgentPropertyIdType.AGENT_PROPERTY_GREET, "false"),
            (AgentPropertyIdType.AGENT_PROPERTY_WELCOME_STATUS, '{"on":false}'),
        ])

    def hardware_mute(on: bool):
        req = SetMute.Request()
        req.request = cr(n)
        req.is_mute = on
        return call(n, mute_cli, req, 3.0)

    def talking():
        req = GetAudioStatus.Request()
        if hasattr(req, "header"):
            req.header = cr(n)
        r = call(n, aud_st, req, 1.2)
        if r is None:
            return False
        st = getattr(r, "tts_status", None)
        if st is None:
            return False
        if getattr(st, "text", ""):
            return True
        val = getattr(getattr(st, "tts_status", None), "value", 0)
        return int(val or 0) != 0

    def enforce_mute():
        nonlocal last_agent, last_hush
        now = time.time()
        req = GetMute.Request()
        req.request = cr(n)
        r = call(n, get_mute, req, 1.2) if get_mute.service_is_ready() else None
        if r is None or not bool(getattr(r, "is_mute", False)):
            hardware_mute(True)
        if talking() or now - last_hush > 2.5:
            hush()
            last_hush = now
        if now - last_agent > 8:
            agent_voice(False)
            last_agent = now

    while True:
        ready, _, _ = select.select([sys.stdin], [], [], 0.7)
        if not ready:
            if want_mute:
                try:
                    enforce_mute()
                except Exception as exc:
                    print("CMD_ERR HOLD", str(exc)[:160], flush=True)
            continue
        line = sys.stdin.readline()
        if not line:
            break
        parts = (line or "").strip().split(None, 1)
        if not parts:
            continue
        op = parts[0].upper()
        arg = parts[1] if len(parts) > 1 else ""
        try:
            if op == "QUIT":
                print("CMD_OK QUIT", flush=True)
                break
            if op == "HUSH":
                hush()
                print("CMD_OK HUSH", flush=True)
                continue
            if op == "MUTE":
                want = arg.strip() in ("1", "true", "True", "on")
                want_mute = want
                r = hardware_mute(want)
                if want:
                    hush()
                    agent_voice(False)
                    last_agent = time.time()
                    last_hush = time.time()
                else:
                    agent_voice(True)
                got = None if r is None else bool(r.is_mute)
                print("CMD_OK MUTE", got, flush=True)
                continue
            if op == "VOL":
                req = SetVolume.Request()
                req.request = cr(n)
                req.audio_volume = int(arg)
                r = call(n, vol_cli, req, 3.0)
                print("CMD_OK VOL", None if r is None else r.audio_volume, flush=True)
                continue
            if op == "SAY":
                req = PlayTts.Request()
                req.header = cr(n)
                req.tts_req.text = arg
                req.tts_req.priority_level.value = TtsPriorityLevel.INTERACTION_L6
                req.tts_req.domain = "sdk"
                req.tts_req.is_interrupted = True
                req.tts_req.trace_id = "gui"
                r = call(n, tts_cli, req, 6.0)
                ok = bool(r and r.tts_resp.is_success)
                print("CMD_OK SAY", ok, flush=True)
                continue
            if op == "EMO":
                req = PlayEmotion.Request()
                req.header = cr(n)
                req.type = "emotion"
                req.emotion_ids = [int(arg)]
                req.file_paths = []
                req.priority = 1
                req.loop_count = 1
                r = call(n, emo_cli, req, 3.0)
                print("CMD_OK EMO", None if r is None else r.header.status.value, flush=True)
                continue
            if op == "LIGHT":
                nums = [int(x) for x in arg.split()]
                mode, r, g, b = (nums + [0, 0, 0, 0])[:4]
                period = nums[4] if len(nums) > 4 else 0
                req = LedStripCommand.Request()
                req.request = cr(n)
                req.led_strip_mode = mode
                req.r, req.g, req.b, req.period = r, g, b, period
                res = call(n, led_cli, req, 3.0)
                print("CMD_OK LIGHT", None if res is None else res.header.status.value, flush=True)
                continue
            if op == "MOTION":
                from aimdk_msgs.srv import SetMcPresetMotion as _S

                req = _S.Request()
                req.header.stamp = n.get_clock().now().to_msg()
                req.source = "sdk_rpc"
                req.area.value = McControlArea.NONE
                req.motion.value = int(arg)
                req.interrupt = True
                req.ani_path = ""
                r = call(n, mot_cli, req, 6.0)
                code = r.response.header.code if r else None
                print("CMD_OK MOTION", code, flush=True)
                continue
            if op == "CMOTION":
                # 用户自录动作：PlayCustomMotion。resource_type 用 GetCustomMotionList
                # 里报上来的原值（1=灵创 / 2=示教），resource_name 是 resource_key。
                # 自录动作没有内置编号，MOTION 那条路对它无效 —— 这是两个不同的服务。
                rtype, _, name = arg.partition(" ")
                req = PlayCustomMotion.Request()
                # 这个服务的 header 是**套娃**：字段名叫 header，类型却是 CommonRequest
                # （别的服务那儿 header 直接是带 stamp 的 RequestHeader）。直接给一个
                # 现成的 cr(n)，别手打嵌套。
                req.header = cr(n)
                try:
                    req.operation_type = 1  # START_PLAY
                    req.resource_type = int(rtype)
                except ValueError:
                    print("CMD_ERR CMOTION bad-type", arg, flush=True)
                    continue
                req.resource_name = name.strip()
                req.token.source = "q1_console"
                req.token.token = "cmot_%d" % int(time.time() * 1000)
                r = call(n, cmot_cli, req, 6.0)
                # 响应整个包在 CommonResponse 里：r.header 是 CommonResponse，
                # code 在 r.header.header.code（ResponseHeader），status 在
                # r.header.status.value（CommonState）。用探针 dir() 过真对象才敢写这行
                # —— ros2 interface show 把 status 印在顶层，照着写就是 AttributeError，
                # 这里前前后后踩了三回，全是这个套娃形状。
                code = r.header.header.code if r else None
                st = r.header.status.value if r else None
                print("CMD_OK CMOTION", code, st, flush=True)
                continue
            if op == "ACTION":
                req = SetMcAction.Request()
                req.header.stamp = n.get_clock().now().to_msg()
                req.source = "sdk_rpc"
                req.command.action.value = int(arg)
                req.command.action_desc = ""
                r = call(n, act_cli, req, 8.0)
                print("CMD_OK ACTION", r is not None, flush=True)
                continue
            if op == "FPERC":
                req = SetFollowPerceptionTask.Request()
                req.header = cr(n)
                req.trigger = arg.strip() in ("1", "true", "on")
                r = call(n, fpe_cli, req, 5.0)
                print("CMD_OK FPERC", r is not None, flush=True)
                continue
            if op == "FOLLOW":
                bits = arg.split()
                on = bits and bits[0] in ("1", "true", "on")
                tid = int(bits[1]) if len(bits) > 1 else 0
                req = SetFollowTaskBiped.Request()
                req.header = cr(n)
                req.trigger = on
                req.type = SetFollowTaskBiped.Request.FOLLOW_TYPE_FOLLOW if on else SetFollowTaskBiped.Request.FOLLOW_TYPE_UNKNOW
                req.direction = 0
                req.id = tid
                req.x = req.y = req.h = req.w = 0
                req.timestamp = 0
                r = call(n, fol_cli, req, 6.0)
                print("CMD_OK FOLLOW", r is not None, on, tid, flush=True)
                continue
            if op == "MAP":
                bits = arg.split(None, 1)
                what = (bits[0] if bits else "").lower()
                name = bits[1].strip() if len(bits) > 1 else ""
                req = MapCommand.Request()
                req.header = cr(n)
                if what in ("start", "startauto"):
                    req.command = MapCommand.Request.COMMAND_START
                    req.mapping_type = (
                        MapCommand.Request.MAPPING_TYPE_AUTO_EXPLORE
                        if what == "startauto"
                        else MapCommand.Request.MAPPING_TYPE_MANUAL
                    )
                elif what == "save":
                    req.command = MapCommand.Request.COMMAND_SAVE
                    req.map_name = name or "q1_home"
                elif what == "cancel":
                    req.command = MapCommand.Request.COMMAND_CANCEL
                else:
                    print("CMD_ERR MAP", what, flush=True)
                    continue
                req.task_id = ""
                if what != "save":
                    req.map_name = name
                r = call(n, map_cli, req, 8.0)
                print("CMD_OK MAP", what, r is not None, getattr(r, "task_id", "") if r else "", flush=True)
                continue
            if op == "LOC":
                bits = arg.split()
                what = (bits[0] if bits else "").lower()
                mid = bits[1] if len(bits) > 1 else ""
                req = MapLocalizationTask.Request()
                req.header = cr(n)
                req.command = (
                    MapLocalizationTask.Request.COMMAND_START
                    if what in ("start", "1", "on")
                    else MapLocalizationTask.Request.COMMAND_STOP
                )
                req.task_id = ""
                req.map_id = mid
                req.token.source = SRC
                req.token.token = ""
                r = call(n, loc_cli, req, 8.0)
                print("CMD_OK LOC", what, None if r is None else (r.rlt_code, r.msg or ""), flush=True)
                continue
            if op == "NAVI":
                bits = arg.split()
                what = (bits[0] if bits else "").lower()
                req = NaviTaskCtrl.Request()
                req.header = cr(n)
                req.token.source = SRC
                req.token.token = ""
                req.x = req.y = req.theta = 0.0
                req.position_name = ""
                if what in ("stop", "cancel"):
                    req.ctrl_type = NaviTaskCtrl.Request.CTRL_CANCEL if what == "cancel" else NaviTaskCtrl.Request.CTRL_STOP
                    req.navi_type = NaviTaskCtrl.Request.NAVI_UNKNOWN
                elif what == "resume":
                    req.ctrl_type = NaviTaskCtrl.Request.CTRL_RESUME
                    req.navi_type = NaviTaskCtrl.Request.NAVI_UNKNOWN
                elif what == "charge":
                    req.ctrl_type = NaviTaskCtrl.Request.CTRL_START
                    req.navi_type = NaviTaskCtrl.Request.NAVI_CHARGE
                elif what == "name" and len(bits) > 1:
                    req.ctrl_type = NaviTaskCtrl.Request.CTRL_START
                    req.navi_type = NaviTaskCtrl.Request.NAVI_POSE_NAME
                    req.position_name = " ".join(bits[1:])
                elif what == "xy" and len(bits) >= 4:
                    req.ctrl_type = NaviTaskCtrl.Request.CTRL_START
                    req.navi_type = NaviTaskCtrl.Request.NAVI_COORDINATE
                    req.x, req.y, req.theta = float(bits[1]), float(bits[2]), float(bits[3])
                else:
                    print("CMD_ERR NAVI", what, flush=True)
                    continue
                r = call(n, navi_cli, req, 8.0)
                print("CMD_OK NAVI", what, r is not None, flush=True)
                continue
            if op == "POINT":
                bits = arg.split(None, 2)
                mid = bits[0] if bits else ""
                name = bits[1] if len(bits) > 1 else "这里"
                req = SetNaviPoint.Request()
                req.header = cr(n)
                req.map_id = mid
                req.is_auto = True
                req.point.point_id = ""
                req.point.point_name = name
                req.point.point_type = NaviPointInfo.POINT_TYPE_NAVI
                r = call(n, point_cli, req, 6.0)
                print("CMD_OK POINT", r is not None, getattr(r, "point_id", "") if r else "", flush=True)
                continue
            print("CMD_ERR unknown", op, flush=True)
        except Exception as exc:
            print("CMD_ERR", op, str(exc)[:160], flush=True)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
