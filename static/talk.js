/* Q1 对讲。上下行都走这一条 /ws/talk：
     按住说话 → 浏览器麦克风 → 24 kHz 单声道 PCM 二进制帧 → 机器人喇叭
     收听开关 → 机器人麦克风 → 同样的帧 → 浏览器扬声器

   为什么不是 fetch 流式 POST：**浏览器不支持流式请求体**。duplex:"half" 这个成员
   存在、也不报参数错，但请求一律 TypeError: Failed to fetch —— Chromium 148 /
   Chrome 151 / Edge 152 都实测过。ws.send() 发二进制各浏览器都支持，所以改 WS。

   IIFE 包起来，只暴露 window.Q1Talk —— 这几个文件和内联脚本共用一个全局作用域。 */
(function () {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const RATE = 24000; // 上行下发都用这个率：机器人原生播放率，也是 48k 的整数一半
  const JITTER = 0.12; // 下行抖动缓冲，秒。太小会断断续续，太大会像打电话有回音
  const PROBE_SECS = 25; // 试话筒自动关掉的时间。一直开着浏览器地址栏会一直挂个录音标
  const UL_GAIN = 2.2; // 上行增益：机器人侧播出来偏小。乘法 + 硬限幅，不够再往上调

  let ctx = null;
  let mic = null;
  let node = null;
  let srcNode = null;
  let ws = null;
  let wsBusy = false; // 正在建连接
  let talking = false; // 正在往机器人送
  let wantTalk = false; // 手指还按着（区别于 talking：建麦要时间，这中间手指可能已经抬了）
  let micOpen = false; // 麦克风已打开（按住说话或试话筒），电平条在跳
  let listening = false;
  let arming = false;
  let probeTimer = null;
  let reopenTimer = 0; // 收听中断线后的重连定时器
  let gain = null; // 下行音量（按住说话时压到 0，免得听见自己）
  let playAt = 0; // 下行排到 ctx.currentTime 的哪个位置

  /* ── 双向电平 ──────────────────────────────────────────────
     用 dBFS 不用裸 RMS：说话有大有小，差几十倍，线性刻度上小声全挤在零头，
     看着像没声。dB 是人对响度的感觉，刻度才铺得开。
     -58…-8 dBFS 是显示范围，-45 以上算「在说话」。 */
  const VU_FLOOR = -58;
  const VU_CEIL = -8;
  const VU_TALK = -45;
  const VU_HOLD = 320; // 说话判定保持这么久，免得字头字尾闪
  const vu = { me: 0, them: 0, meAt: 0, themAt: 0 };
  let vuLast = 0;

  function dbfs(i16) {
    let s = 0;
    for (let i = 0; i < i16.length; i++) s += i16[i] * i16[i];
    const rms = Math.sqrt(s / Math.max(1, i16.length));
    return 20 * Math.log10(Math.max(rms, 1e-6) / 32768);
  }

  /* 只负责「起」：取大值。落交给 vuLoop 按时间衰减 ——
     时间衰减跟帧率无关，帧率高的机器上不会掉得更快。 */
  function vuHit(who, i16) {
    const db = dbfs(i16);
    const t = Math.max(0, Math.min(1, (db - VU_FLOOR) / (VU_CEIL - VU_FLOOR)));
    if (t > vu[who]) vu[who] = t;
    if (db > VU_TALK) vu[who + "At"] = performance.now();
  }

  function vuState() {
    const box = $("vu");
    if (!box) return;
    // 三态要分得出来：麦没开=压暗；麦开着但没按=中性色（收得到、没发出去）；按着=橙色
    box.classList.toggle("off-me", !micOpen);
    box.classList.toggle("off-them", !listening);
    box.classList.toggle("tx", talking);
    const lab = $("micLab");
    if (lab) lab.textContent = micOpen ? "关话筒" : "试话筒";
    const btn = $("btnMic");
    if (btn) {
      btn.classList.toggle("on", micOpen);
      btn.setAttribute("aria-pressed", micOpen ? "true" : "false");
    }
  }

  const VU_ROWS = [
    ["me", "vuMeBar", "vuMe"],
    ["them", "vuThemBar", "vuThem"],
  ];

  function vuLoop(ts) {
    const dt = Math.min(0.1, (ts - vuLast) / 1000 || 0);
    vuLast = ts;
    const rel = Math.pow(0.08, dt); // 一秒掉到 8%
    const now = performance.now();
    for (const [who, barId, labId] of VU_ROWS) {
      if (vu[who] > 0) vu[who] = Math.max(0, vu[who] * rel);
      const bar = $(barId);
      if (bar) {
        const w = Math.round(vu[who] * 100);
        if (bar.dataset.w !== String(w)) {
          bar.dataset.w = String(w);
          bar.style.width = w + "%";
        }
      }
      const lab = $(labId);
      if (lab) {
        // 「我」这条只在真的在送的时候才算「在说话」—— 试话筒时条子在跳，
        // 但一个字都没发出去，点亮就骗人了
        const on = who === "me"
          ? talking && now - vu.meAt < VU_HOLD
          : now - vu.themAt < VU_HOLD;
        if (lab.classList.contains("on") !== on) lab.classList.toggle("on", on);
      }
    }
    requestAnimationFrame(vuLoop);
  }

  function tag(text, cls) {
    const el = $("talkTag");
    if (!el) return;
    el.textContent = text;
    el.className = "tag" + (cls ? " " + cls : "");
  }

  /* 话筒那一行小字：名字或具体原因。用户问「是不是我麦克风坏了」时，
     答案得写在页面上，不能只留在控制台日志里。 */
  function micInfo(text, cls) {
    const el = $("micInfo");
    if (!el) return;
    el.textContent = text || "";
    el.className = "kv" + (cls ? " " + cls : "");
    el.hidden = !text;
  }

  function live_(on) {
    const btn = $("btnTalk");
    if (btn) btn.classList.toggle("live", !!on);
  }

  function ensureCtx() {
    if (ctx) return true;
    // 必须在用户手势里建并 resume，否则一直挂起；也别传 sampleRate，会和设备对不上
    ctx = new AudioContext();
    if (ctx.state === "suspended") ctx.resume();
    gain = ctx.createGain();
    gain.connect(ctx.destination);
    return true;
  }

  function connect() {
    if (ws && (ws.readyState === 0 || ws.readyState === 1)) return ws;
    if (wsBusy) return null;
    wsBusy = true;
    const url = (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws/talk";
    let sock;
    try {
      sock = new WebSocket(url);
    } catch (_) {
      wsBusy = false;
      tag("连不上对讲口", "bad");
      return null;
    }
    sock.binaryType = "arraybuffer";
    ws = sock;
    sock.onopen = () => {
      wsBusy = false;
      if (listening) sock.send(JSON.stringify({ listen: true }));
      tag(talking ? "讲话中" : "已连接");
    };
    sock.onmessage = (e) => {
      if (typeof e.data === "string") {
        try {
          const j = JSON.parse(e.data);
          if (j.err) tag(j.err, "bad");
          else if (j.robot) {
            const n = j.robot.chunks || 0;
            tag(n ? "已送达 " + Math.round((j.sent || 0) / 48) + " 毫秒" : "没送到机器人",
                n ? "ok" : "bad");
          }
        } catch (_) {
          /* 不是给我们看的 */
        }
        return;
      }
      if (listening) pushDown(e.data);
    };
    sock.onclose = () => {
      wsBusy = false;
      if (ws === sock) ws = null;
      playAt = 0;
      if (talking) live_(false);
      if (listening) {
        /* 「收听」是个持续意图，断了要自己接回来。以前只在标签上写「下行断了」
           就不管了 —— 后果是服务器一重启（改完后端必然撞上），耳朵里就静了，
           而页面看着还像在收听，得手动关一次再开一次才恢复。
           等 800ms 再连：服务器重启那几秒连上去也是立刻断，白刷一条「下行断了」。 */
        tag("下行断了，重连中", "warn");
        window.clearTimeout(reopenTimer);
        reopenTimer = window.setTimeout(() => {
          if (!listening) return;
          const s = connect();
          if (s && s.readyState === 1) s.send(JSON.stringify({ listen: true }));
        }, 800);
      }
    };
    sock.onerror = () => tag("对讲口出错", "bad");
    return sock;
  }

  function pushDown(buf) {
    // 下行是 24 kHz 单声道 S16LE 裸块，交给 Web Audio 排期播
    if (!ctx || !gain) return;
    const i16 = new Int16Array(buf);
    if (!i16.length) return;
    vuHit("them", i16); // 电平在转成 float 之前算，用原始整数
    const f32 = new Float32Array(i16.length);
    for (let i = 0; i < i16.length; i++) f32[i] = i16[i] / 32768;
    const ab = ctx.createBuffer(1, f32.length, RATE);
    ab.copyToChannel(f32, 0);
    const src = ctx.createBufferSource();
    src.buffer = ab;
    src.connect(gain);
    const now = ctx.currentTime;
    if (playAt < now + 0.05) playAt = now + JITTER; // 追上了/断流了，重锚
    else if (playAt > now + 0.6) playAt = now + JITTER; // 积压太多，丢掉重排
    src.start(playAt);
    playAt += ab.duration;
  }

  /* 麦克风权限/设备的失败原因要分开说。以前一律「麦不可用」，
     用户没法自己判断是没插、被占了、还是地址不对 —— 这三件事的解法完全不同。 */
  function micError(err) {
    const name = (err && err.name) || "";
    if (name === "NotAllowedError" || name === "SecurityError")
      return "麦克风被拒：点地址栏左边的图标，把「麦克风」改成允许";
    if (name === "NotFoundError" || name === "OverconstrainedError")
      return "这台机器上没有可用的麦克风";
    if (name === "NotReadableError" || name === "AbortError")
      return "麦克风被别的程序占着（会议软件/录屏），关掉再试";
    return "麦克风打不开：" + (name || err);
  }

  async function ensureMic() {
    if (mic && node) return true;
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      // 非安全上下文（局域网 IP / http 主机名）连 mediaDevices 都不存在
      micInfo("这个网址不给用麦克风 —— 浏览器只允许 127.0.0.1 或 https", "bad");
      tag("浏览器不给麦克风", "bad");
      return false;
    }
    try {
      mic = await navigator.mediaDevices.getUserMedia({
        audio: {
          channelCount: 1,
          echoCancellation: true,
          noiseSuppression: true,
          autoGainControl: true,
        },
      });
    } catch (err) {
      micInfo(micError(err), "bad");
      tag(err && err.name === "NotAllowedError" ? "麦克风被拒" : "麦不可用", "bad");
      return false;
    }
    ensureCtx();
    try {
      await ctx.audioWorklet.addModule("/static/talk-worklet.js?v=15");
    } catch (err) {
      micInfo("音频处理模块加载失败：" + err, "bad");
      stopMic();
      return false;
    }
    srcNode = ctx.createMediaStreamSource(mic);
    node = new AudioWorkletNode(ctx, "q1-talk");
    srcNode.connect(node); // 不接 destination，免得自己听自己
    node.port.onmessage = (e) => {
      // 电平永远算：试话筒时也要看得到格子跳，不然没法判断麦克风到底有没有在收
      vuHit("me", e.data);
      if (!talking || !ws || ws.readyState !== 1) return; // 没在按住就只显示、不发送
      // 上行增益：放大 + 硬限幅，别把尖峰顶成回卷的爆音
      const up = new Int16Array(e.data);
      for (let i = 0; i < up.length; i++) {
        const v = Math.round(up[i] * UL_GAIN);
        up[i] = v > 32767 ? 32767 : v < -32768 ? -32768 : v;
      }
      ws.send(up.buffer); // Int16Array 的字节就是小端 S16LE，直接当裸字节发
    };
    micOpen = true;
    const tr = mic.getAudioTracks()[0];
    micInfo("话筒：" + ((tr && tr.label) || "默认设备"));
    vuState();
    return true;
  }

  /* 关掉麦克风：松开设备，浏览器地址栏那个录音标记才会消失。 */
  function stopMic() {
    if (talking) return; // 正在送就先别关
    if (node) { try { node.disconnect(); } catch (_) {} node = null; }
    if (srcNode) { try { srcNode.disconnect(); } catch (_) {} srcNode = null; }
    if (mic) { mic.getTracks().forEach((t) => { try { t.stop(); } catch (_) {} }); mic = null; }
    micOpen = false;
    vu.me = 0;
    vuState();
  }

  /* 试话筒：只开麦、不发送。用来回答「是不是我麦克风坏了」——
     格子会跳就是好的，不跳看下面那行字，写的是具体原因。 */
  async function probe() {
    if (probeTimer) { clearTimeout(probeTimer); probeTimer = null; }
    if (micOpen) {
      if (talking) return;
      stopMic();
      micInfo("");
      tag("—");
      return;
    }
    ensureCtx();
    if (ctx.state === "suspended") await ctx.resume();
    if (!(await ensureMic())) return;
    tag("话筒在收 · 说两句看格子", "acc");
    probeTimer = setTimeout(() => {
      probeTimer = null;
      if (!talking) { stopMic(); micInfo(""); tag("—"); }
    }, PROBE_SECS * 1000);
  }

  async function press(ev) {
    if (ev) ev.preventDefault();
    if (talking || arming) return;
    wantTalk = true;
    arming = true;
    try {
      connect(); // 先连上，等权限和音频上下文的那段时间正好用来握手
      ensureCtx();
      if (ctx.state === "suspended") await ctx.resume();
      if (!(await ensureMic())) return;
    } catch (err) {
      tag("音频起不来", "bad");
      return;
    } finally {
      arming = false;
    }
    /* 手指可能在等权限/建麦的时候就已经抬了（第一次用会弹权限框，一弹就丢指针）。
       这时候不能再进 talking —— 否则松了手还在送，而且下一次按会被
       `if (talking) return` 吃掉，表现为「按了没反应」。 */
    if (!wantTalk) {
      if (ws && ws.readyState === 1) ws.send(JSON.stringify({ end: true }));
      if (!probeTimer) { stopMic(); micInfo(""); }
      return;
    }
    talking = true;
    live_(true);
    tag("讲话中", "acc");
    vuState();
    if (gain) gain.gain.value = 0; // 半双工：说话时不听，否则机器人麦克风把喇叭声收回来自激
  }

  function release(ev) {
    if (ev) ev.preventDefault();
    wantTalk = false;
    if (!talking) return;
    talking = false;
    live_(false);
    vuState();
    if (gain) gain.gain.value = listening ? 1 : 0;
    if (ws && ws.readyState === 1) {
      // 先给个「送出去了」的即时反馈。只等服务端回执的话，回执没来之前按钮已经弹起来了、
      // 标签还写着「讲话中」，两处对不上，看着像没松手。
      tag("送出中…", "acc");
      // 不关连接：收听可能还开着，关掉会把下行一起掐断。发个 end 让服务端收尾。
      ws.send(JSON.stringify({ end: true }));
    } else {
      tag("松开了");
    }
    // 试话筒开着的话，麦继续留着；否则松开就还回去
    if (!probeTimer) window.setTimeout(() => { if (!talking && !probeTimer) stopMic(); }, 400);
  }

  async function toggleListen(ev) {
    if (ev) ev.preventDefault();
    const btn = $("btnListen");
    if (!listening) {
      try {
        ensureCtx();
        if (ctx.state === "suspended") await ctx.resume();
      } catch (err) {
        tag("音频起不来", "bad");
        return;
      }
      listening = true;
      playAt = 0;
      if (gain) gain.gain.value = talking ? 0 : 1;
      const sock = connect();
      if (sock && sock.readyState === 1) sock.send(JSON.stringify({ listen: true }));
      tag("收听中", "acc");
    } else {
      listening = false;
      playAt = 0;
      window.clearTimeout(reopenTimer); // 撤销挂起的重连，别关了还在后台重连
      if (ws && ws.readyState === 1) ws.send(JSON.stringify({ listen: false }));
      tag(talking ? "讲话中" : "—");
    }
    vuState();
    if (btn) {
      btn.classList.toggle("on", listening);
      btn.setAttribute("aria-pressed", listening ? "true" : "false");
    }
    const lab = $("listenLab");
    if (lab) lab.textContent = listening ? "停止" : "收听";
  }

  function boot() {
    const btn = $("btnTalk");
    if (btn) {
      btn.addEventListener("pointerdown", press);
      btn.addEventListener("pointerup", release);
      btn.addEventListener("pointercancel", release);
      btn.addEventListener("pointerleave", release);
      btn.addEventListener("contextmenu", (e) => e.preventDefault());
      // 键盘也要能用：按住空格/回车
      btn.addEventListener("keydown", (e) => {
        if ((e.key === " " || e.key === "Enter") && !e.repeat) press(e);
      });
      btn.addEventListener("keyup", (e) => {
        if (e.key === " " || e.key === "Enter") release(e);
      });
    }
    const lis = $("btnListen");
    if (lis) lis.addEventListener("click", toggleListen);
    const pm = $("btnMic");
    if (pm) pm.addEventListener("click", probe);
    tag("—");
    vuState();
    requestAnimationFrame(vuLoop);
  }

  window.Q1Talk = { boot, press, release, toggleListen, probe };
})();
