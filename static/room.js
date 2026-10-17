/* 画布尺寸只跟 CSS 走：读 canvas 自己的盒子，绝不写 style.width/height，
   否则会盖掉 CSS 的 inset:0，把画布锁死在首次测量到的旧尺寸上。 */
(function () {
"use strict";

function fit(canvas) {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  // 上限兜底：万一哪天 CSS 又漏了 width/height，别让它滚成天文数字
  const w = Math.min(4096, Math.max(2, Math.floor(canvas.clientWidth)));
  const h = Math.min(4096, Math.max(2, Math.floor(canvas.clientHeight)));
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

/* 配色一律从 app.css 的变量里取，换主题不用动这个文件。
   值缓存一次——运行期变量不会变，没必要每帧去问 getComputedStyle。 */
const _cv = {};
function cssv(name, fb) {
  if (_cv[name] === undefined) {
    _cv[name] = getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fb;
  }
  return _cv[name];
}
const ink   = () => cssv("--ink", "#f2f5f8");
const mute  = () => cssv("--mute", "#b7c1cc");
const acc   = () => cssv("--acc", "#e85d1c");
const bad   = () => cssv("--bad", "#ef4343");
const dim2  = () => cssv("--dim2", "#758295");
const voidc = () => cssv("--void", "#05070c");
const ln2   = () => cssv("--ln2", "#272b35");
const s3    = () => cssv("--s3", "#161a22");
/* 字体栈跟着 --m 走；canvas 的 font 是简写，得拼成 weight + size + family */
function mono(px, weight) {
  return (weight ? weight + " " : "") + px + "px " + cssv("--m", "ui-monospace, monospace");
}

const roomMap = {
  img: null,
  tick: "",
  points: [],
};

function loadMapImage(tick) {
  if (tick === roomMap.tick && roomMap.img) return;
  roomMap.tick = tick;
  const im = new Image();
  im.onload = () => {
    roomMap.img = im;
  };
  im.src = "/api/map.png?t=" + encodeURIComponent(tick);
}

function drawRoom(canvas, live) {
  const host = canvas.parentElement || canvas;
  const { ctx, w, h } = fit(canvas);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = voidc();
  ctx.fillRect(0, 0, w, h);
  const m = live.map || {};
  const info = live.map_info;
  const has = (m.has_png || (info && info.w)) && roomMap.img;
  if (m.has_png || (info && info.w)) {
    loadMapImage(String(m.png_bytes || info?.w || 0) + ":" + (m.current || m.task_id || info?.id || ""));
  }
  const pad = 10;
  let ox = pad, oy = pad, dw = w - pad * 2, dh = h - pad * 2;
  if (has && roomMap.img.width) {
    const s = Math.min(dw / roomMap.img.width, dh / roomMap.img.height);
    dw = roomMap.img.width * s;
    dh = roomMap.img.height * s;
    ox = (w - dw) / 2;
    oy = (h - dh) / 2;
    ctx.drawImage(roomMap.img, ox, oy, dw, dh);
  } else {
    ctx.fillStyle = dim2();
    ctx.font = mono(12);
    ctx.fillText("无底图", 14, h / 2 + 4);
  }
  const pts = live.points || [];
  roomMap.points = pts.map((p) => {
    const px = ox + (p.x / ((info && info.w) || roomMap.img?.width || 1)) * dw;
    const py = oy + (p.y / ((info && info.h) || roomMap.img?.height || 1)) * dh;
    return { ...p, px, py };
  });
  for (const p of roomMap.points) {
    ctx.beginPath();
    ctx.fillStyle = acc();
    ctx.arc(p.px, p.py, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = voidc();
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.fillStyle = ink();
    ctx.font = mono(11);
    ctx.fillText(p.name || p.id || "点", p.px + 7, p.py + 4);
  }
  const odom = live.odom;
  if (has && odom && info && info.resolution) {
    const rx = ox + ((odom.x - info.origin.x) / (info.resolution * (info.w || 1))) * dw;
    const ry = oy + dh - ((odom.y - info.origin.y) / (info.resolution * (info.h || 1))) * dh;
    if (rx > 0 && ry > 0 && rx < w && ry < h) {
      ctx.save();
      ctx.translate(rx, ry);
      ctx.rotate((-Number(odom.yaw || 0) * Math.PI) / 180);
      ctx.fillStyle = bad();
      ctx.beginPath();
      ctx.moveTo(10, 0);
      ctx.lineTo(-6, 5);
      ctx.lineTo(-6, -5);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }
  }
}

function drawRadar(canvas, live) {
  const host = canvas.parentElement || canvas;
  const { ctx, w, h } = fit(canvas);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = voidc();
  ctx.fillRect(0, 0, w, h);
  const cx = w / 2, cy = h / 2 - 6;     // 往上挪一点，给底部标签和说明条留位
  const r = Math.min(w, h) * 0.30;      // 留出边缘给「前后左右」标签
  ctx.strokeStyle = ln2();
  ctx.lineWidth = 1;
  for (const k of [0.35, 0.7, 1]) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * k, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.beginPath();
  ctx.moveTo(cx - r, cy);
  ctx.lineTo(cx + r, cy);
  ctx.moveTo(cx, cy - r);
  ctx.lineTo(cx, cy + r);
  ctx.stroke();
  const dirs = [
    { code: [1, 5, 9], ang: -Math.PI / 2, lab: "前" },
    { code: [2, 6, 10], ang: Math.PI, lab: "左" },
    { code: [3, 7, 11], ang: 0, lab: "右" },
    { code: [4, 8, 12], ang: Math.PI / 2, lab: "后" },
  ];
  const hit = live.fusion ? Number(live.fusion.code) : 0;
  for (const d of dirs) {
    const on = d.code.includes(hit);
    ctx.fillStyle = on ? bad() : s3();
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, r * 0.92, d.ang - 0.35, d.ang + 0.35);
    ctx.closePath();
    ctx.globalAlpha = on ? 0.85 : 0.35;
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.fillStyle = on ? ink() : dim2();
    ctx.font = mono(11);
    ctx.fillText(d.lab, cx + Math.cos(d.ang) * (r + 12) - 6, cy + Math.sin(d.ang) * (r + 12) + 4);
  }
  for (const p of live.dangers || []) {
    const ang = Math.atan2(p.y, p.x);
    const dist = Math.min(1, Math.hypot(p.x, p.y) / 1.6);
    ctx.fillStyle = acc();
    ctx.beginPath();
    ctx.arc(cx + Math.cos(ang) * r * dist, cy + Math.sin(ang) * r * dist, 4, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.fillStyle = acc();
  ctx.beginPath();
  ctx.moveTo(cx, cy - 8);
  ctx.lineTo(cx + 5, cy + 6);
  ctx.lineTo(cx - 5, cy + 6);
  ctx.closePath();
  ctx.fill();
}

function drawHeading(canvas, live) {
  const host = canvas.parentElement || canvas;
  const { ctx, w, h } = fit(canvas);
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = voidc();
  ctx.fillRect(0, 0, w, h);
  const yaw = Number((live.imu && live.imu.yaw) ?? (live.odom && live.odom.yaw) ?? 0);
  const cx = w / 2, cy = h / 2, r = Math.min(w, h) * 0.36;
  ctx.strokeStyle = ln2();
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.stroke();
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate((-yaw * Math.PI) / 180);
  ctx.fillStyle = acc();
  ctx.beginPath();
  ctx.moveTo(0, -r + 6);
  ctx.lineTo(8, 10);
  ctx.lineTo(0, 4);
  ctx.lineTo(-8, 10);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
  ctx.fillStyle = dim2();
  ctx.font = mono(11);
  ctx.fillText("N", cx - 4, cy - r - 6);
  ctx.fillStyle = ink();
  ctx.font = mono(16, 600);
  ctx.textAlign = "center";
  ctx.fillText(Math.round(yaw) + "°", cx, cy + r + 24);
  ctx.textAlign = "left";
}

function nearestPoint(x, y) {
  let best = null, d0 = 22;
  for (const p of roomMap.points) {
    const d = Math.hypot(p.px - x, p.py - y);
    if (d < d0) {
      d0 = d;
      best = p;
    }
  }
  return best;
}

let lastLive = {};

window.Q1Room = {
  draw(live) {
    if (live) lastLive = live;
    const a = document.getElementById("roomMap");
    const b = document.getElementById("radar");
    const c = document.getElementById("heading");
    if (a) drawRoom(a, lastLive);
    if (b) drawRadar(b, lastLive);
    if (c) drawHeading(c, lastLive);
    /* 没数据的画布在手机上纯占地方（导航面板实测 1215px 高，一半是空白框）。
       标 dead 类让 CSS 在窄屏把它收成一行状态文字；桌面端保持原框 ——
       空框在桌面上是「这块有位置、只是没数据」，收掉反而像坏了。 */
    const m = (lastLive && lastLive.map) || {};
    const info = lastLive && lastLive.map_info;
    const setViz = (id, alive) => {
      const el = document.getElementById(id);
      const viz = el && el.closest(".viz");
      if (viz) viz.classList.toggle("dead", !alive);
    };
    setViz("roomMap", !!(m.has_png || (info && info.w)));
    setViz("radar", !!(lastLive && lastLive.fusion));
    setViz("heading", !!(lastLive && lastLive.imu));
  },
  hit(ev) {
    const canvas = document.getElementById("roomMap");
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    return nearestPoint(ev.clientX - rect.left, ev.clientY - rect.top);
  },
};

/* 盒子尺寸变了就重画一遍（离线时没有 live 推送，靠这个才不会糊） */
if (window.ResizeObserver) {
  const ro = new ResizeObserver(() => window.Q1Room.draw());
  for (const id of ["roomMap", "radar", "heading"]) {
    const c = document.getElementById(id);
    if (c) ro.observe(c);
  }
}
window.addEventListener("resize", () => window.Q1Room.draw());
})();
