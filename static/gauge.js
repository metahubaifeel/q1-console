/* Q1 总控台 · 姿态仪 + 速度矢量
   读 live.imu(pitch/roll) 与 live.vel(fwd/lat/yaw)，没有数据就画静止刻度。
   颜色跟随 app.css 的 CSS 变量，别写死。 */
(function () {
"use strict";


/* 值缓存一次——运行期变量不会变，没必要每帧去问 getComputedStyle。 */
const _cv = {};
function cssv(name, fb) {
  if (_cv[name] === undefined) {
    _cv[name] = getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fb;
  }
  return _cv[name];
}
const C = {
  ink:  () => cssv("--ink", "#f2f5f8"),
  mute: () => cssv("--mute", "#b7c1cc"),
  dim:  () => cssv("--dim2", "#758295"),
  acc:  () => cssv("--acc", "#e85d1c"),
  ok:   () => cssv("--ok", "#34d399"),
  bad:  () => cssv("--bad", "#ef4343"),
  bg:   () => cssv("--void", "#05070c"),
  sky:  () => cssv("--sky", "#0b1420"),
  gnd:  () => cssv("--gnd", "#20120a"),
};
/* 字体栈跟着 --m 走；canvas 的 font 是简写，得拼成 weight + size + family */
function mono(px, weight) {
  return (weight ? weight + " " : "") + px + "px " + cssv("--m", "ui-monospace, monospace");
}

/* 支持 HiDPI：读 canvas 自己的 CSS 盒子设物理像素，返回 CSS 像素坐标系。
   别用父元素尺寸——父元素有 padding，会算出比画布大的值。 */
function fit(canvas) {
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const w = Math.min(4096, Math.max(40, canvas.clientWidth || 200));
  const h = Math.min(4096, Math.max(40, canvas.clientHeight || 108));
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

/* ── 姿态仪：地平线 + 俯仰梯 + 滚转弧 ───────────────────── */
const att = { pitch: 0, roll: 0, ok: false };

function drawAtt(canvas) {
  const { ctx, w, h } = fit(canvas);
  const cx = w / 2, cy = h / 2;
  const R = Math.min(w * 0.42, h * 0.44);

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = C.bg();
  ctx.fillRect(0, 0, w, h);

  ctx.save();
  ctx.beginPath();
  ctx.arc(cx, cy, R, 0, Math.PI * 2);
  ctx.clip();

  /* 天地 */
  const ppd = R / 32;                       // 每度像素：±32° 填满
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate((att.roll * Math.PI) / 180);
  const off = att.pitch * ppd;
  ctx.fillStyle = C.sky();
  ctx.fillRect(-R * 2, -R * 2 + off, R * 4, R * 2);
  ctx.fillStyle = C.gnd();
  ctx.fillRect(-R * 2, off, R * 4, R * 2);
  ctx.strokeStyle = C.acc() + "cc";
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.moveTo(-R * 2, off);
  ctx.lineTo(R * 2, off);
  ctx.stroke();

  /* 俯仰梯：每 10° */
  ctx.font = mono(9);
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  for (let d = -30; d <= 30; d += 10) {
    if (d === 0) continue;
    const y = off - d * ppd;
    const half = d % 20 === 0 ? R * 0.34 : R * 0.18;
    ctx.strokeStyle = "#ffffff44";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(-half, y);
    ctx.lineTo(half, y);
    ctx.stroke();
    if (d % 20 === 0) {
      ctx.fillStyle = "#ffffff55";
      ctx.fillText(String(Math.abs(d)), -half - 9, y);
      ctx.fillText(String(Math.abs(d)), half + 9, y);
    }
  }
  ctx.restore();

  /* 滚转刻度弧 */
  ctx.save();
  ctx.translate(cx, cy);
  ctx.strokeStyle = "#ffffff33";
  ctx.lineWidth = 1;
  for (const a of [-30, -20, -10, 0, 10, 20, 30]) {
    const rad = ((a - 90) * Math.PI) / 180;
    const r0 = R * 0.93, r1 = a % 30 === 0 ? R * 0.80 : R * 0.87;
    ctx.beginPath();
    ctx.moveTo(Math.cos(rad) * r0, Math.sin(rad) * r0);
    ctx.lineTo(Math.cos(rad) * r1, Math.sin(rad) * r1);
    ctx.stroke();
  }
  /* 滚转指针 */
  const rr = (-att.roll - 90) * (Math.PI / 180);
  ctx.fillStyle = Math.abs(att.roll) > 25 ? C.bad() : C.acc();
  ctx.beginPath();
  ctx.moveTo(Math.cos(rr) * R * 0.79, Math.sin(rr) * R * 0.79);
  ctx.lineTo(Math.cos(rr - 0.055) * R * 0.90, Math.sin(rr - 0.055) * R * 0.90);
  ctx.lineTo(Math.cos(rr + 0.055) * R * 0.90, Math.sin(rr + 0.055) * R * 0.90);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  /* 固定机标 */
  ctx.strokeStyle = C.acc();
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cx - R * 0.52, cy); ctx.lineTo(cx - R * 0.20, cy);
  ctx.moveTo(cx + R * 0.20, cy); ctx.lineTo(cx + R * 0.52, cy);
  ctx.moveTo(cx, cy - 3); ctx.lineTo(cx, cy + 3);
  ctx.stroke();
  ctx.restore();

  /* 外圈 */
  ctx.strokeStyle = "#ffffff26";
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(cx, cy, R, 0, Math.PI * 2); ctx.stroke();

  /* 读数：标签在左，数字在右，中间一条分隔线把上下两行分开 */
  const ro = (label, val, y, col, on) => {
    ctx.textAlign = "left"; ctx.textBaseline = "middle";
    ctx.fillStyle = C.dim();
    ctx.font = mono(9);
    ctx.fillText(label, 8, y);
    ctx.textAlign = "right";
    ctx.fillStyle = on ? col : C.dim();
    ctx.font = mono(15, 600);
    ctx.fillText(val, w - 8, y);
  };
  const mid = (6 + (h - 15)) / 2;
  ctx.strokeStyle = "#ffffff12";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(6, mid); ctx.lineTo(w - 6, mid);
  ctx.stroke();
  ro("PITCH", att.ok ? att.pitch.toFixed(1) + "°" : "——", 6 + 7, C.ink(), att.ok);
  ro("ROLL", att.ok ? att.roll.toFixed(1) + "°" : "——", h - 15 + 7,
     Math.abs(att.roll) > 25 ? C.bad() : C.ink(), att.ok);
}

/* ── 速度矢量：底盘俯视，箭头长度=合速度，弧=角速度 ─────── */
const vel = { fwd: 0, lat: 0, yaw: 0, ok: false };

function drawVel(canvas) {
  const { ctx, w, h } = fit(canvas);
  const cx = w / 2, cy = h / 2;
  const R = Math.min(w * 0.42, h * 0.44);

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = C.bg();
  ctx.fillRect(0, 0, w, h);

  /* 底盘框 */
  ctx.strokeStyle = "#ffffff1f";
  ctx.lineWidth = 1;
  const bw = R * 0.86, bh = R * 1.0;
  ctx.beginPath();
  ctx.rect(cx - bw / 2, cy - bh / 2, bw, bh);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(cx - bw / 2, cy - bh * 0.28);
  ctx.lineTo(cx + bw / 2, cy - bh * 0.28);
  ctx.stroke();

  /* 速度环 */
  for (const f of [0.33, 0.66, 1]) {
    ctx.strokeStyle = "#ffffff14";
    ctx.beginPath(); ctx.arc(cx, cy, R * f, 0, Math.PI * 2); ctx.stroke();
  }
  ctx.beginPath();
  ctx.moveTo(cx - R, cy); ctx.lineTo(cx + R, cy);
  ctx.moveTo(cx, cy - R); ctx.lineTo(cx, cy + R);
  ctx.stroke();

  const MAXV = 0.6;                                   // 满量程 m/s
  const nf = Math.max(-1, Math.min(1, vel.fwd / MAXV));
  const nl = Math.max(-1, Math.min(1, vel.lat / MAXV));
  const mag = Math.min(1, Math.hypot(nf, nl));

  /* 矢量箭头：屏幕 x=横向(右为左移 lat 取反)，y=前 */
  if (mag > 0.012) {
    const ex = cx - nl * R, ey = cy - nf * R;
    const color = mag > 0.9 ? C.bad() : C.acc();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.2;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke();
    const a = Math.atan2(ey - cy, ex - cx);
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(ex, ey);
    ctx.lineTo(ex - Math.cos(a - 0.4) * 8, ey - Math.sin(a - 0.4) * 8);
    ctx.lineTo(ex - Math.cos(a + 0.4) * 8, ey - Math.sin(a + 0.4) * 8);
    ctx.closePath();
    ctx.fill();
  }
  ctx.fillStyle = "#ffffff40";
  ctx.beginPath(); ctx.arc(cx, cy, 2.4, 0, Math.PI * 2); ctx.fill();

  /* 角速度弧 */
  const ny = Math.max(-1, Math.min(1, vel.yaw / 1.0));
  if (Math.abs(ny) > 0.02) {
    const r = R * 1.10;
    ctx.strokeStyle = C.ok();
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(cx, cy, r, -Math.PI / 2, -Math.PI / 2 + ny * Math.PI * 0.9, ny < 0);
    ctx.stroke();
  }

  /* 读数 */
  const ro = (label, val, y, col, on) => {
    ctx.textAlign = "left"; ctx.textBaseline = "middle";
    ctx.fillStyle = C.dim();
    ctx.font = mono(9);
    ctx.fillText(label, 8, y);
    ctx.textAlign = "right";
    ctx.fillStyle = on ? col : C.dim();
    ctx.font = mono(15, 600);
    ctx.fillText(val, w - 8, y);
  };
  const mid = (6 + (h - 15)) / 2;
  ctx.strokeStyle = "#ffffff12";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(6, mid); ctx.lineTo(w - 6, mid);
  ctx.stroke();
  ro("FWD", vel.ok ? vel.fwd.toFixed(2) + " m/s" : "——", 13, C.ink(), vel.ok);
  ro("YAW", vel.ok ? vel.yaw.toFixed(2) + " r/s" : "——", h - 8,
     Math.abs(vel.yaw) > 0.02 ? C.ok() : C.ink(), vel.ok);
}

let dirty = true;
let booted = false;
let lastSig = "";

function update(live) {
  const imu = live && live.imu;
  const v = live && live.vel;
  att.ok = !!(imu && typeof imu.pitch === "number");
  if (att.ok) { att.pitch = Number(imu.pitch) || 0; att.roll = Number(imu.roll) || 0; }
  vel.ok = !!(v && typeof v.fwd === "number");
  if (vel.ok) { vel.fwd = Number(v.fwd) || 0; vel.lat = Number(v.lat) || 0; vel.yaw = Number(v.yaw) || 0; }
  const sig = [att.ok, att.pitch, att.roll, vel.ok, vel.fwd, vel.lat, vel.yaw].join(",");
  if (sig !== lastSig) { lastSig = sig; dirty = true; }
}

function draw() {
  const a = document.getElementById("att");
  if (a) drawAtt(a);
  const s = document.getElementById("svec");
  if (s) drawVel(s);
  dirty = false;
}

function loop() {
  if (dirty) draw();
  requestAnimationFrame(loop);
}

window.Q1Gauge = {
  update,
  redraw() { dirty = true; },
  boot() {
    if (booted) return;
    booted = true;
    window.addEventListener("resize", () => { dirty = true; });
    if (window.ResizeObserver) {
      const ro = new ResizeObserver(() => { dirty = true; });
      for (const id of ["att", "svec"]) {
        const c = document.getElementById(id);
        if (c) ro.observe(c);
      }
    }
    draw();
    requestAnimationFrame(loop);
  },
};
})();
