import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const URDF = "/static/urdf/q1/q1.urdf";
const MESH_DIR = "/static/urdf/q1/meshes/";

/* 三维视口也要跟 app.css 的配色走 —— 它和相机画面并排，
   底色差一点就看得出来是两块东西。值缓存，别每帧问。 */
const _cv = {};
function cssv(name, fb) {
  if (_cv[name] === undefined) {
    _cv[name] = getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fb;
  }
  return _cv[name];
}

function nums(s, n) {
  const a = String(s || "")
    .trim()
    .split(/\s+/)
    .map(Number);
  while (a.length < n) a.push(0);
  return a.slice(0, n);
}

function originOf(el) {
  const o = el && [...el.children].find((c) => c.tagName === "origin");
  return {
    xyz: nums(o && o.getAttribute("xyz"), 3),
    rpy: nums(o && o.getAttribute("rpy"), 3),
  };
}

function applyOrigin(obj, xyz, rpy) {
  obj.position.set(xyz[0], xyz[1], xyz[2]);
  obj.rotation.set(rpy[0], rpy[1], rpy[2], "XYZ");
}

function meshName(filename) {
  return String(filename || "")
    .trim()
    .split(/[/\\]/)
    .pop();
}

function colorOf(vis) {
  const c = vis && vis.querySelector("material color");
  if (!c) return new THREE.Color(cssv("--ink", "#f2f5f8"));
  const [r, g, b] = nums(c.getAttribute("rgba"), 4);
  return new THREE.Color(r, g, b);
}

/* 折叠/低姿判定走 mc（McCommonState 实测值）：
   form=1 是折叠收纳；action_id 1001=STORE_DEFAULT 折叠、1013=坐下、
   1008/1009/1010=趴；motion 名带 SIT/STORE/LIE/KNEEL 也算。
   站高字段在折叠时报 0.0 —— 但好好站着时它也是 0.0，所以
   「站还是折」不能靠站高，只能靠 mc 的这些信号。 */
function foldPose(mc) {
  if (!mc) return false;
  if (mc.form === 1) return true;
  if (mc.action_id != null && [1001, 1013, 1008, 1009, 1010].includes(Number(mc.action_id))) return true;
  return /SIT|STORE|LIE|KNEEL/.test(String(mc.motion || ""));
}

function guessLegs(body, mc) {
  const h = Number(body && body.stand_height);
  /* 机器人不下发腿部关节角（/upper_body_joint_states 只发上半身，
     /tf 是空的，没有全身 /joint_states），腿只能按站高估。
     但站高报 0 是「没上报」而不是「蹲到底」—— 实测机器人好好站着时
     这个字段就是 0.0。当成 0 会让孪生一直蹲着，比不显示还误导。
     折叠/真摔了有 mc 说，不靠这个字段。 */
  const folded = foldPose(mc) || !!(mc && mc.fallen);
  const t = folded ? 0 : Number.isFinite(h) && h > 0.02 ? Math.max(0, Math.min(1, h / 0.45)) : 1;
  const lerp = (a, b) => a + (b - a) * t;
  const out = {
    left_hip_pitch_joint: lerp(-1.15, -0.28),
    left_hip_roll_joint: lerp(0.22, 0.04),
    left_hip_yaw_joint: 0,
    left_knee_joint: lerp(1.85, 0.55),
    left_ankle_pitch_joint: lerp(-0.55, -0.28),
    left_ankle_roll_joint: 0,
    right_hip_pitch_joint: lerp(-1.15, -0.28),
    right_hip_roll_joint: lerp(-0.22, -0.04),
    right_hip_yaw_joint: 0,
    right_knee_joint: lerp(1.85, 0.55),
    right_ankle_pitch_joint: lerp(-0.55, -0.28),
    right_ankle_roll_joint: 0,
  };
  if (body) {
    if (body.bend_sideways != null) out.waist_roll_joint = Number(body.bend_sideways);
    if (body.twist_waist != null) out.waist_yaw_joint = Number(body.twist_waist);
  }
  return out;
}

/* URDF 里写的是 xxx.STL，仓里放的是 xxx.glb —— 只换扩展名。
   为什么不是直接发 STL：官方 24 个 STL 一共 25 MB / 51 万面，STL 没有顶点索引，
   每个三角形的三个顶点各存一份，同一份几何要重复约三遍。转成索引化 GLB
   再简化+量化之后是 2.3 MB，浏览器拉得动。
   转换脚本见 tools/stl2glb.py，管线：焊接顶点 → simplify 0.3 → 量化。 */
function glbName(stlName) {
  return String(stlName || "").replace(/\.[^.]*$/, "") + ".glb";
}

async function loadGlbs(names) {
  const loader = new GLTFLoader();
  const geos = {};
  const missing = [];
  await Promise.all(
    names.map(async (name) => {
      try {
        const gltf = await loader.loadAsync(MESH_DIR + glbName(name));
        gltf.scene.updateMatrixWorld(true);
        let geo = null;
        gltf.scene.traverse((o) => {
          if (geo || !o.isMesh || !o.geometry) return;
          geo = o.geometry;
          /* 必须把节点变换烘进几何。
             量化后的 GLB 里，顶点是 ±1 的 int16，真实的米数和偏移写在节点的
             translation/scale 上（gltf-transform 的量化就是这么落的）。
             我们只拿 geometry 挂到自己的 URDF 骨架上，节点变换会被丢掉 ——
             不烘进去，腰那件会大 28 倍并整体偏出去，相机直接埋在模型里。
             第一版就是漏了这一步，截出来一片糊。 */
          geo.applyMatrix4(o.matrixWorld);
        });
        if (!geo) throw new Error("glb 里没有网格");
        geos[name] = geo;
      } catch {
        missing.push(name);
      }
    }),
  );
  return { geos, missing };
}

function collectMeshNames(robotEl) {
  const names = new Set();
  for (const mesh of robotEl.querySelectorAll("visual mesh[filename]")) {
    const n = meshName(mesh.getAttribute("filename"));
    if (n) names.add(n);
  }
  return [...names];
}

function buildRobot(xml, geos) {
  const doc = new DOMParser().parseFromString(xml, "text/xml");
  const robotEl = doc.querySelector("robot");
  if (!robotEl) throw new Error("urdf 坏了");

  const links = {};
  const joints = {};
  const childOf = new Set();
  let meshCount = 0;

  for (const el of robotEl.querySelectorAll("link")) {
    const name = el.getAttribute("name");
    const group = new THREE.Group();
    group.name = name;
    for (const vis of el.querySelectorAll(":scope > visual")) {
      const file = meshName(vis.querySelector("mesh") && vis.querySelector("mesh").getAttribute("filename"));
      const geo = file && geos[file];
      if (!geo) continue;
      const mat = new THREE.MeshStandardMaterial({
        color: colorOf(vis),
        metalness: 0.12,
        roughness: 0.46,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.castShadow = false;
      mesh.receiveShadow = false;
      const o = originOf(vis);
      const wrap = new THREE.Group();
      applyOrigin(wrap, o.xyz, o.rpy);
      wrap.add(mesh);
      group.add(wrap);
      meshCount += 1;
    }
    links[name] = group;
  }

  for (const el of robotEl.querySelectorAll("joint")) {
    const name = el.getAttribute("name");
    const type = el.getAttribute("type") || "fixed";
    const parent = el.querySelector("parent")?.getAttribute("link");
    const child = el.querySelector("child")?.getAttribute("link");
    if (!parent || !child || !links[parent] || !links[child]) continue;
    const o = originOf(el);
    const axisEl = el.querySelector("axis");
    const axis = new THREE.Vector3(...nums(axisEl && axisEl.getAttribute("xyz"), 3));
    if (axis.lengthSq() < 1e-8) axis.set(0, 0, 1);
    axis.normalize();

    const origin = new THREE.Group();
    origin.name = name + "_origin";
    applyOrigin(origin, o.xyz, o.rpy);
    const moving = new THREE.Group();
    moving.name = name;
    if (name === "waist_yaw_joint") {
      /* 折叠包裹器：插在关节 origin 和 moving 之间，绕**模型 Y 轴**旋转整个上身
         （torso + 头 + 双臂）。绕 Y 正转（右手定则、拇指朝模型左侧）= 上身前倾。
         tilt 是世界系：tilt.rotation.x 绕世界 X，作用到模型上是绕模型 X 的
         **侧翻**不是俯仰 —— 第一版折叠就是错在这，看着像侧倒。 */
      const fold = new THREE.Group();
      fold.name = name + "_fold";
      origin.add(fold);
      fold.add(moving);
      joints[name] = { moving, axis, type, fold };
    } else {
      origin.add(moving);
      joints[name] = { moving, axis, type };
    }
    moving.add(links[child]);
    links[parent].add(origin);
    childOf.add(child);
  }

  const roots = Object.keys(links).filter((n) => !childOf.has(n));
  const root = new THREE.Group();
  root.name = "q1";
  for (const n of roots) root.add(links[n]);
  return { root, joints, meshCount, needed: collectMeshNames(robotEl) };
}

class Twin {
  constructor(canvas) {
    this.canvas = canvas;
    this.joints = {};
    this.ready = false;
    this.liveCount = 0;
    this.guessCount = 0;
    this.missing = [];
    this.meshCount = 0;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(cssv("--void", "#05070c"), 1);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(42, 16 / 9, 0.02, 20);
    this.camera.position.set(1.15, 0.72, 1.05);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(0, 0.38, 0);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.scene.add(new THREE.AmbientLight(0xfff6e8, 0.55));
    const key = new THREE.DirectionalLight(0xffe6b0, 1.15);
    key.position.set(1.4, 2.2, 1.1);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0x88aacc, 0.35);
    fill.position.set(-1.2, 0.6, -0.8);
    this.scene.add(fill);
    const grid = new THREE.GridHelper(2.4, 16, cssv("--ln2", "#272b35"), cssv("--s3", "#161a22"));
    grid.position.y = 0;
    this.scene.add(grid);
    this.tilt = new THREE.Group();
    this.zup = new THREE.Group();
    this.zup.rotation.x = -Math.PI / 2;
    this.tilt.add(this.zup);
    this.scene.add(this.tilt);
    this._resize();
    this._ro = new ResizeObserver(() => this._resize());
    this._ro.observe(canvas.parentElement || canvas);
    this._loop = this._loop.bind(this);
    requestAnimationFrame(this._loop);
  }

  /* 按模型实际尺寸取景，别写死相机距离。
     写死的话换个 URDF（或者哪天线上一版尺寸变了）画面就飞了。 */
  frame() {
    const box = new THREE.Box3().setFromObject(this.zup);
    if (box.isEmpty()) return;
    const c = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    /* 竖直方向要装下 size.y，水平方向要装下 size.x/z 里大的那个，
       两个各算一个距离取更远的 —— 只按最长边算会让扁长的视口里模型缩成一点
       （第一版取 max(x,y,z) 再乘 2.6，画面里人只有指甲盖大）。 */
    const aspect = Math.max(0.3, this.camera.aspect || 1.6);
    const t = Math.tan((this.camera.fov * Math.PI) / 360);
    const dist =
      Math.max(size.y / 2 / t, Math.max(size.x, size.z) / 2 / (t * aspect)) * 1.5;
    this.controls.target.copy(c);
    this.camera.position.copy(c).add(new THREE.Vector3(0.5, 0.22, 1).normalize().multiplyScalar(dist));
    this.camera.near = Math.max(0.005, dist * 0.02);
    this.camera.far = dist * 40;
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  _resize() {
    const host = this.canvas.parentElement || this.canvas;
    const w = Math.max(2, host.clientWidth);
    const h = Math.max(2, host.clientHeight);
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  _loop() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    requestAnimationFrame(this._loop);
  }

  async load() {
    const res = await fetch(URDF, { cache: "no-store" });
    if (!res.ok) throw new Error("urdf " + res.status);
    const xml = await res.text();
    const doc = new DOMParser().parseFromString(xml, "text/xml");
    const robotEl = doc.querySelector("robot");
    if (!robotEl) throw new Error("urdf 坏了");
    const needed = collectMeshNames(robotEl);
    const { geos, missing } = await loadGlbs(needed);
    this.missing = missing;
    const { root, joints, meshCount } = buildRobot(xml, geos);
    this.zup.add(root);
    this.joints = joints;
    this.meshCount = meshCount;
    this.frame();
    this.ready = true;
    if (!meshCount) {
      throw new Error("官方 STL 没装上，URDF 已挂");
    }
  }

  setJoints(map) {
    if (!this.ready) return;
    for (const [name, j] of Object.entries(this.joints)) {
      if (j.type === "fixed") continue;
      const q = map[name];
      if (q == null || !Number.isFinite(q)) continue;
      j.moving.quaternion.setFromAxisAngle(j.axis, q);
    }
  }

  setImu(imu) {
    if (!imu) return;
    /* 世界系轴映射（2026-09-14 定案）：tilt.rotation.x 绕世界 X = 模型侧翻轴，
       tilt.rotation.z 绕世界 Z = 模型俯仰轴。所以 pitch → z、roll → x，
       之前写反了：pitch 放大在 x 上，大俯仰时孪生会侧倒（折叠帧实测头侧移 22cm）。
       符号：前倾（pitch 正）= 绕模型 Y 正 = 绕世界 Z 负；右倾（roll 正）= 绕模型 X 负。 */
    const p = THREE.MathUtils.degToRad(Number(imu.pitch) || 0) * 0.45;
    const r = THREE.MathUtils.degToRad(Number(imu.roll) || 0) * 0.45;
    this.tilt.rotation.x = -r;
    this.tilt.rotation.z = -p;
  }

  update(live) {
    const mc = live && live.mc;
    const folded = foldPose(mc) || !!(mc && mc.fallen);
    const map = guessLegs(live && live.body, mc);
    this.guessCount = Object.keys(map).length;
    this.liveCount = 0;
    for (const j of (live && live.joints) || []) {
      if (!j || !j.name) continue;
      map[j.name] = Number(j.pos);
      this.liveCount += 1;
    }
    this.setJoints(map);
    const fold = this.joints["waist_yaw_joint"] && this.joints["waist_yaw_joint"].fold;
    if (fold) fold.rotation.y = 0; // 每帧复位；折叠分支下面覆盖，站→折→站不会残留
    if (folded && mc.posture) {
      /* 折叠：上身绕模型 Y 轴向前折，用 posture.pitch 的**真实值**（实测 1.32 rad），
         0~1.4 rad 只夹掉不可能的野值。腿是估的（固件不下发腿关节），俯仰是真数。 */
      const pitch = Number(mc.posture.pitch) || 0;
      if (fold) fold.rotation.y = Math.max(0, Math.min(1.4, pitch));
      this.tilt.rotation.set(0, 0, 0);
    } else {
      this.setImu(live && live.imu);
    }
  }
}

const canvas = document.getElementById("twin");
const cap = document.getElementById("twinCap");
let lastLive = null;
const api = {
  update(live) {
    lastLive = live || lastLive;
  },
};

if (canvas) {
  const twin = new Twin(canvas);
  api.twin = twin; // 调试/测试句柄：探针拿它量头部世界坐标等
  twin
    .load()
    .then(() => {
      // 藏掉「URDF」占位符。相机那边是靠 .cam.has-frame 这个类藏的，孪生一直没加过
      const box = document.getElementById("twinBox");
      if (box) box.classList.add("has-frame");
      api.update = (live) => {
        lastLive = live || lastLive;
        twin.update(lastLive || {});
        if (!cap) return;
        const extra = twin.missing.length ? ` · 缺 ${twin.missing.length} 件` : "";
        const pose = foldPose(lastLive && lastLive.mc) ? " · 折叠" : "";
        const n = twin.liveCount;
        cap.textContent = n
          ? `真模型 ${twin.meshCount} · 真关节 ${n} · 腿无反馈估${pose}${extra}`
          : `真模型 ${twin.meshCount} · 上身还没刷到${extra}`;
      };
      if (lastLive) api.update(lastLive);
      else if (cap) cap.textContent = `真模型 ${twin.meshCount} · 等关节`;
    })
    .catch((e) => {
      if (!cap) return;
      const n = twin.missing.length;
      cap.textContent = n ? `孪生缺 ${n} 件网格` : "孪生没装上";
    });
}

window.Q1Twin = api;
