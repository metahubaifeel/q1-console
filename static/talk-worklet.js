/* 对讲上行的工作线程：麦克风 → 24 kHz 单声道 Int16 → 每 0.1 秒交给主线程发走。
   放工作线程是因为主线程要跑 3D 房间和仪表盘，ScriptProcessor 那种主线程回调会被拖出爆音。
   重采样按实际采样率算，不假设 48 kHz —— 设备是 44.1 kHz 时直接 ÷2 会变调。 */
const OUT_RATE = 24000;
const FRAME = 2400; // 0.1 秒

class TalkWorklet extends AudioWorkletProcessor {
  constructor() {
    super();
    this.ratio = sampleRate / OUT_RATE;
    this.pos = 0; // 在当前缓冲里的浮点读位置
    this.buf = new Float32Array(0);
    this.out = [];
  }

  process(inputs) {
    const ch = inputs[0] && inputs[0][0];
    if (!ch || !ch.length) return true;

    if (this.buf.length) {
      const merged = new Float32Array(this.buf.length + ch.length);
      merged.set(this.buf);
      merged.set(ch, this.buf.length);
      this.buf = merged;
    } else {
      this.buf = ch;
    }

    while (this.pos + 1 < this.buf.length) {
      const i = this.pos | 0;
      const f = this.pos - i;
      const v = this.buf[i] * (1 - f) + this.buf[i + 1] * f;
      const s = v < -1 ? -1 : v > 1 ? 1 : v;
      this.out.push((s * 32767) | 0);
      this.pos += this.ratio;
      if (this.out.length >= FRAME) {
        const pcm = Int16Array.from(this.out);
        this.port.postMessage(pcm, [pcm.buffer]);
        this.out.length = 0;
      }
    }

    const drop = this.pos | 0;
    if (drop > 0) {
      this.buf = this.buf.slice(drop);
      this.pos -= drop;
    }
    return true;
  }
}

registerProcessor("q1-talk", TalkWorklet);
