"""Local face book: keep unknown faces, name them later."""
from __future__ import annotations

import base64
import json
import re
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DIR = ROOT / ".cache" / "people"
BOOK = DIR / "book.json"


def _safe(pid: str) -> str:
    s = re.sub(r"[^A-Za-z0-9._-]+", "_", pid or "face").strip("_")
    return (s or "face")[:80]


def _lock_path() -> Path:
    DIR.mkdir(parents=True, exist_ok=True)
    return DIR / ".book.lock"


def _with_lock(fn):
    lock = _lock_path()
    with open(lock, "a+", encoding="utf-8") as fh:
        try:
            import fcntl

            fcntl.flock(fh.fileno(), fcntl.LOCK_EX)
        except OSError:
            pass
        try:
            return fn()
        finally:
            try:
                import fcntl

                fcntl.flock(fh.fileno(), fcntl.LOCK_UN)
            except OSError:
                pass


def _parse_book(raw: str) -> list[dict]:
    raw = (raw or "").strip()
    if not raw:
        return []
    try:
        rows = json.loads(raw)
        return rows if isinstance(rows, list) else []
    except json.JSONDecodeError:
        pass
    rows = []
    for m in re.finditer(r"\{[^{}]*\}", raw, re.S):
        try:
            obj = json.loads(m.group(0))
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict) and obj.get("id"):
            rows.append(obj)
    by = {}
    for r in rows:
        by[r["id"]] = r
    return list(by.values())


def load() -> list[dict]:
    if not BOOK.exists():
        return []
    try:
        return _parse_book(BOOK.read_text(encoding="utf-8"))
    except OSError:
        return []


def _save_unlocked(rows: list[dict]) -> None:
    DIR.mkdir(parents=True, exist_ok=True)
    BOOK.write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")


def save(rows: list[dict]) -> None:
    _with_lock(lambda: _save_unlocked(rows))


def names_by_id() -> dict[str, str]:
    return {r["id"]: (r.get("name") or "") for r in load() if r.get("id")}


def upsert(pid: str, name: str | None = None, image_b64: str = "", box: dict | None = None) -> dict:
    def _do() -> dict:
        rows = load()
        now = time.time()
        row = next((r for r in rows if r.get("id") == pid), None)
        if row is None:
            row = {"id": pid, "name": "", "kept": now, "seen": now}
            rows.append(row)
        row["seen"] = now
        if name is not None:
            row["name"] = name.strip()
        if box:
            row["box"] = box
        if image_b64:
            raw = image_b64.split(",", 1)[-1]
            jpg = DIR / f"{_safe(pid)}.jpg"
            jpg.write_bytes(base64.b64decode(raw))
            row["thumb"] = jpg.name
        _save_unlocked(rows)
        return row

    return _with_lock(_do)


def delete(pid: str) -> bool:
    def _do() -> bool:
        rows = load()
        keep = [r for r in rows if r.get("id") != pid]
        if len(keep) == len(rows):
            return False
        _save_unlocked(keep)
        p = DIR / f"{_safe(pid)}.jpg"
        if p.exists():
            p.unlink()
        return True

    return _with_lock(_do)


def thumb_path(pid: str) -> Path | None:
    p = DIR / f"{_safe(pid)}.jpg"
    return p if p.exists() else None


def _jpg_mean(path: Path) -> float:
    try:
        from PIL import Image

        im = Image.open(path).convert("L").resize((12, 12))
        px = list(im.getdata())
        return sum(px) / max(1, len(px))
    except Exception:
        return 0.0


def sweep_junk() -> dict:
    """Drop unnamed dark/tiny crops and rewrite a valid book."""

    def _do() -> dict:
        rows = load()
        by = {r.get("id"): r for r in rows if r.get("id")}
        kept = []
        dropped = 0
        for jpg in DIR.glob("*.jpg"):
            pid = jpg.stem
            row = by.get(pid) or {"id": pid, "name": "", "kept": jpg.stat().st_mtime, "seen": jpg.stat().st_mtime}
            named = bool((row.get("name") or "").strip())
            mean = _jpg_mean(jpg)
            tiny = jpg.stat().st_size < 500
            if not named:
                jpg.unlink(missing_ok=True)
                dropped += 1
                continue
            if tiny or mean < 22:
                jpg.unlink(missing_ok=True)
                dropped += 1
                continue
            row["thumb"] = jpg.name
            kept.append(row)
        _save_unlocked(kept)
        return {"kept": len(kept), "dropped": dropped}

    return _with_lock(_do)


def attach(faces: list[dict]) -> list[dict]:
    names = names_by_id()
    out = []
    for f in faces:
        item = dict(f)
        fid = str(item.get("id") or "")
        ours = names.get(fid) or ""
        item["book_name"] = ours
        item["label"] = ours or item.get("nickname") or item.get("name") or ("生人" if fid else "脸")
        item["known"] = bool(ours)
        out.append(item)
    return out
