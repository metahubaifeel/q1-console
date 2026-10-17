#!/usr/bin/env python3
"""Turn q1.urdf mesh visuals into collision primitives. Robot has no STL."""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "static/urdf/q1/q1.urdf"
DST = ROOT / "static/urdf/q1/q1_viz.urdf"

FALLBACK = {
    "left_ankle_pitch_link": ("cylinder", {"radius": "0.028", "length": "0.046"}, "0 0 -0.018", "0 1.57 0"),
    "right_ankle_pitch_link": ("cylinder", {"radius": "0.028", "length": "0.046"}, "0 0 -0.018", "0 1.57 0"),
    "left_ankle_roll_link": ("box", {"size": "0.09 0.046 0.022"}, "0.03 0 -0.014", "0 0 0"),
    "right_ankle_roll_link": ("box", {"size": "0.09 0.046 0.022"}, "0.03 0 -0.014", "0 0 0"),
    "waist_roll_link": ("cylinder", {"radius": "0.036", "length": "0.042"}, "0 0 0.02", "0 0 0"),
    "head_link": ("sphere", {"radius": "0.055"}, "0 0 0.04", "0 0 0"),
}


def primitive(geom: ET.Element) -> bool:
    return any(geom.find(t) is not None for t in ("cylinder", "box", "sphere"))


def add_visual(link: ET.Element, kind: str, attrs: dict, xyz: str, rpy: str, rgba: str) -> None:
    vis = ET.SubElement(link, "visual")
    ET.SubElement(vis, "origin", xyz=xyz, rpy=rpy)
    geom = ET.SubElement(vis, "geometry")
    ET.SubElement(geom, kind, **attrs)
    mat = ET.SubElement(vis, "material", name="")
    ET.SubElement(mat, "color", rgba=rgba)


def main() -> None:
    tree = ET.parse(SRC)
    robot = tree.getroot()
    for child in list(robot):
        if child.tag == "mujoco":
            robot.remove(child)

    for link in robot.findall("link"):
        name = link.get("name") or ""
        rgba = "0.86667 0.86667 0.8902 1"
        for vis in list(link.findall("visual")):
            color = vis.find("material/color")
            if color is not None and color.get("rgba"):
                rgba = color.get("rgba")
            link.remove(vis)

        prims = []
        for col in list(link.findall("collision")):
            geom = col.find("geometry")
            if geom is not None and primitive(geom):
                prims.append(col)
            else:
                link.remove(col)

        if prims:
            for col in prims:
                orig = col.find("origin")
                xyz = orig.get("xyz", "0 0 0") if orig is not None else "0 0 0"
                rpy = orig.get("rpy", "0 0 0") if orig is not None else "0 0 0"
                geom = col.find("geometry")
                shape = None
                for tag in ("cylinder", "box", "sphere"):
                    el = geom.find(tag) if geom is not None else None
                    if el is not None:
                        shape = (tag, dict(el.attrib))
                        break
                if shape:
                    add_visual(link, shape[0], shape[1], xyz, rpy, rgba)
        elif name in FALLBACK:
            kind, attrs, xyz, rpy = FALLBACK[name]
            add_visual(link, kind, attrs, xyz, rpy, rgba)
        else:
            add_visual(link, "sphere", {"radius": "0.02"}, "0 0 0", "0 0 0", rgba)

    ET.indent(tree, space="  ")
    DST.write_bytes(b"<?xml version='1.0' encoding='utf-8'?>\n" + ET.tostring(robot, encoding="utf-8"))
    print("wrote", DST)


if __name__ == "__main__":
    main()
