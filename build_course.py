#!/usr/bin/env python3
"""
build_course.py - build a MuJoCo model of the RoboSub pool with the section-D props.

    python build_course.py --meshes path/to/folder_with_STLs
    python -m mujoco.viewer --mjcf=build/course.xml

What it does
  1. Converts map pixels to metres using the calibration in course_layout.json.
  2. For every STL listed under "meshes": fixes units (mm -> m, auto-detected), converts the
     CAD up-axis (SolidWorks Y-up -> MuJoCo Z-up), optionally aligns the mesh's longest
     horizontal axis to +x, recentres it (XY bounding-box centre at the origin, bottom at z=0)
     and writes the result to <out>/meshes/.
  3. Writes <out>/course.xml: full pool (floor, walls, translucent water surface, section
     dividers), the props from "items", pinger/start markers, and two cameras.
  4. Optionally (--map map.png) draws the resulting positions back onto the map so you can
     check them by eye.

Requires: pip install mujoco trimesh numpy   (matplotlib + pillow only for --map)
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import trimesh


# ----------------------------------------------------------------------------- geometry
def px_to_m(layout, px, py):
    p, m = layout["pool"], layout["map"]
    fx = (px - m["px_left"]) / (m["px_right"] - m["px_left"])
    fy = (py - m["px_top"]) / (m["px_bottom"] - m["px_top"])
    return fx * p["length_m"] - p["length_m"] / 2, p["width_m"] / 2 - fy * p["width_m"]


def m_to_px(layout, x, y):
    p, m = layout["pool"], layout["map"]
    fx = (x + p["length_m"] / 2) / p["length_m"]
    fy = (p["width_m"] / 2 - y) / p["width_m"]
    return (m["px_left"] + fx * (m["px_right"] - m["px_left"]),
            m["px_top"] + fy * (m["px_bottom"] - m["px_top"]))


def item_xy(layout, it):
    if "xy_m" in it:
        return tuple(it["xy_m"])
    return px_to_m(layout, *it["map_px"])


def rot_z(deg):
    a = math.radians(deg)
    T = np.eye(4)
    T[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return T


# ----------------------------------------------------------------------------- meshes
def process_mesh(src, cfg, defaults, out_path, max_faces):
    """Return dict(height, extents_m, notes) and write the cleaned mesh to out_path."""
    notes = []
    mesh = trimesh.load_mesh(str(src), force="mesh")
    raw_ext = mesh.extents.copy()

    # 1. units
    units = cfg.get("units", defaults.get("units", "auto"))
    if units == "auto":
        units = "mm" if raw_ext.max() > 30 else "m"
    scale = {"mm": 0.001, "m": 1.0, "cm": 0.01, "in": 0.0254}[units]
    mesh.apply_scale(scale)
    notes.append(f"units={units}")

    # 2. up axis -> +z
    up = cfg.get("up_axis", defaults.get("up_axis", "y")).lower()
    if up == "y":                       # (x, y, z) -> (x, -z, y)
        T = np.eye(4)
        T[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
        mesh.apply_transform(T)
    elif up != "z":
        raise SystemExit(f"{src.name}: up_axis must be 'y' or 'z', got {up!r}")
    notes.append(f"up={up}")

    # 2b. optional extra rotation about the world axes (right-hand rule), applied x, then y, then z.
    #     Use it to tip a mesh upright, flip it, or tilt it, e.g. "rotate_deg": [0, 90, 0]
    rot = cfg.get("rotate_deg")
    if rot:
        if len(rot) != 3:
            raise SystemExit(f"{src.name}: rotate_deg must be [rx, ry, rz] in degrees")
        for axis, deg in zip(([1, 0, 0], [0, 1, 0], [0, 0, 1]), rot):
            if deg:
                mesh.apply_transform(trimesh.transformations.rotation_matrix(math.radians(deg), axis))
        notes.append(f"rotate_deg={list(rot)}")

    # 3. alignment of the longest horizontal axis to +x
    align = cfg.get("align", "none")
    if align == "principal":
        np.random.seed(0)
        pts = mesh.sample(20000)[:, :2]
        pts = pts - pts.mean(axis=0)
        w, v = np.linalg.eigh(np.cov(pts.T))
        ratio = math.sqrt(w[1] / max(w[0], 1e-12))
        theta = math.degrees(math.atan2(v[1, 1], v[0, 1]))
        mesh.apply_transform(rot_z(-theta))
        notes.append(f"aligned (was {theta % 180:.1f} deg, elongation {ratio:.2f})")
        if ratio < 1.2:
            notes.append("WARNING: nearly symmetric footprint, set align='none' and use yaw_deg instead")

    # 4. recentre: XY bbox centre -> 0, bottom -> z=0
    lo, hi = mesh.bounds
    mesh.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])

    # 5. optional decimation
    if len(mesh.faces) > max_faces:
        try:
            mesh = mesh.simplify_quadric_decimation(face_count=max_faces)
            notes.append(f"decimated to {len(mesh.faces)} faces")
        except Exception as exc:
            notes.append(f"WARNING: {len(mesh.faces)} faces (pip install fast-simplification to decimate): {exc}")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(str(out_path))
    return {"height": float(mesh.extents[2]), "extents": mesh.extents.copy(), "notes": notes}


# ----------------------------------------------------------------------------- xml
def fmt(vals):
    return " ".join(f"{v:.4f}".rstrip("0").rstrip(".") if abs(v) > 1e-9 else "0" for v in vals)


def yaw_quat(deg):
    a = math.radians(deg) / 2
    return (math.cos(a), 0.0, 0.0, math.sin(a))


def build_xml(layout, mesh_info):
    P = layout["pool"]
    L, W, D, T = P["length_m"], P["width_m"], P["depth_m"], P["wall_thickness_m"]
    collide = layout.get("props_collide", False)
    prop_col = "" if collide else ' contype="0" conaffinity="0"'

    # centre of the last section (for the default view / cameras)
    mp = layout["map"]
    dx_px = (mp["section_dividers_px_x"][-1] + mp["px_right"]) / 2
    cx, cy = px_to_m(layout, dx_px, (mp["px_top"] + mp["px_bottom"]) / 2)

    x = []
    x.append('<mujoco model="robosub_course">')
    x.append('  <compiler angle="degree" meshdir="meshes" autolimits="true"/>')
    x.append('  <option timestep="0.005" gravity="0 0 -9.81" density="1000" viscosity="0.001" integrator="implicitfast"/>')
    x.append(f'  <statistic center="{fmt([cx, cy, -D / 2])}" extent="{max(L, W) / 3:.1f}"/>')
    x.append('  <visual>')
    x.append('    <headlight ambient="0.55 0.55 0.55" diffuse="0.6 0.6 0.6" specular="0 0 0"/>')
    x.append('    <global azimuth="130" elevation="-28" offwidth="1920" offheight="1080"/>')
    x.append('    <map znear="0.01" zfar="300"/>')
    x.append('  </visual>')

    x.append('  <asset>')
    x.append('    <texture type="skybox" builtin="gradient" rgb1="0.55 0.75 0.95" rgb2="0.9 0.95 1" width="64" height="64"/>')
    for fname in mesh_info:
        x.append(f'    <mesh name="{Path(fname).stem}" file="{Path(fname).stem}.stl" inertia="shell"/>')
    x.append('  </asset>')

    x.append('  <worldbody>')
    x.append(f'    <light name="sun" pos="{fmt([cx, cy, 20])}" dir="0 0 -1" directional="true" diffuse="0.5 0.5 0.5" castshadow="false"/>')

    # pool shell (collides)
    floor_rgba = "0.38 0.78 0.84 1"
    wall_rgba = "0.55 0.80 0.85 1"
    x.append(f'    <geom name="pool_floor" type="box" pos="0 0 {-(D + 0.1):.4f}" size="{L / 2:.4f} {W / 2:.4f} 0.1" rgba="{floor_rgba}"/>')
    hz = (D + 0.7) / 2
    zc = 0.5 - hz
    x.append(f'    <geom name="wall_west"  type="box" pos="{-(L / 2 + T / 2):.4f} 0 {zc:.4f}" size="{T / 2:.4f} {W / 2 + T:.4f} {hz:.4f}" rgba="{wall_rgba}"/>')
    x.append(f'    <geom name="wall_east"  type="box" pos="{L / 2 + T / 2:.4f} 0 {zc:.4f}" size="{T / 2:.4f} {W / 2 + T:.4f} {hz:.4f}" rgba="{wall_rgba}"/>')
    x.append(f'    <geom name="wall_south" type="box" pos="0 {-(W / 2 + T / 2):.4f} {zc:.4f}" size="{L / 2:.4f} {T / 2:.4f} {hz:.4f}" rgba="{wall_rgba}"/>')
    x.append(f'    <geom name="wall_north" type="box" pos="0 {W / 2 + T / 2:.4f} {zc:.4f}" size="{L / 2:.4f} {T / 2:.4f} {hz:.4f}" rgba="{wall_rgba}"/>')

    # water surface (visual only)
    x.append(f'    <geom name="water_surface" type="box" pos="0 0 -0.01" size="{L / 2:.4f} {W / 2:.4f} 0.01" rgba="0.1 0.45 0.8 0.12" contype="0" conaffinity="0"/>')

    # section dividers at the surface (visual only)
    names = mp.get("section_names", [])
    for i, dpx in enumerate(mp["section_dividers_px_x"]):
        dxm, _ = px_to_m(layout, dpx, mp["px_top"])
        lab = f"{names[i]}_{names[i + 1]}" if len(names) > i + 1 else str(i)
        x.append(f'    <geom name="divider_{lab}" type="box" pos="{dxm:.4f} 0 -0.04" size="0.04 {W / 2:.4f} 0.04" rgba="0.9 0.1 0.1 1" contype="0" conaffinity="0"/>')

    # props
    for it in layout["items"]:
        fname = it["file"]
        info = mesh_info[fname]
        cfg = layout["meshes"][fname]
        px, py = item_xy(layout, it)
        anchor = cfg.get("anchor", "floor")
        if anchor == "floor":
            z = -D + cfg.get("lift_m", 0.0)
        elif anchor == "surface":
            z = -info["height"] - cfg.get("drop_m", 0.0)
        elif anchor == "z":
            z = cfg["z_m"]
        else:
            raise SystemExit(f"{it['name']}: unknown anchor {anchor!r}")
        it["_pos"] = (px, py, z)
        q = yaw_quat(it.get("yaw_deg", 0.0))
        rgba = fmt(cfg.get("rgba", [0.7, 0.7, 0.7, 1]))
        x.append(f'    <body name="{it["name"]}" pos="{fmt([px, py, z])}" quat="{fmt(q)}">')
        x.append(f'      <geom name="{it["name"]}_geom" type="mesh" mesh="{Path(fname).stem}" rgba="{rgba}"{prop_col}/>')
        x.append('    </body>')

    # markers
    for mk in layout.get("markers", []):
        px, py = item_xy(layout, mk)
        if mk["kind"] == "pinger":
            z = -D + mk.get("z_above_floor_m", 0.05)
            x.append(f'    <site name="{mk["name"]}" type="sphere" pos="{fmt([px, py, z])}" size="0.12" rgba="0.1 0.8 0.3 1"/>')
        elif mk["kind"] == "start":
            z = mk.get("z_m", -0.6)
            a = math.radians(mk.get("yaw_deg", 0.0))
            tip = [px + 0.8 * math.cos(a), py + 0.8 * math.sin(a), z]
            x.append(f'    <site name="{mk["name"]}" type="sphere" pos="{fmt([px, py, z])}" size="0.12" rgba="0.2 0.4 1 1"/>')
            x.append(f'    <site name="{mk["name"]}_heading" type="capsule" fromto="{fmt([px, py, z])} {fmt(tip)}" size="0.04" rgba="0.2 0.4 1 1"/>')
        mk["_pos"] = (px, py)

    # camera target + cameras
    x.append(f'    <body name="view_target_D" pos="{fmt([cx, cy, -D / 2])}"/>')
    x.append(f'    <camera name="top_D" pos="{fmt([cx, cy, 14])}" euler="0 0 0" fovy="60"/>')
    x.append(f'    <camera name="iso_D" pos="{fmt([cx - 9, cy - 9, 4])}" mode="targetbody" target="view_target_D" fovy="60"/>')
    x.append('  </worldbody>')
    x.append('</mujoco>')
    return "\n".join(x) + "\n"


# ----------------------------------------------------------------------------- overlay
def draw_overlay(layout, map_png, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    img = Image.open(map_png).convert("RGB")
    fig, ax = plt.subplots(figsize=(16, 8.3), dpi=110)
    ax.imshow(img)
    mp = layout["map"]
    ax.plot([mp["px_left"], mp["px_right"], mp["px_right"], mp["px_left"], mp["px_left"]],
            [mp["px_top"], mp["px_top"], mp["px_bottom"], mp["px_bottom"], mp["px_top"]], "--", color="yellow", lw=1)
    for it in layout["items"]:
        x, y = item_xy(layout, it)
        px, py = m_to_px(layout, x, y)             # metres -> pixels (round trip check)
        a = math.radians(it.get("yaw_deg", 0.0))
        ax.plot([px - 28 * math.cos(a), px + 28 * math.cos(a)], [py + 28 * math.sin(a), py - 28 * math.sin(a)], "-", color="magenta", lw=2)
        ax.plot(px, py, "o", color="magenta", ms=6, mec="white")
        ax.text(px + 8, py - 8, f'{it["name"]}\n({x:.1f}, {y:.1f}) m', color="white", fontsize=7,
                bbox=dict(boxstyle="round,pad=0.15", fc="black", alpha=0.65))
    for mk in layout.get("markers", []):
        x, y = item_xy(layout, mk)
        px, py = m_to_px(layout, x, y)
        ax.plot(px, py, "s", color="lime" if mk["kind"] == "pinger" else "deepskyblue", ms=6, mec="white")
        ax.text(px + 8, py + 14, mk["name"], color="white", fontsize=6,
                bbox=dict(boxstyle="round,pad=0.12", fc="black", alpha=0.55))
    ax.set_xlim(940, 1300)
    ax.set_ylim(745, 190)
    ax.set_title("Estimated prop positions (m, origin = pool centre) drawn back onto the map; magenta line = yaw")
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--meshes", required=True, help="folder containing the STL files named in the layout")
    ap.add_argument("--layout", default=str(Path(__file__).with_name("course_layout.json")))
    ap.add_argument("--out", default="build", help="output folder (default: ./build)")
    ap.add_argument("--max-faces", type=int, default=100000)
    ap.add_argument("--map", help="optional map image: also writes map_overlay.png into the output folder")
    args = ap.parse_args()

    layout = json.loads(Path(args.layout).read_text())
    mesh_dir, out = Path(args.meshes), Path(args.out)
    defaults = layout.get("mesh_defaults", {})

    used = sorted({it["file"] for it in layout["items"]})
    mesh_info, problems = {}, 0
    for fname in used:
        src = mesh_dir / fname
        if not src.exists():
            print(f"MISSING: {src}")
            problems += 1
            continue
        info = process_mesh(src, layout["meshes"].get(fname, {}), defaults,
                            out / "meshes" / (Path(fname).stem + ".stl"), args.max_faces)
        mesh_info[fname] = info
        e = info["extents"]
        print(f"{fname:28s} size after fixes (x,y,z) = {e[0]:.2f} x {e[1]:.2f} x {e[2]:.2f} m   [{'; '.join(info['notes'])}]")
    if problems:
        sys.exit("Fix the missing files above (or edit course_layout.json) and run again.")

    xml = build_xml(layout, mesh_info)
    out.mkdir(parents=True, exist_ok=True)
    (out / "course.xml").write_text(xml)

    print("\nplacement (world metres; z = bottom of the prop):")
    for it in layout["items"]:
        x, y, z = it["_pos"]
        print(f'  {it["name"]:10s} x={x:7.2f} y={y:6.2f} z={z:6.2f}  yaw={it.get("yaw_deg", 0):6.1f}  <- {it["file"]}')
    print(f"\nwrote {out / 'course.xml'}")
    print(f"view it:  python -m mujoco.viewer --mjcf={out / 'course.xml'}")
    if args.map:
        draw_overlay(layout, args.map, out / "map_overlay.png")
        print(f"wrote {out / 'map_overlay.png'}")


if __name__ == "__main__":
    main()
