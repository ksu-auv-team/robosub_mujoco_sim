#!/usr/bin/env python3
"""
fly_auv.py - put the AUV into the RoboSub course and fly it with the keyboard.

  1. python build_course.py --meshes stl            (makes build/course.xml, as before)
  2. python fly_auv.py                              (opens the MuJoCo viewer; on a Mac use: mjpython fly_auv.py)
  3. python auv_keyboard.py                         (small key-capture window; click it and KEEP IT FOCUSED)

Why two windows: the MuJoCo viewer has its own single-letter shortcuts (i, j, k, o, u ... toggle display
options), so the keys must go to a different window. auv_keyboard.py sends the commands to this script
over UDP on localhost (port 25000).

Keys (in the auv_keyboard.py window)
  i / k    forward / back          j / l   left / right
  u / o    yaw left / right        r / f   up / down
  x        reset the sub to the start pose
  space / m   fire torpedo / drop marker (only printed for now)

What this script does
  * reads sub_config.json (mass, thruster table, drag, optional sub STL),
  * adds the sub to build/course.xml at the start_D marker, writing build/course_auv.xml,
  * works out the thruster mixing from the thruster positions/directions (least squares),
  * applies buoyancy at the centre of buoyancy; water drag/added mass come from MuJoCo's fluid model,
  * stops the thrusters if no keyboard packets arrive for failsafe_timeout_s.

Requires: pip install mujoco numpy trimesh   (trimesh only if you use a sub STL)
"""
import argparse
import json
import math
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np
import mujoco

PACKET = struct.Struct("<7d")        # surge, sway, heave, yaw, fire, drop, reset  (from auv_keyboard.py)


# ------------------------------------------------------------------------------- config / mesh
def load_config(path):
    path = Path(path)
    cfg = json.loads(path.read_text())
    cfg["_dir"] = path.resolve().parent
    return cfg


def prepare_mesh(mcfg, cfg_dir, out_path):
    """Convert the sub STL (units, up axis, rotation, centring) and write it for MuJoCo. Returns its extents."""
    import trimesh
    src = Path(mcfg["file"])
    if not src.exists():
        src = cfg_dir / mcfg["file"]
    if not src.exists():
        sys.exit(f"sub mesh not found: {mcfg['file']}")
    mesh = trimesh.load_mesh(str(src), force="mesh")
    units = mcfg.get("units", "auto")
    if units == "auto":
        units = "mm" if mesh.extents.max() > 30 else "m"
    mesh.apply_scale({"mm": 0.001, "m": 1.0, "cm": 0.01, "in": 0.0254}[units])
    if mcfg.get("up_axis", "y").lower() == "y":                      # (x, y, z) -> (x, -z, y)
        T = np.eye(4)
        T[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
        mesh.apply_transform(T)
    for axis, deg in zip(([1, 0, 0], [0, 1, 0], [0, 0, 1]), mcfg.get("rotate_deg", [0, 0, 0])):
        if deg:
            mesh.apply_transform(trimesh.transformations.rotation_matrix(math.radians(deg), axis))
    if mcfg.get("center", "bbox") == "bbox":
        lo, hi = mesh.bounds
        mesh.apply_translation(-(lo + hi) / 2)
    mesh.apply_translation(mcfg.get("offset_m", [0, 0, 0]))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(str(out_path))
    e = mesh.extents
    print(f"sub mesh: units={units}; size (x,y,z) = {e[0]:.2f} x {e[1]:.2f} x {e[2]:.2f} m "
          f"(x should be the length, z the height; nose +x)")
    return e.copy()


# ------------------------------------------------------------------------------- model assembly
def fmt(v):
    return " ".join(f"{float(x):.5g}" for x in v)


def compose(course_xml, cfg, out_xml):
    """Write out_xml = course_xml + the sub. Returns dict with the sub's hull half sizes."""
    course_xml, out_xml = Path(course_xml), Path(out_xml)
    if not course_xml.exists():
        sys.exit(f"{course_xml} not found - run build_course.py first")
    base = mujoco.MjModel.from_xml_path(str(course_xml))

    # start pose
    st = cfg.get("start", {})
    if st.get("pos_m") is not None:
        pos = np.array(st["pos_m"], float)
        yaw = math.radians(st.get("yaw_deg") or 0.0)
    else:
        mk = st.get("marker", "start_D")
        try:
            sp = base.site(mk).pos.copy()
            hp = base.site(mk + "_heading").pos.copy()
        except KeyError:
            sys.exit(f"start marker '{mk}' not in {course_xml}; set start.pos_m / start.yaw_deg in the config")
        pos = sp
        yaw = math.atan2(hp[1] - sp[1], hp[0] - sp[0])
    quat = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

    # sub mesh (optional)
    mcfg = cfg.get("mesh", {})
    mesh_ext = None
    asset = ""
    if mcfg.get("file"):
        mesh_ext = prepare_mesh(mcfg, cfg["_dir"], course_xml.parent / "meshes" / "auv.stl")
        asset = '    <mesh name="auv_mesh" file="auv.stl" inertia="shell"/>\n'

    hull_half = cfg["hull"].get("half_sizes_m")
    if hull_half is None:
        hull_half = (mesh_ext / 2 * 0.9) if mesh_ext is not None else [0.45, 0.22, 0.15]
    a, b, c = [float(x) for x in hull_half]
    mass = float(cfg["mass_kg"])
    inertia = cfg.get("inertia_diag_kgm2") or [mass / 5 * (b * b + c * c), mass / 5 * (a * a + c * c), mass / 5 * (a * a + b * b)]

    rgba = cfg["mesh"].get("rgba", [0.2, 0.5, 0.9, 1.0])
    lines = []
    lines.append(f'    <body name="auv" pos="{fmt(pos)}" quat="{fmt(quat)}">')
    lines.append('      <freejoint name="auv_free"/>')
    lines.append(f'      <inertial pos="0 0 0" mass="{mass:g}" diaginertia="{fmt(inertia)}"/>')
    if mesh_ext is not None:
        lines.append(f'      <geom name="auv_visual" type="mesh" mesh="auv_mesh" rgba="{fmt(rgba)}" contype="0" conaffinity="0"/>')
        hull_rgba = "0 0 0 0"
    else:
        hull_rgba = fmt(rgba[:3] + [0.9])
    lines.append(f'      <geom name="auv_hull" type="ellipsoid" size="{fmt([a, b, c])}" rgba="{hull_rgba}" '
                 f'fluidshape="ellipsoid" fluidcoef="{fmt(cfg["hull"]["fluid_coefficients"])}"/>')
    tmax = float(cfg["thruster"]["max_thrust_N"])
    acts = []
    for t in cfg["thrusters"]:
        d = np.array(t["dir"], float)
        d /= np.linalg.norm(d)
        lines.append(f'      <site name="{t["name"]}" type="cylinder" pos="{fmt(t["pos_m"])}" zaxis="{fmt(d)}" '
                     f'size="0.03 0.05" rgba="1 0.55 0 1"/>')
        acts.append(f'    <motor name="{t["name"]}_motor" site="{t["name"]}" gear="0 0 1 0 0 0" ctrlrange="{-tmax:g} {tmax:g}"/>')
    lines.append('    </body>')

    text = course_xml.read_text()
    for needle in ("  </worldbody>", "</mujoco>"):
        if needle not in text:
            sys.exit(f"{course_xml} does not look like the file build_course.py writes ({needle!r} missing)")
    if asset:
        if "  </asset>" not in text:
            sys.exit("no <asset> block in course.xml")
        text = text.replace("  </asset>", asset + "  </asset>", 1)
    text = text.replace("  </worldbody>", "\n".join(lines) + "\n  </worldbody>", 1)
    text = text.replace("</mujoco>", "  <actuator>\n" + "\n".join(acts) + "\n  </actuator>\n</mujoco>", 1)
    out_xml.write_text(text)
    return {"hull_half": (a, b, c), "start_pos": pos, "start_yaw_deg": math.degrees(yaw)}


# ------------------------------------------------------------------------------- vehicle model
class AUV:
    def __init__(self, m, cfg, hull_half):
        self.bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "auv")
        names = [t["name"] for t in cfg["thrusters"]]
        self.act = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n + "_motor") for n in names])
        P = np.array([t["pos_m"] for t in cfg["thrusters"]], float)
        D = np.array([t["dir"] for t in cfg["thrusters"]], float)
        D /= np.linalg.norm(D, axis=1)[:, None]
        self.B = np.vstack([D.T, np.cross(P, D).T])          # wrench produced by 1 N on each thruster (6 x n)
        self.pinv = np.linalg.pinv(self.B)
        self.tmax = float(cfg["thruster"]["max_thrust_N"])
        self.tau = float(cfg["thruster"]["time_constant_s"])
        self.scale = cfg["command_scale"]
        self.weight = float(cfg["mass_kg"]) * 9.81
        self.buoy = self.weight * float(cfg["buoyancy_ratio"])
        self.cb = np.array(cfg["cb_offset_m"], float)
        self.hh = float(hull_half[2])
        self.thrust = np.zeros(len(names))
        rank = np.linalg.matrix_rank(self.B)
        if rank < 6:
            print(f"note: thruster layout controls {rank} of 6 degrees of freedom (fine if intended)")

    def allocate(self, cmd):
        """cmd = (surge, sway, heave, yaw) in [-1, 1]; sway>0 = right, yaw>0 = turn right (keyboard convention)."""
        s = self.scale
        wrench = np.array([cmd[0] * s["surge_N"], -cmd[1] * s["sway_N"], cmd[2] * s["heave_N"],
                           0.0, 0.0, -cmd[3] * s["yaw_Nm"]])
        t = self.pinv @ wrench
        peak = np.max(np.abs(t))
        if peak > self.tmax:
            t *= self.tmax / peak
        return t

    def step_controls(self, m, d, cmd):
        target = self.allocate(cmd)
        alpha = m.opt.timestep / (self.tau + m.opt.timestep)
        self.thrust += alpha * (target - self.thrust)
        d.ctrl[self.act] = self.thrust
        # buoyancy at the centre of buoyancy, reduced as the hull leaves the water
        d.qfrc_applied[:] = 0
        z = d.xpos[self.bid][2]
        frac = min(1.0, max(0.0, (self.hh - z) / (2 * self.hh)))
        if frac > 0:
            R = d.xmat[self.bid].reshape(3, 3)
            point = d.xpos[self.bid] + R @ self.cb
            mujoco.mj_applyFT(m, d, np.array([0, 0, self.buoy * frac]), np.zeros(3), point, self.bid, d.qfrc_applied)

    def reset(self, m, d):
        mujoco.mj_resetData(m, d)
        self.thrust[:] = 0
        mujoco.mj_forward(m, d)

    def pose(self, d):
        x, y, z = d.xpos[self.bid]
        w, qx, qy, qz = d.xquat[self.bid]
        yaw = math.degrees(math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
        speed = float(np.linalg.norm(d.qvel[:3]))
        return x, y, z, yaw, speed


# ------------------------------------------------------------------------------- input + loop
class UdpSource:
    def __init__(self, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(("127.0.0.1", port))
        except OSError as exc:
            sys.exit(f"cannot listen on UDP port {port} ({exc}). Another copy of fly_auv.py (or the Simulink model) may be using it.")
        self.sock.setblocking(False)

    def read(self, _t):
        """Newest packet since the last call as a 7-vector, or None."""
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
            except (BlockingIOError, ConnectionResetError):
                break
            if len(data) == PACKET.size:
                latest = np.array(PACKET.unpack(data))
        return latest


def run(m, d, auv, source, failsafe_s, seconds=None, viewer=None, realtime=True, quiet=False, log=None):
    dt = m.opt.timestep
    spf = max(1, round((1 / 60) / dt))                      # physics steps per displayed frame
    cmd = np.zeros(7)
    last_rx = -1e9
    prev = np.zeros(3)                                       # previous fire / drop / reset flags
    next_frame = time.perf_counter()
    next_print = 0.0
    while True:
        if viewer is not None and not viewer.is_running():
            break
        if seconds is not None and d.time >= seconds:
            break
        pkt = source.read(d.time)
        if pkt is not None:
            cmd, last_rx = pkt, d.time
        live = (d.time - last_rx) < failsafe_s
        c = cmd if live else np.zeros(7)

        flags = (c[4:7] > 0.5).astype(float)
        if flags[0] and not prev[0]:
            print("\n[fire torpedo pressed - not simulated yet]")
        if flags[1] and not prev[1]:
            print("\n[drop marker pressed - not simulated yet]")
        if flags[2] and not prev[2]:
            if viewer is not None:
                with viewer.lock():
                    auv.reset(m, d)
            else:
                auv.reset(m, d)
            print("\n[reset to start pose]")
        prev = flags

        for _ in range(spf):
            auv.step_controls(m, d, c[:4])
            mujoco.mj_step(m, d)
        if log is not None:
            log.append((d.time, *auv.pose(d)))
        if viewer is not None:
            viewer.sync()
        if realtime:
            next_frame += spf * dt
            delay = next_frame - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.25:
                next_frame = time.perf_counter()
        if not quiet and d.time >= next_print:
            x, y, z, yaw, sp = auv.pose(d)
            sys.stdout.write(f"\rt={d.time:6.1f}s  x={x:6.2f} y={y:6.2f} depth={-z:4.2f} m  heading={yaw:6.1f} deg  speed={sp:4.2f} m/s  "
                             f"{'' if live else '[no keyboard input]'}   ")
            sys.stdout.flush()
            next_print = d.time + 0.25


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default="build/course.xml")
    ap.add_argument("--config", default=str(Path(__file__).with_name("sub_config.json")))
    ap.add_argument("--port", type=int, default=25000)
    ap.add_argument("--free-cam", action="store_true", help="do not make the camera follow the sub")
    ap.add_argument("--headless", action="store_true", help="no viewer (for tests); needs --seconds")
    ap.add_argument("--seconds", type=float, help="stop after this much simulated time")
    args = ap.parse_args()

    cfg = load_config(args.config)
    out_xml = Path(args.course).with_name("course_auv.xml")
    info = compose(args.course, cfg, out_xml)
    m = mujoco.MjModel.from_xml_path(str(out_xml))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    auv = AUV(m, cfg, info["hull_half"])
    source = UdpSource(args.port)
    sx, sy, sz = info["start_pos"]
    print(f"sub starts at x={sx:.1f} y={sy:.1f} depth={-sz:.2f} m, heading {info['start_yaw_deg']:.0f} deg. "
          f"Listening for keyboard packets on UDP port {args.port}.")

    if args.headless:
        run(m, d, auv, source, cfg["failsafe_timeout_s"], seconds=args.seconds or 10, realtime=True)
        print()
        return
    import mujoco.viewer as mjv
    with mjv.launch_passive(m, d) as viewer:
        if not args.free_cam:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = auv.bid
            viewer.cam.distance = 5.0
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
        run(m, d, auv, source, cfg["failsafe_timeout_s"], seconds=args.seconds, viewer=viewer)
    print()


if __name__ == "__main__":
    main()
