"""
auv_keyboard.py - keyboard teleop sender for the Simulink AUV model.

Opens a small window that captures key presses and streams the current
command to Simulink over UDP (localhost) at a fixed rate.

KEEP THIS WINDOW FOCUSED while flying. Keys only go to the focused window,
so the Simulation 3D Viewer never sees them (it binds O, L, P, Q, E, etc. to
camera actions).

Controls
    i / k   forward / backward   (surge)
    j / l   left / right         (sway)
    u / o   yaw left / right
    r / f   up / down            (heave)   <- added, delete if unwanted
    space   fire torpedo
    m       drop marker
    x       reset request (only does something if wired to a reset in Simulink)
    esc     quit

Packet: 7 little-endian doubles
    [surge, sway, heave, yaw, fire, drop, reset]   (motion values in -1..1)

Requires only the Python standard library.
"""

import socket
import struct
import tkinter as tk

# ---------------- settings ----------------
HOST = "127.0.0.1"
PORT = 25000            # must match the UDP Receive block's local port
RATE_HZ = 50            # send rate (match the Simulink block sample time, 0.02 s)
RAMP_TIME = 0.25        # seconds to go from 0 to full command (smooths key presses)

AXIS_KEYS = {           # key -> (axis index, direction)
    "i": (0, +1), "k": (0, -1),   # surge
    "l": (1, +1), "j": (1, -1),   # sway
    "r": (2, +1), "f": (2, -1),   # heave (+ = up)
    "o": (3, +1), "u": (3, -1),   # yaw
}
BUTTON_KEYS = {"space": 4, "m": 5, "x": 6}  # fire, drop, reset
# ------------------------------------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
held = set()
current = [0.0] * 4     # ramped motion commands


def on_press(event):
    key = event.keysym.lower()
    if key == "escape":
        root.destroy()
        return
    held.add(key)


def on_release(event):
    held.discard(event.keysym.lower())


def on_focus_out(_event):
    held.clear()        # fail safe: losing focus zeroes everything


def tick():
    step = 1.0 / (RATE_HZ * RAMP_TIME)

    # target for each axis from held keys
    target = [0.0] * 4
    for key, (idx, sign) in AXIS_KEYS.items():
        if key in held:
            target[idx] += sign
    target = [max(-1.0, min(1.0, t)) for t in target]

    # ramp toward target
    for i in range(4):
        diff = target[i] - current[i]
        current[i] += max(-step, min(step, diff))

    buttons = [0.0, 0.0, 0.0]
    for key, idx in BUTTON_KEYS.items():
        if key in held:
            buttons[idx - 4] = 1.0

    packet = struct.pack("<7d", *current, *buttons)
    sock.sendto(packet, (HOST, PORT))

    status.set(
        f"surge {current[0]:+.2f}   sway {current[1]:+.2f}\n"
        f"heave {current[2]:+.2f}   yaw  {current[3]:+.2f}\n"
        f"fire {int(buttons[0])}   drop {int(buttons[1])}   reset {int(buttons[2])}"
    )
    root.after(int(1000 / RATE_HZ), tick)


root = tk.Tk()
root.title("AUV keyboard control (keep focused)")
root.geometry("340x170")
status = tk.StringVar()
tk.Label(root, textvariable=status, font=("Consolas", 12), justify="left").pack(pady=10)
tk.Label(
    root,
    text="i/k fwd/back   j/l left/right   u/o yaw\nr/f up/down   space fire   m marker   esc quit",
    font=("Consolas", 9),
).pack()

root.bind("<KeyPress>", on_press)
root.bind("<KeyRelease>", on_release)
root.bind("<FocusOut>", on_focus_out)
root.focus_force()
tick()
root.mainloop()
