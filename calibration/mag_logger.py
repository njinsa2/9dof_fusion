#!/usr/bin/env python3

import csv
import queue
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button


SERIAL_PORT = "/dev/ttyACM1"
BAUD_RATE = 115200       # USB CDC usually ignores this setting
SERIAL_TIMEOUT = 0.2

OUTPUT_DIR = Path("mag_data")
PLOT_INTERVAL_MS = 100
MAX_DISPLAY_POINTS = 150000


data_queue = queue.Queue(maxsize=100000)
stop_event = threading.Event()

state_lock = threading.Lock()
state = {
    "running": True,
    "connected": False,
    "total": 0,
    "invalid": 0,
    "error": "",
    "last_sample_time": None,
}

samples = []
plot_samples = []

OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

session_name = datetime.now().strftime("mag_%Y%m%d_%H%M%S")
csv_path = OUTPUT_DIR / f"{session_name}.csv"


def serial_reader():

    ser = None

    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=SERIAL_TIMEOUT,
        )

        with state_lock:
            state["connected"] = True
            state["error"] = ""

        print(f"Connected to {SERIAL_PORT}")
        print(f"Logging to {csv_path}")

        while not stop_event.is_set():

            try:
                line = ser.readline()

                if not line:
                    continue

                try:
                    text = line.decode("ascii").strip()
                    fields = text.split(",")

                    if len(fields) != 3:
                        raise ValueError("Expected exactly three values")

                    x, y, z = map(float, fields)

                    values = np.array([x, y, z], dtype=float)

                    if not np.all(np.isfinite(values)):
                        raise ValueError("Non-finite measurement")

                except (UnicodeDecodeError, ValueError):
                    with state_lock:
                        state["invalid"] += 1
                    continue

                timestamp = time.time()

                try:
                    data_queue.put(
                        (timestamp, x, y, z),
                        timeout=0.2,
                    )
                except queue.Full:
                    with state_lock:
                        state["error"] = (
                            "Data queue full; consumer is falling behind"
                        )

            except serial.SerialException as exc:
                with state_lock:
                    state["error"] = f"Serial error: {exc}"
                    state["connected"] = False
                break

    except (serial.SerialException, OSError) as exc:
        with state_lock:
            state["error"] = str(exc)
            state["connected"] = False

    finally:
        if ser is not None and ser.is_open:
            ser.close()

        with state_lock:
            state["connected"] = False


def write_csv_header():
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "unix_timestamp",
            "elapsed_seconds",
            "x_gauss",
            "y_gauss",
            "z_gauss",
        ])


write_csv_header()
session_start = time.time()


def save_sample(timestamp, x, y, z):

    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            f"{timestamp:.6f}",
            f"{timestamp - session_start:.6f}",
            f"{x:.8f}",
            f"{y:.8f}",
            f"{z:.8f}",
        ])


plt.rcParams["toolbar"] = "toolbar2"

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection="3d")

plt.subplots_adjust(bottom=0.22, right=0.78)

ax.set_title("MMC5983MA - Live Raw Magnetometer Measurements")
ax.set_xlabel("X (gauss)")
ax.set_ylabel("Y (gauss)")
ax.set_zlabel("Z (gauss)")

ax.grid(True)

scatter = ax.scatter(
    [], [], [],
    s=5,
    alpha=0.65,
    c="tab:blue",
    depthshade=False,
)

status_text = fig.text(
    0.03, 0.13, "",
    fontsize=10,
    family="monospace",
)

help_text = fig.text(
    0.03, 0.045,
    "Rotate the sensor through all orientations. "
    "Use the mouse to rotate the 3D plot.",
    fontsize=9,
)


def toggle_collection(event):
    with state_lock:
        state["running"] = not state["running"]
        running = state["running"]

    pause_button.label.set_text(
        "Pause collection" if running else "Resume collection"
    )

    fig.canvas.draw_idle()


def clear_display(event):

    plot_samples.clear()

    scatter._offsets3d = ([], [], [])

    fig.canvas.draw_idle()


def finish(event):
    stop_event.set()
    plt.close(fig)


ax_pause = plt.axes([0.78, 0.13, 0.18, 0.055])
pause_button = Button(ax_pause, "Pause collection")
pause_button.on_clicked(toggle_collection)

ax_clear = plt.axes([0.78, 0.055, 0.18, 0.055])
clear_button = Button(ax_clear, "Clear display")
clear_button.on_clicked(clear_display)


def update_plot(_frame):

    new_samples = []

    while True:
        try:
            item = data_queue.get_nowait()
        except queue.Empty:
            break

        new_samples.append(item)

    for timestamp, x, y, z in new_samples:

        with state_lock:
            running = state["running"]

        if not running:
            continue

        save_sample(timestamp, x, y, z)

        sample = (x, y, z)
        samples.append(sample)
        plot_samples.append(sample)

        with state_lock:
            state["total"] += 1
            state["last_sample_time"] = timestamp

    if len(plot_samples) > MAX_DISPLAY_POINTS:
        del plot_samples[:-MAX_DISPLAY_POINTS]

    if plot_samples:

        arr = np.asarray(plot_samples)

        scatter._offsets3d = (
            arr[:, 0],
            arr[:, 1],
            arr[:, 2],
        )

        mins = arr.min(axis=0)
        maxs = arr.max(axis=0)

        center = (mins + maxs) / 2.0
        radius = max(float(np.max(maxs - mins)) / 2.0, 1e-4)
        radius *= 1.15

        ax.set_xlim(center[0] - radius, center[0] + radius)
        ax.set_ylim(center[1] - radius, center[1] + radius)
        ax.set_zlim(center[2] - radius, center[2] + radius)

    with state_lock:
        connected = state["connected"]
        total = state["total"]
        invalid = state["invalid"]
        error = state["error"]
        running = state["running"]

    status_text.set_text(
        f"Port:       {SERIAL_PORT}\n"
        f"Connected:  {connected}\n"
        f"Collecting: {running}\n"
        f"Samples:    {total}\n"
        f"Invalid:    {invalid}\n"
        f"CSV:        {csv_path.name}\n"
        f"Status:     {error or 'OK'}"
    )

    return scatter, status_text


def main():

    reader = threading.Thread(
        target=serial_reader,
        daemon=True,
    )
    reader.start()

    animation = FuncAnimation(
        fig,
        update_plot,
        interval=PLOT_INTERVAL_MS,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    finally:
        stop_event.set()
        reader.join(timeout=1.0)

        print(f"\nCSV saved to: {csv_path}")
        print(f"Valid samples logged: {len(samples)}")


if __name__ == "__main__":
    main()
