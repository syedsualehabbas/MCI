"""
Lab 7 Task 2 - Real-Time Signal Plotter
Reads raw_mV,filtered_mV lines from STM32 UART and plots live.

Install: pip install pyserial matplotlib
Run:     python3 plot_lab7.py
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ── Change this to your port ──────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyUSB1"
BAUD_RATE   = 115200
WINDOW_SIZE = 200
# ─────────────────────────────────────────────────────────────────────────────

raw_data      = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)
filtered_data = deque([0] * WINDOW_SIZE, maxlen=WINDOW_SIZE)

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.flushInput()
    print(f"[INFO] Opened {SERIAL_PORT} @ {BAUD_RATE} baud")
except serial.SerialException as e:
    print(f"[ERROR] Cannot open {SERIAL_PORT}: {e}")
    raise SystemExit(1)

fig, ax = plt.subplots(figsize=(12, 5))
fig.patch.set_facecolor("#1e1e2e")
ax.set_facecolor("#1e1e2e")

line_raw,      = ax.plot([], [], color="#89b4fa", linewidth=1.2, label="Raw ADC (mV)")
line_filtered, = ax.plot([], [], color="#a6e3a1", linewidth=2.0, label="Filtered (mV)")

ax.set_xlim(0, WINDOW_SIZE - 1)
ax.set_ylim(0, 3400)
ax.set_xlabel("Sample", color="#cdd6f4")
ax.set_ylabel("Voltage (mV)", color="#cdd6f4")
ax.set_title("Lab 7 Task 2 - Moving Average Low-Pass Filter", color="#cdd6f4", fontsize=13)
ax.tick_params(colors="#cdd6f4")
for spine in ax.spines.values():
    spine.set_edgecolor("#45475a")
ax.legend(facecolor="#313244", labelcolor="#cdd6f4", loc="upper right")
ax.grid(True, color="#313244", linewidth=0.5)

x_data = list(range(WINDOW_SIZE))

def update(_frame):
    # Read ALL available lines in one go
    for _ in range(50):                  # max 50 lines per frame to stay responsive
        if not ser.in_waiting:
            break
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
        except Exception:
            continue

        if not line or line.startswith("#"):
            continue

        parts = line.split(",")
        if len(parts) != 2:
            continue

        try:
            raw_data.append(int(parts[0]))
            filtered_data.append(int(parts[1]))
        except ValueError:
            continue

    line_raw.set_data(x_data, list(raw_data))
    line_filtered.set_data(x_data, list(filtered_data))
    return line_raw, line_filtered


ani = animation.FuncAnimation(fig, update, interval=50,
                              blit=True, cache_frame_data=False)
plt.tight_layout()

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("[INFO] Done.")