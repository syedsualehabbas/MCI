import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ============================================================
# CONFIGURATION — change COM port to match your system
# Windows example : 'COM3'
# Linux example   : '/dev/ttyUSB0' or '/dev/ttyACM0'
# macOS example   : '/dev/cu.usbmodem1234'
# ============================================================
SERIAL_PORT = '/dev/ttyUSB0'   # <-- CHANGE THIS to your actual port
BAUD_RATE   = 115200            # Must match huart2 baud rate in STM32 code
NUM_SAMPLES = 100               # How many samples to show on screen at once

# ============================================================
# Setup
# ============================================================
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

time_axis = []   # x-axis sample counter
temp_data = []   # temperature values
x_data    = []   # X angular velocity (dps)
y_data    = []   # Y angular velocity (dps)
z_data    = []   # Z angular velocity (dps)
counter   = [0]  # mutable counter (used inside nested function)

fig, (ax_temp, ax_gyro) = plt.subplots(2, 1, figsize=(10, 7))
fig.suptitle('I3G4250D Gyroscope — Live Data', fontsize=14)

# Temperature subplot
line_temp, = ax_temp.plot([], [], 'r.-', label='Temperature')
ax_temp.set_title('Temperature (Live)')
ax_temp.set_xlabel('Sample Number')
ax_temp.set_ylabel('Temperature (relative, degree C)')
ax_temp.set_ylim(-50, 50)
ax_temp.grid(True)
ax_temp.legend(loc='upper left')

# Angular velocity subplot
line_x, = ax_gyro.plot([], [], 'r.-', label='X (dps)')
line_y, = ax_gyro.plot([], [], 'g.-', label='Y (dps)')
line_z, = ax_gyro.plot([], [], 'b.-', label='Z (dps)')
ax_gyro.set_title('Angular Velocity (Live)')
ax_gyro.set_xlabel('Sample Number')
ax_gyro.set_ylabel('Angular Velocity (dps)')
ax_gyro.set_ylim(-250, 250)
ax_gyro.grid(True)
ax_gyro.legend(loc='upper left')

# ============================================================
# Animation update function — called repeatedly by FuncAnimation
# ============================================================
def update(frame):
    """Read one line from UART, parse all 4 fields, update plots."""
    try:
        if ser.in_waiting > 0:
            raw = ser.readline().decode('utf-8', errors='ignore').strip()
            if not raw:
                return line_temp, line_x, line_y, line_z

            # Parse — Task 3 sends: "<temp>, <x_dps>, <y_dps>, <z_dps>"
            # Example: "21, 0.23, -1.74, 3.14"
            parts = raw.split(',')
            if len(parts) != 4:
                return line_temp, line_x, line_y, line_z

            temp_val = int(parts[0].strip())
            x_val    = float(parts[1].strip())
            y_val    = float(parts[2].strip())
            z_val    = float(parts[3].strip())

            # Append data
            time_axis.append(counter[0])
            temp_data.append(temp_val)
            x_data.append(x_val)
            y_data.append(y_val)
            z_data.append(z_val)
            counter[0] += 1

            # Keep only last NUM_SAMPLES points
            if len(time_axis) > NUM_SAMPLES:
                time_axis.pop(0)
                temp_data.pop(0)
                x_data.pop(0)
                y_data.pop(0)
                z_data.pop(0)

            # ── Update temperature plot ───────────────────────────────────
            line_temp.set_data(time_axis, temp_data)
            ax_temp.set_xlim(
                time_axis[0] if time_axis else 0,
                time_axis[-1] + 1 if time_axis else NUM_SAMPLES
            )
            if temp_data:
                margin = 5
                ax_temp.set_ylim(min(temp_data) - margin, max(temp_data) + margin)

            # ── Update angular velocity plot ──────────────────────────────
            line_x.set_data(time_axis, x_data)
            line_y.set_data(time_axis, y_data)
            line_z.set_data(time_axis, z_data)
            ax_gyro.set_xlim(
                time_axis[0] if time_axis else 0,
                time_axis[-1] + 1 if time_axis else NUM_SAMPLES
            )
            if x_data and y_data and z_data:
                all_gyro = x_data + y_data + z_data
                margin = max(10.0, (max(all_gyro) - min(all_gyro)) * 0.1)
                ax_gyro.set_ylim(min(all_gyro) - margin, max(all_gyro) + margin)

    except ValueError:
        print(f"[WARN] Could not parse line: '{raw}'")
    except Exception as e:
        print(f"[ERROR] {e}")

    return line_temp, line_x, line_y, line_z

# ============================================================
# Run animation  (interval = ms between frames)
# ============================================================
ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
plt.tight_layout()
plt.show()

# Close serial port when window is closed
ser.close()