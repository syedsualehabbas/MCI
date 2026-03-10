import serial
import matplotlib.pyplot as plt
from drawnow import drawnow

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyUSB1', 115200)
sinWaveData.flushInput()

plt.ion()

adcValues      = []
filteredValues = []
time_ms        = []
cnt            = 0

def makeFig():
    plt.clf()
    plt.title('Live ADC & Filtered Data - Lab 7 Task 2')
    plt.grid(True)
    plt.xlabel('Sample Index')
    plt.ylabel('Voltage (mV)')
    plt.ylim(0, 3400)
    plt.plot(time_ms, adcValues,      'r.-', label='Raw ADC (mV)')
    plt.plot(time_ms, filteredValues, 'b.-', label='Filtered Mean (mV)')
    plt.legend(loc='upper left')

print("Waiting for data from STM32...")

while True:
    while sinWaveData.inWaiting() == 0:
        pass

    try:
        line = sinWaveData.readline().decode('utf-8', errors='replace').strip()

        # Skip empty lines
        if not line:
            continue

        # Only accept lines that look like "number,number"
        # Everything else (headers, status messages) gets skipped
        values = line.split(',')
        if len(values) != 2:
            continue

        # Try parsing - if it fails it's a text line, just skip it
        raw      = int(values[0].strip())
        filtered = int(values[1].strip())

        adcValues.append(raw)
        filteredValues.append(filtered)
        time_ms.append(cnt)
        cnt += 1

        drawnow(makeFig)
        plt.pause(0.0001)

        # Keep only last 500 samples on screen
        if len(adcValues) > 500:
            adcValues.pop(0)
            filteredValues.pop(0)
            time_ms.pop(0)

    except ValueError:
        pass   # not a data line, skip silently
    except Exception as e:
        print("Error:", e)