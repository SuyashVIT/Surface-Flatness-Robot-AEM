import serial
import time
import csv
import matplotlib.pyplot as plt
from datetime import datetime

# ====== CONFIGURE THIS ======
PORT = 'COM3'   # <-- CHANGE THIS to your Arduino COM port
BAUD = 115200
# ============================

ser = serial.Serial(PORT, BAUD, timeout=1)

# Create log file with timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
filename = f"flatness_log_{timestamp}.csv"

print(f"Logging to {filename}")

# Open CSV file
csv_file = open(filename, 'w', newline='')
csv_writer = None  # will create after reading header

# For plotting
times = []
FIs = []

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], '-o', markersize=2)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Flatness Index (FI)")
ax.set_title("Real-time Flatness Index")

start_time = time.time()

try:
    # Read lines from serial
    while True:
        line_bytes = ser.readline()
        if not line_bytes:
            continue

        try:
            line_str = line_bytes.decode('utf-8').strip()
        except UnicodeDecodeError:
            continue

        if not line_str:
            continue

        # Print for debugging
        # print(line_str)

        # CSV header from Arduino
        if line_str.startswith("time_ms"):
            header = line_str.split(',')
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(header)
            csv_file.flush()
            print("Header received:", header)
            continue

        # Regular data line
        parts = line_str.split(',')
        if csv_writer is None:
            # no header yet, skip
            continue

        if len(parts) != 4:
            # expecting 4 columns: time_ms, pitch_deg, height_cm, FI
            continue

        # Write raw line to CSV
        csv_writer.writerow(parts)
        csv_file.flush()

        # Parse for graph
        try:
            time_ms = float(parts[0])
            FI = float(parts[3])
        except ValueError:
            continue

        t_sec = time_ms / 1000.0
        times.append(t_sec)
        FIs.append(FI)

        # Update plot every few points
        if len(times) % 3 == 0:
            line.set_xdata(times)
            line.set_ydata(FIs)
            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)

except KeyboardInterrupt:
    print("\nStopping logging...")

finally:
    ser.close()
    csv_file.close()
    # Save final graph
    graph_name = f"flatness_graph_{timestamp}.png"
    plt.ioff()
    plt.savefig(graph_name)
    print(f"Graph saved as {graph_name}")
    plt.show()
