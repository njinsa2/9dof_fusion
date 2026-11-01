import time
import serial
import numpy as np
import allantools
import matplotlib.pyplot as plt

PORT = '/dev/ttyACM1'          # Update to your STM32 CDC port (e.g., '/dev/ttyACM0' on Linux)
BAUD = 115200
SAMPLE_RATE = 208.0    # Fixed sensor loop rate (Hz)
DURATION_HOURS = 4.0    # Log duration
TARGET_SAMPLES = int(DURATION_HOURS * 3600 * SAMPLE_RATE)
OUTPUT_TXT = "gyro_4hr_stationary.txt"
OUTPUT_PLOT = "allan_variance_analysis.png"

print(f"[1/3] Connecting to {PORT}... Ensure board is completely motionless.")
ser = serial.Serial(PORT, BAUD, timeout=2.0)
time.sleep(2)

samples_collected = 0
start_time = time.time()

print(f"Logging {TARGET_SAMPLES:,} samples (~{DURATION_HOURS} hrs at {SAMPLE_RATE} Hz)...")

with open(OUTPUT_TXT, "w") as f:
    try:
        while samples_collected < TARGET_SAMPLES:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            
            parts = line.split(',')
            if len(parts) == 3:
                try:
                    gx, gy, gz = float(parts[0]), float(parts[1]), float(parts[2])
                    f.write(f"{gx},{gy},{gz}\n")
                    samples_collected += 1
                    
                    if samples_collected % 100000 == 0:
                        elapsed_min = (time.time() - start_time) / 60.0
                        pct = (samples_collected / TARGET_SAMPLES) * 100
                        print(f" Progress: {samples_collected:,} / {TARGET_SAMPLES:,} ({pct:.1f}%) | Elapsed: {elapsed_min:.1f} min")
                except ValueError:
                    continue
    except KeyboardInterrupt:
        print("\n[!] Logging halted early by user. Processing collected data...")

ser.close()
print(f"Data collection complete. Raw readings saved to '{OUTPUT_TXT}'.\n")

print("[2/3] Processing Allan Variance statistics across X, Y, and Z axes...")
data = np.loadtxt(OUTPUT_TXT, delimiter=',')

axes = ['Gyro X', 'Gyro Y', 'Gyro Z']
colors = ['r', 'g', 'b']
kp_default = 2.0  # Reference proportional gain

results = {}

plt.figure(figsize=(12, 7))

for i, axis_name in enumerate(axes):
    axis_data = data[:, i]
    
    (taus, adev, errors, ns) = allantools.oadev(axis_data, rate=SAMPLE_RATE, data_type="freq")
    
    idx_tau_1 = np.argmin(np.abs(taus - 1.0))
    N_g = adev[idx_tau_1]
    
    idx_min = np.argmin(adev)
    B_g = adev[idx_min]
    tau_b = taus[idx_min]
    
    ki_optimal = kp_default / tau_b
    
    results[axis_name] = {
        'N_g': N_g,
        'B_g': B_g,
        'tau_b': tau_b,
        'Ki': ki_optimal
    }
    
    plt.loglog(taus, adev, color=colors[i], label=f"{axis_name} (\u03c4_min={tau_b:.1f}s)", linewidth=1.8)
    plt.loglog(1.0, N_g, color=colors[i], marker='o', markersize=6)
    plt.loglog(tau_b, B_g, color=colors[i], marker='s', markersize=6)

plt.grid(True, which="both", ls="--", alpha=0.6)
plt.title("IEEE-952 Allan Deviation Noise Analysis (ISM330DHCX Gyroscope)", fontsize=13, fontweight='bold')
plt.xlabel("Averaging Time \u03c4 [seconds]", fontsize=11)
plt.ylabel("Allan Deviation \u03c3(\u03c4) [rad/s]", fontsize=11)
plt.legend(loc='upper right', fontsize=10)
plt.tight_layout()

plt.savefig(OUTPUT_PLOT, dpi=300)
print(f"Plot figure successfully saved to '{OUTPUT_PLOT}'.\n")

print("=" * 72)
print("              IEEE-952 ALLAN VARIANCE CHARACTERIZATION RESULTS         ")
print("=" * 72)
print(f"{'Axis':<10} | {'ARW N_g [rad/s/\u221aHz]':<22} | {'Bias Inst. B_g [rad/s]':<22} | {'\u03c4_b [s]':<8}")
print("-" * 72)

for axis_name in axes:
    r = results[axis_name]
    print(f"{axis_name:<10} | {r['N_g']:<22.6e} | {r['B_g']:<22.6e} | {r['tau_b']:<8.1f}")

print("-" * 72)

avg_tau_b = np.mean([results[a]['tau_b'] for a in axes])
recommended_ki = kp_default / avg_tau_b

print(f"\n[SUMMARY FOR PORTFOLIO / FIRMWARE]")
print(f" * Average Bias Drift Time Horizon (\u03c4_b): {avg_tau_b:.2f} seconds")
print(f" * Presumed Proportional Gain (Kp)   : {kp_default:.2f}")
print(f" * Derived Analytical Integral Gain (Ki) : {recommended_ki:.6f}")
print("=" * 72)
