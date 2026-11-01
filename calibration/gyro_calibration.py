import serial
import time
import numpy as np

SERIAL_PORT = "/dev/ttyACM1"  # Linux ACM port
BAUD_RATE = 115200
SAMPLES_TO_COLLECT = 2000      # ~5 seconds of data at 100 Hz

def collect_gyro_samples(ser, count):
    samples = []
    ser.reset_input_buffer()
    
    print(f"Collecting {count} static samples... DO NOT touch or move the sensor!")

    while len(samples) < count:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split(',')
                if len(parts) >= 3:
                    values = [float(p) for p in parts]
                    gx, gy, gz = values[-3:] if len(values) >= 6 else values[:3]
                    samples.append([gx, gy, gz])
        except (ValueError, UnicodeDecodeError):
            continue

    return np.array(samples)

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2)
    except serial.SerialException as e:
        print(f"Error opening port {SERIAL_PORT}: {e}")
        print("Ensure your device is plugged in and permissions are granted: sudo chmod a+rw /dev/ttyACM1")
        return

    print("\n--- GYROSCOPE ZERO-RATE BIAS CALIBRATION ---")
    input("Place the sensor flat on a motionless table and press ENTER to start...")

    samples = collect_gyro_samples(ser, SAMPLES_TO_COLLECT)
    ser.close()

    bias = np.mean(samples, axis=0)
    std_dev = np.std(samples, axis=0)

    print("\n================ GYRO CALIBRATION RESULTS ================")
    print(f"Units: dps (degrees per second)")
    print(f"Bias X (b_gx): {bias[0]:.6f} dps  (Noise StdDev: {std_dev[0]:.6f})")
    print(f"Bias Y (b_gy): {bias[1]:.6f} dps  (Noise StdDev: {std_dev[1]:.6f})")
    print(f"Bias Z (b_gz): {bias[2]:.6f} dps  (Noise StdDev: {std_dev[2]:.6f})")
    print("---------------------------------------------------------")
    print("C++ Header Format (Copy & Paste):")
    print(f"const float gyro_bias[3] = {{{bias[0]:.6f}f, {bias[1]:.6f}f, {bias[2]:.6f}f}};")
    print("=========================================================\n")

if __name__ == "__main__":
    main()
