import serial
import time
import numpy as np

SERIAL_PORT = "/dev/ttyACM1"  # On Linux/Mac: '/dev/ttyUSB0' or '/dev/ttyACM0'
BAUD_RATE = 115200
SAMPLES_PER_POS = 200  # Number of samples to average at each position
GRAVITY = 1.0         # Set to 1.0 if working in g-units instead of m/s^2

positions = [
    ("+Z pointing straight UP", "z_max"),
    ("-Z pointing straight UP (Upside Down)", "z_min"),
    ("+Y pointing straight UP", "y_max"),
    ("-Y pointing straight UP", "y_min"),
    ("+X pointing straight UP", "x_max"),
    ("-X pointing straight UP", "x_min"),
]

def read_samples(ser, count):
    samples = []
    ser.reset_input_buffer()
    
    while len(samples) < count:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split(',')
                if len(parts) == 3:
                    ax, ay, az = map(float, parts)
                    samples.append([ax, ay, az])
        except (ValueError, UnicodeDecodeError):
            continue
            
    return np.mean(samples, axis=0)

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    time.sleep(2)
    
    data = {}

    print("\n--- ACCELEROMETER CALIBRATION START ---")
    print("Place the sensor on a flat, stable surface for each step.\n")

    for prompt, key in positions:
        input(f"Press ENTER when sensor is resting with {prompt}...")
        print("Collecting static measurements...")
        avg_reading = read_samples(ser, SAMPLES_PER_POS)
        data[key] = avg_reading
        print(f"Recorded average [X, Y, Z]: {avg_reading}\n")

    ser.close()

    bias_x = (data['x_max'][0] + data['x_min'][0]) / 2.0
    bias_y = (data['y_max'][1] + data['y_min'][1]) / 2.0
    bias_z = (data['z_max'][2] + data['z_min'][2]) / 2.0

    scale_x = (2.0 * GRAVITY) / (data['x_max'][0] - data['x_min'][0])
    scale_y = (2.0 * GRAVITY) / (data['y_max'][1] - data['y_min'][1])
    scale_z = (2.0 * GRAVITY) / (data['z_max'][2] - data['z_min'][2])

    print("\n================ CALIBRATION RESULTS ================")
    print(f"Bias Vector  (b_x, b_y, b_z): [{bias_x:.6f}, {bias_y:.6f}, {bias_z:.6f}]")
    print(f"Scale Factors(s_x, s_y, s_z): [{scale_x:.6f}, {scale_y:.6f}, {scale_z:.6f}]")
    print("====================================================\n")

if __name__ == "__main__":
    main()
