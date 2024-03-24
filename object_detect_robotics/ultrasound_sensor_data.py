import serial
import matplotlib.pyplot as plt
from datetime import datetime
import time
import numpy as np

serial_port = "COM9"
baud_rate = 9600

ser = serial.Serial(serial_port, baud_rate)

times = []
distances = []
means = []
variances = []

try:
    while True:
        distance_str = ser.readline().decode('utf-8').strip()
        distance = float(distance_str)
        current_time = datetime.now()
        times.append(current_time)
        distances.append(distance)

        # Calculate mean and variance
        mean_distance = np.mean(distances)
        variance_distance = np.var(distances)

        means.append(mean_distance)
        variances.append(variance_distance)

        print(f"{current_time}: Distance = {distance} cm, Mean = {mean_distance} cm, Variance = {variance_distance} cm^2")
        time.sleep(1)

except KeyboardInterrupt:
    ser.close()

    # Plotting
    plt.plot(times, distances, marker='o', label='Distance')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('Ultrasonic Sensor Data')
    plt.xticks(rotation=45)
    plt.legend()
    plt.tight_layout()
    plt.show()

    plt.plot(times, means, marker='o', label='Mean')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('Ultrasonic Sensor Data')
    plt.xticks(rotation=45)
    plt.legend()
    plt.tight_layout()
    plt.show()

    plt.plot(times, variances, marker='o', label='Variance')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('Ultrasonic Sensor Data')
    plt.xticks(rotation=45)
    plt.legend()
    plt.tight_layout()
    plt.show()
