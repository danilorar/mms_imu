import pandas as pd
import matplotlib.pyplot as plt

acc_data = []
gyro_data = []

with open("demoData.csv", "r") as f:
    for line in f:
        parts = line.strip().split(",")

        if len(parts) < 6:
            continue

        msg_type = int(parts[0])

        if msg_type == 1:
            acc_data.append(parts[:6])
        elif msg_type == 2:
            gyro_data.append(parts[:6])

# dataframes
acc = pd.DataFrame(acc_data, columns=["type", "time", "idx", "ax", "ay", "az"]).astype(float)
gyro = pd.DataFrame(gyro_data, columns=["type", "time", "idx", "wx", "wy", "wz"]).astype(float)

# Sort timestamp/index
acc = acc.sort_values("idx")
gyro = gyro.sort_values("idx")

# relative time/index
acc["t"] = acc["idx"] - acc["idx"].iloc[0]
gyro["t"] = gyro["idx"] - gyro["idx"].iloc[0]

# Downsample for plotting
acc_plot = acc.iloc[::100]
gyro_plot = gyro.iloc[::20]

plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(acc_plot["t"], acc_plot["ax"], "-", markersize=2, label="ax")
plt.plot(acc_plot["t"], acc_plot["ay"], "-", markersize=2, label="ay")
plt.plot(acc_plot["t"], acc_plot["az"], "-", markersize=2, label="az")
plt.title("Accelerometer Data")
plt.xlabel("Relative index/time")
plt.ylabel("Acceleration [m/s²]")
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(gyro_plot["t"], gyro_plot["wx"], "-", markersize=2, label="wx")
plt.plot(gyro_plot["t"], gyro_plot["wy"], "-", markersize=2, label="wy")
plt.plot(gyro_plot["t"], gyro_plot["wz"], "-", markersize=2, label="wz")
plt.title("Gyroscope Data")
plt.xlabel("Relative index/time")
plt.ylabel("Angular Velocity [rad/s]")
plt.legend()

plt.tight_layout()
plt.show()