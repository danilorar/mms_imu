import os
import pandas as pd
import matplotlib.pyplot as plt
from load_data import load_imu
from trial_detection import detect_trials

# reset_index used to align indices, besides gps
def plot_trials(csv_file , runs=None, title=""):
    if title == "all":
        data = load_imu(csv_file)  
    else: 
        data = pd.read_csv(csv_file)
        
    maneuver = "acc_brake" if "acc_brake" in csv_file else "cornering"
    
    
    acc = data[data["sensor"] == "acc"]
    gyro = data[data["sensor"] == "gyro"]
    gps = data[data["sensor"] == "gps"]

    plt.figure(figsize=(12, 8))

    # accelerometer
    plt.subplot(2, 2, 1)
    plt.plot(acc["ax"].reset_index(drop=True), label="ax")
    plt.plot(acc["ay"].reset_index(drop=True), label="ay")
    plt.plot(acc["az"].reset_index(drop=True), label="az")
    plt.title("Accelerometer")
    plt.xlabel("Time [s]")
    plt.ylabel("m/s²")
    plt.legend()
    plt.grid(True)

    # gyroscope
    plt.subplot(2, 2, 2)
    plt.plot(gyro["wx"].reset_index(drop=True), label="wx")
    plt.plot(gyro["wy"].reset_index(drop=True), label="wy")
    plt.plot(gyro["wz"].reset_index(drop=True), label="wz")
    plt.title("Gyroscope")
    plt.xlabel("Time [s]")
    plt.ylabel("rad/s")
    plt.legend()
    plt.grid(True)

    # GPS speed
    plt.subplot(2, 2, 3)
    plt.plot(gps["t"].reset_index(drop=True), gps["v_kmh"].reset_index(drop=True), label="speed")
    plt.title("GPS speed")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [km/h]")


    # detect trials and plot as shaded areas
    runs = detect_trials(gps, maneuver)
    for start, end in runs:
        plt.axvspan(start, end, color="grey", alpha=0.2)

    plt.legend()
    plt.grid(True)

    # GPS path
    plt.subplot(2, 2, 4)
    plt.plot(gps["x"], gps["y"], label="GPS path")
    plt.title("GPS path")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)

    plt.suptitle(title)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # one separated trial
    plot_trials(
        "data/raw/cornering/hard/cornering_hard_trial02.csv",
        title=""
    )

    # full original log
    plot_trials(
        "data/raw/opendlv/ts_1778673001.csv",
        title="all"
    )