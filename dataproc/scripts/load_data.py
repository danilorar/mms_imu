import os
import pandas as pd 
import numpy as np

def gps_to_xy(gps):
    R = 6371000  # Earth radius [m]

    lat = gps["lat"] * 3.1416 / 180.0
    lon = gps["lon"] * 3.1415 / 180.0

    lat0 = lat.iloc[0]
    lon0 = lon.iloc[0]

    x = R * (lon - lon0) * np.cos(lat0)
    y = R * (lat - lat0)

    return x, y


def load_imu(csv_path_input):
    """
    0 = GPS
    1 = accelerometer
    2 = gyroscope
    """

    gps_data = []
    acc_data = []
    gyro_data = []

    with open(csv_path_input, "r") as f:
        for line in f:
            parts = line.strip().split(",")

            if len(parts) < 6:
                continue

            msg_type = int(parts[0])

            if msg_type == 0 and len(parts) >= 7:
                gps_data.append(parts[:7])
            elif msg_type == 1 and len(parts) >= 6:
                acc_data.append(parts[:6])
            elif msg_type == 2 and len(parts) >= 6:
                gyro_data.append(parts[:6])

    # raw data
    acc_raw = pd.DataFrame(
        acc_data,
        columns=["type", "time", "idx", "ax", "ay", "az"]
    ).astype(float)

    gyro_raw = pd.DataFrame(
        gyro_data,
        columns=["type", "time", "idx", "wx", "wy", "wz"]
    ).astype(float)

    gps_raw = pd.DataFrame(
        gps_data,
        columns=["type", "time", "lat", "lon", "alt", "v_kmh", "satellites"]
    ).astype(float)

    gps_raw["x"], gps_raw["y"] = gps_to_xy(gps_raw) 
 
    # common time reference
    acc_raw = acc_raw.sort_values("time").reset_index(drop=True)
    gyro_raw = gyro_raw.sort_values("time").reset_index(drop=True)
    gps_raw = gps_raw.sort_values("time").reset_index(drop=True)

    t0 = min(acc_raw["time"].min(),gyro_raw["time"].min(),gps_raw["time"].min())

    acc_raw["t"] = acc_raw["time"] - t0
    gyro_raw["t"] = gyro_raw["time"] - t0
    gps_raw["t"] = gps_raw["time"] - t0
    
    # combine into one dataframe imu based on "sensor"
    acc = pd.DataFrame({
        "sensor": "acc",
        "time": acc_raw["time"],
        "t": acc_raw["t"],
        "ax": acc_raw["ax"],
        "ay": acc_raw["ay"],
        "az": acc_raw["az"],
    })

    gyro = pd.DataFrame({
        "sensor": "gyro",
        "time": gyro_raw["time"],
        "t": gyro_raw["t"],
        "wx": gyro_raw["wx"],
        "wy": gyro_raw["wy"],
        "wz": gyro_raw["wz"],
    })

    gps = pd.DataFrame({
        "sensor": "gps",
        "time": gps_raw["time"],
        "t": gps_raw["t"],
        "lat": gps_raw["lat"],
        "lon": gps_raw["lon"],
        "alt": gps_raw["alt"],
        "v_kmh": gps_raw["v_kmh"],
        "satellites": gps_raw["satellites"],
        "x": gps_raw["x"],
        "y": gps_raw["y"],
    })

    imu = pd.concat([acc, gyro, gps], ignore_index=True)
    
    return imu

if __name__ == "__main__":

    from plot_data import plot_trials
    import matplotlib.pyplot as plt
    # plt data to see if imu_data is correct 
    
    input_csv = "data/raw/opendlv/ts_1778675255.csv"
    load_imu(input_csv)
    
    plot_trials(input_csv, title="all") 
    plt.show()