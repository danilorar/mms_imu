import os
import pandas as pd

# read raw and return synchronized imu dataframe
def load_and_sync_imu(csv_path_input, save=True):
    acc_data = []
    gyro_data = []

    if not os.path.exists(csv_path_input):
        raise FileNotFoundError(f"Input file not found: {csv_path_input}")

    # read csv and separate acc and gyro data
    with open(csv_path_input, "r") as f:
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
    acc = pd.DataFrame(acc_data,columns=["type", "time", "idx", "ax", "ay", "az"]).astype(float)
    gyro = pd.DataFrame(gyro_data,columns=["type", "time", "idx", "wx", "wy", "wz"]).astype(float)

    # relative time
    acc["t"] = acc["time"] - acc["time"].iloc[0]
    gyro["t"] = gyro["time"] - gyro["time"].iloc[0]

    # merge accel and gyro using nearest timestamp
    imu = pd.merge_asof(
        acc.sort_values("t"),
        gyro.sort_values("t"),
        on="t",
        direction="nearest"
    )

    imu = imu[["t", "ax", "ay", "az", "wx", "wy", "wz"]]

    if save:
        maneuver, setting, trial = parse_filename(csv_path_input)
        save_imu_data(
            imu,
            stage="clean",
            maneuver=maneuver,
            setting=setting,
            trial=trial
        )

    return imu


# file handling
def parse_filename(csv_path):
    name = os.path.basename(csv_path).replace(".csv", "")
    parts = name.split("_")

    # format: maneuver_setting_trialXX.csv
    maneuver = parts[0]
    setting = parts[1]
    trial = int(parts[2].replace("trial", ""))

    return maneuver, setting, trial


def save_imu_data(df, stage, maneuver, setting, trial, data_dir="data"):
    folder = os.path.join(data_dir, stage, maneuver, setting)
    os.makedirs(folder, exist_ok=True)

    filename = f"{maneuver}_{setting}_trial{trial:02d}.csv"
    path = os.path.join(folder, filename)

    df.to_csv(path, index=False)
    print(f"Saved IMU data to {path}")


# 1D Kalman filter
def kalman_filter(csv_path_input, q, r, save=True):
    # raw data synchronized
    imu = load_and_sync_imu(csv_path_input, save=True)
    imu_kf = imu.copy()

    signals = ["ax", "ay", "az", "wx", "wy", "wz"]

    for signal in signals:
        z_vector = imu[signal].values

        x = z_vector[0]
        P = 1.0

        filtered_signal = []

        for z in z_vector:
            # prediction
            x_pred = x
            P_pred = P + q

            # update
            K = P_pred / (P_pred + r)
            x = x_pred + K * (z - x_pred)
            P = (1 - K) * P_pred

            filtered_signal.append(x)

        imu_kf[signal] = filtered_signal

    if save:
        maneuver, setting, trial = parse_filename(csv_path_input)
        save_imu_data(
            imu_kf,
            stage="filtered",
            maneuver=maneuver,
            setting=setting,
            trial=trial
        )

    return imu_kf


if __name__ == "__main__":
    pass