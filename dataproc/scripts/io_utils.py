import os
import matplotlib.pyplot as plt
import pandas as pd

# =======================
# === IN-TEST CHECKER ===
# =======================

# read raw and return synchronized imu dataframe
def load_imu(csv_path_input, save, plot):
    ''' 
    - If save it will save the synchronized imu data in data/clean/maneuver/setting/
    - If plot it will display the synchronized imu data
    '''
    acc_data = []
    gyro_data = []

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

    # dataframes without syncronization
    acc = pd.DataFrame(acc_data,columns=["type", "time", "idx", "ax", "ay", "az"]).astype(float)
    gyro = pd.DataFrame(gyro_data,columns=["type", "time", "idx", "wx", "wy", "wz"]).astype(float)
    
    # concatenate into one imu_df based on "sensor_type"
    acc = pd.DataFrame({"sensor": "acc", "time": acc["time"], "ax": acc["ax"], "ay": acc["ay"], "az": acc["az"]})
    gyro = pd.DataFrame({"sensor": "gyro", "time": gyro["time"], "wx": gyro["wx"], "wy": gyro["wy"], "wz": gyro["wz"]})
    imu = pd.concat([acc, gyro], ignore_index=True)
    
    # to save in /clean folder
    if save:
        maneuver, setting, trial = parse_filename(csv_path_input)
        save_imu_data(
            imu,
            stage="clean",
            maneuver=maneuver,
            setting=setting,
            trial=trial
        )
    
    # can be deleted after testing is done
    if plot: 
        acc = imu[imu["sensor"] == "acc"]
        gyro = imu[imu["sensor"] == "gyro"]
        plt.figure(figsize=(12,6))
        
        plt.subplot(2,1,1)
        plt.plot(acc["ax"], label="ax")
        plt.plot(acc["ay"], label="ay")
        plt.plot(acc["az"], label="az")
        plt.title("Accelerometer")
        plt.ylabel("m/s²")
        plt.legend()
        plt.grid(True)
        
        plt.subplot(2,1,2)
        plt.plot(gyro["wx"], label="wx")
        plt.plot(gyro["wy"], label="wy")
        plt.plot(gyro["wz"], label="wz")
        plt.title("Gyroscope")
        plt.ylabel("rad/s")
        plt.legend()
        
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    return imu

# ==============================
# === FOLDER & FILE HANDLING ===
# ==============================

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

# =====================
# === KALMAN FILTER ===
# =====================

def kalman_filter(csv_path_input, q, r, save=True):
    # raw data 
    imu = load_imu(csv_path_input, save=save, plot=False)
    imu_kf = imu.copy()
    
    sensor_signals = {"acc":["ax", "ay", "az"], "gyro":["wx", "wy", "wz"]}

    for sensor, signals in sensor_signals.items():
        sensor_idx = imu["sensor"] == sensor    
        for signal in signals:
            z_vector = imu.loc[sensor_idx, signal].values

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

            imu_kf.loc[sensor_idx, signal] = filtered_signal

    if save:
        maneuver, setting, trial = parse_filename(csv_path_input)
        save_imu_data(
            imu_kf,
            stage="filtered",
            maneuver=maneuver,
            setting=setting,
            trial=trial
        )  
        
    # plot kalman filter (tdb)
    # if plot:

    return imu_kf


if __name__ == "__main__":
    # in-site checker
    # imu = load_imu(
    #     csv_path_input= "data/raw/ts_1778505496.csv",
    #     save=False,
    #     plot=True
    #     )
    # print(imu[imu["sensor"] == "acc"].head())
    # print(imu[imu["sensor"] == "gyro"].head())    
    
    kalman_filter(
        csv_path_input="data/raw/braking/soft/braking_soft_trial01.csv",
        q=0.001,
        r=0.1,
        save=True
    )
