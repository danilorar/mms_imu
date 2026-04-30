# read raw and return synchronized imu dataframe
import os
from signal import signal

def load_and_sync_imu(csv_path_input):
    import pandas as pd 
    
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
    acc = pd.DataFrame(acc_data, columns=["type", "time", "idx", "ax", "ay", "az"]).astype(float)
    gyro = pd.DataFrame(gyro_data, columns=["type", "time", "idx", "wx", "wy", "wz"]).astype(float)
    
    # relative time
    acc["t"] = acc["time"] - acc["time"].iloc[0]
    gyro["t"] = gyro["time"] - gyro["time"].iloc[0]
    
    # merge 
    imu = pd.merge_asof(acc.sort_values("t"), gyro.sort_values("t"), on="t", direction="nearest")
    imu = imu[["t", "ax", "ay", "az", "wx", "wy", "wz"]]
    
    # save to csv and folder 
    maneuver_type, setting, trial = parse_filename(csv_path_input)
    save_imu_data(imu, stage="clean", maneuver_type=maneuver_type, setting=setting, trial=trial)
    
    return imu


# file handling
def parse_filename(csv_path): 
    
    name = os.path.basename(csv_path).replace(".csv", "")
    parts = name.split("_")

    # filename format: maneuver_setting_trialXX.csv
    maneuver_type = parts[0]
    setting = parts[1]
    trial = int(parts[2].replace ("trial", ""))

    return maneuver_type, setting, trial

def save_imu_data(df, stage, maneuver_type, setting, trial, data_dir="data"):
    
    folder = os.path.join(data_dir, stage, maneuver_type, setting)  
    os.makedirs(folder, exist_ok=True) # create folder if it doesn't exist

    filename = f"{maneuver_type}_{setting}_{trial}.csv"
    path = os.path.join(folder, filename)

    df.to_csv(path, index=False)
    print(f"Saved IMU data to {path}")
  
    
# 1D kalman filter 
def kalman_filter(csv_path_input, q, r, save=True):
    
    # raw data synchronized  
    imu = load_and_sync_imu(csv_path_input)
    imu_kf = imu.copy() # dataframe to store filtered values
    
    signals = ['ax', 'ay', 'az', 'wx', 'wy', 'wz']
    
    for signal in signals: 
        z_vector = imu[signal].values

        x = z_vector[0]  # initial state estimate
        P = 1.0 # initial estimate error covariance
        
        filtered_signal = [] # store filtered values
        
        for z in z_vector:
            # prediction 
            x_pred = x
            P_pred = P + q
            
            # update 
            K = P_pred / (P_pred + r) # kalman gain 
            x = x_pred + K * (z - x_pred) # update state estimate 
            P = (1-K) * P_pred # update error covariance 
            
            filtered_signal.append(x)
            
        imu_kf[signal] = filtered_signal
     
    if save:
        maneuver_type, setting, trial = parse_filename(csv_path_input) 
        save_imu_data(imu_kf, stage="filtered", maneuver_type=maneuver_type, setting=setting, trial=trial)
        
    return imu_kf
        
if __name__ == "__main__":
    pass 
    
    

