import pandas as pd
from pathlib import Path
from plot_kalman import plot_filter_comparison

BASE_DIR = Path(__file__).parent.parent
DATA_DIR = BASE_DIR / "data"
    

def kalman_filter(csv_path_input, q, r):
    # raw data 
    imu = pd.read_csv(csv_path_input)
    imu_kf = imu.copy()

    sensor_signals = {"acc":["ax", "ay", "az"], "gyro":["wx", "wy", "wz"]}

    for sensor, signals in sensor_signals.items():
        sensor_idx = imu_kf["sensor"] == sensor    
        
        for signal in signals:
            z_vector = imu_kf.loc[sensor_idx, signal].dropna().values

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
            
    
    csv_path_input = Path(csv_path_input).resolve()

    relative_path = csv_path_input.relative_to(DATA_DIR / "raw")
    output_path = DATA_DIR / "filtered" / relative_path

    output_path.parent.mkdir(parents=True, exist_ok=True)
    imu_kf.to_csv(output_path, index=False)

    print(f"Saved filtered data to: {output_path}")

    return imu_kf


if __name__ == "__main__":
    
    # tune kalman parameters:
    weights= [(0.0001, 0.5), (0.001, 0.1), (0.01, 0.05)]
    
    for q, r in weights:
        print(f"Testing q={q}, r={r}")
        kalman_filter(csv_path_input="data/raw/cornering/soft/cornering_soft_trial01.csv",q=q, r=r)
        plot_filter_comparison(
            maneuver="cornering",
            setting="soft",
            trial=1,
            signal="az",
            title=f"q={q}, r={r}"
        )
