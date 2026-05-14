from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns


BASE_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = BASE_DIR / "data"


def get_sensor_from_signal(signal):
    if signal in ["ax", "ay", "az"]:
        return "acc"
    elif signal in ["wx", "wy", "wz"]:
        return "gyro"
    elif signal in ["v_kmh"]:
        return "gps"
    else:
        raise ValueError(f"Unknown signal: {signal}")


def load_signal(maneuver, setting, trial, stage, signal):
    path = (
        DATA_DIR
        / stage
        / maneuver
        / setting
        / f"{maneuver}_{setting}_trial{trial:02d}.csv"
    )

    df = pd.read_csv(path)

    sensor = get_sensor_from_signal(signal)
    df = df[df["sensor"] == sensor].copy()

    return df[[signal]].dropna().reset_index(drop=True)


def moving_average(df, signal, window=5):
    return df[signal].rolling(window=window, center=True, min_periods=1).mean()


def plot_filter_comparison(maneuver, setting, trial, signal):
    
    # load csv data for raw and filtered signals
    raw = load_signal(maneuver, setting, trial, stage="raw", signal=signal)
    filtered = load_signal(maneuver, setting, trial, stage="filtered", signal=signal)

    # align lengths of raw and filtered data
    n = min(len(raw), len(filtered))
    raw = raw.iloc[:n]
    filtered = filtered.iloc[:n]

    # extras
    residual = raw[signal] - filtered[signal]
    ma_signal = moving_average(raw, signal, window=5)
    sample = range(len(raw))

    plt.figure(figsize=(12, 10))
    
    plt.subplot(3, 1, 1)
    plt.plot(raw[signal], label="Raw", alpha=0.3, linestyle="-")
    plt.plot(filtered[signal], label="Kalman", linewidth=2)
    plt.plot(ma_signal, label="Moving average", linestyle=":")

    plt.title(f"{signal} comparison")
    plt.xlabel("Sample index")
    plt.ylabel(signal)
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 2)
    plt.plot(residual, label="Residual",color="orange")
    plt.title(f"{signal} residual: raw - filtered")
    plt.xlabel("Sample index")
    plt.ylabel(signal)
    plt.legend()
    plt.grid(True)
    
    plt.subplot(3, 1, 3)
    sns.histplot(residual, bins=30, kde=True)
    plt.title(f"{signal} residual histogram")
    plt.xlabel(f"{signal} residual")
    plt.grid(True)
    plt.suptitle(f"{maneuver} - {setting} - Trial {trial:02d}", fontsize=15)
    plt.tight_layout()
    plt.show()

    
if __name__ == "__main__":
    plot_filter_comparison(
        maneuver="cornering",
        setting="soft",
        trial=1,
        signal="ax",
    )