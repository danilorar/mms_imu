# filter validation plots
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
    return df[signal].rolling(window=window,center=True,min_periods=1).mean()


def plot_filter_comparison(maneuver, setting, trial, signal):
    clean = load_signal(maneuver, setting, trial, stage="clean", signal=signal)
    filtered = load_signal(maneuver, setting, trial, stage="filtered", signal=signal)

    n = min(len(clean), len(filtered))
    clean = clean.iloc[:n]
    filtered = filtered.iloc[:n]

    residual = clean[signal] - filtered[signal]
    ma_signal = moving_average(clean, signal, window=5)

    fig, axs = plt.subplots(3, 1, figsize=(10, 8))

    sns.lineplot(x=clean.index, y=clean[signal], label="Clean", ax=axs[0], alpha=0.7)
    sns.lineplot(x=filtered.index, y=filtered[signal], label="Kalman", ax=axs[0], linestyle="--")
    sns.lineplot(x=clean.index, y=ma_signal, label="Moving average", ax=axs[0], linestyle=":")

    axs[0].set_title(f"{signal} comparison")
    axs[0].set_xlabel("Sample index")
    axs[0].set_ylabel(signal)
    axs[0].legend()
    axs[0].grid(True)

    sns.lineplot(x=clean.index, y=residual, label="Residual", ax=axs[1])

    axs[1].set_title(f"{signal} residual: clean - filtered")
    axs[1].set_xlabel("Sample index")
    axs[1].set_ylabel(signal)
    axs[1].legend()
    axs[1].grid(True)

    sns.histplot(residual, bins=30, kde=True, ax=axs[2])

    axs[2].set_title(f"{signal} residual histogram")
    axs[2].set_xlabel(f"{signal} residual")
    axs[2].grid(True)

    fig.suptitle(f"{maneuver} - {setting} - Trial {trial:02d}", fontsize=15)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
        plot_filter_comparison(
        maneuver="braking",
        setting="soft",
        trial=1,
        signal="az"
    )