import os
import pandas as pd
import folium
import matplotlib.pyplot as plt
from load_data import load_imu
from trial_detection import detect_trials

# plot gps map using 
def gps_map(gps, out_file=""):
    
    # coordinates for folium map
    coords = list(zip(gps["lat"], gps["lon"]))
    map = folium.Map(location=[gps["lat"].mean(), gps["lon"].mean()], zoom_start=18) 

    # plot gps path
    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri World Imagery",
        name="Satellite",
        overlay=False,
        control=True
    ).add_to(map)
    
    folium.PolyLine(coords).add_to(map)
    
    # add start and end markers to map
    folium.Marker(coords[0], tooltip="Start").add_to(map)
    folium.Marker(coords[-1], tooltip="End").add_to(map)
    
    # save as html
    folium.LayerControl().add_to(map)
    map.save(out_file)


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

    # GPS path in function of speed
    plt.subplot(2, 2, 4)
    plt.plot(gps["x"], gps["y"], label="GPS path", color="lightgrey", alpha=0.5)
    
    # colorbar speed
    plt.scatter(gps["x"], gps["y"], c=gps["v_kmh"], cmap="jet", label="GPS path") 
    cbar = plt.colorbar()
    cbar.set_label("Speed [km/h]")
    
    plt.title("GPS path")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis("equal")
    plt.legend()
    plt.grid(True)

    # show
    plt.suptitle(title)
    plt.tight_layout()
    plt.show()
    
    # plot gps map (html)
    gps_map(gps, out_file="gps_map.html")
    

def plot_cm(cm_files, real_files, cm_signal, real_signal, normalize_time=True, labels=["", ""]):
    

    fig, axes = plt.subplots(2, 1, figsize=(12, 9), sharex=False)
    fig.suptitle(f"CarMaker vs Real - {real_signal}")

    for cm_file, real_file, label in zip(cm_files, real_files, labels):
        cm = pd.read_csv(cm_file)
        real = pd.read_csv(real_file)

        cm_t = cm["Time [s]"] - cm["Time [s]"].iloc[0]
        real_t = real["t"] - real["t"].iloc[0]

        if normalize_time:
            cm_t = cm_t / cm_t.max()
            real_t = real_t / real_t.max()
            xlabel = "Normalized time [-]"
        else:
            xlabel = "Time [s]"

        axes[0].plot(cm_t, cm[cm_signal], label=label)
        axes[1].plot(real_t, real[real_signal], label=label)
        axes[1].plot(cm_t, cm[cm_signal], label="Real", alpha=0.5, linestyle="--")

    axes[0].set_title("CarMaker")
    axes[0].set_ylabel(cm_signal)
    axes[0].grid(True)
    axes[0].legend()

    axes[1].set_title("filtered data")
    axes[1].set_xlabel(xlabel)
    axes[1].set_ylabel(real_signal)
    axes[1].grid(True)
    axes[1].legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    
    plot_trials(
        csv_file="data/raw/opendlv/ts_1778674201.csv",
        title="all"
    )
    
    cm_files = [
        "data/carmaker/acc_brake_soft.csv",
        "data/carmaker/acc_brake_medium.csv",
        "data/carmaker/acc_brake_hard.csv",
    ]

    real_files = [
        "data/filtered/acc_brake/soft/acc_brake_soft_trial01.csv",
        "data/filtered/acc_brake/medium/acc_brake_medium_trial01.csv"
    ]

    # plot_cm(
    #     cm_files=cm_files,
    #     real_files=real_files,
    #     cm_signal="Speed [km/h]",
    #     real_signal="v_kmh",
    #     labels=["soft","hard"]
    # )