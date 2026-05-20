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
  
    plt.plot(acc["t"], acc["ax"], label="ax")
    plt.plot(acc["t"], acc["ay"], label="ay")
    plt.plot(acc["t"], acc["az"], label="az")
    
    plt.title("Accelerometer")
    plt.xlabel("Time [s]")
    plt.ylabel("m/s²")
    plt.legend()
    plt.grid(True)

    # gyroscope
    plt.subplot(2, 2, 2)
    plt.plot(gyro["t"], gyro["wx"], label="wx")
    plt.plot(gyro["t"], gyro["wy"], label="wy")
    plt.plot(gyro["t"], gyro["wz"], label="wz")
    
    plt.title("Gyroscope")
    plt.xlabel("Time [s]")
    plt.ylabel("rad/s")
    plt.legend()
    plt.grid(True)

    # GPS speed
    plt.subplot(2, 2, 3)
    plt.plot(gps["t"], gps["v_kmh"], label="speed")
    
    plt.title("GPS speed")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [km/h]")
    plt.legend()
    plt.grid(True)

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
        real_t = real["t"] 

        if normalize_time:
            cm_t = cm_t / cm_t.max()
            real_t = real_t / real_t.max()
            xlabel = "Normalized time [-]"
        else:
            xlabel = "Time [s]"

        axes[0].plot(cm_t, cm[cm_signal], label=label)
        axes[1].plot(real_t,real[real_signal], label=label)
        #axes[1].plot(cm_t, cm[cm_signal], label="Carmaker", alpha=0.5, linestyle="--")

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


    
