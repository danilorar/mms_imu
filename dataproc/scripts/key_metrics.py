import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft

# filtering  
def compute_residuals(raw, filtered, signal): 
     residual = raw[f"{signal}"] - filtered[f"{signal}"]
     print("raw std:", raw[f"{signal}"].std().round(4))
     print("filtered std:", filtered[f"{signal}"].std().round(4))
     print("residual std:", residual.std().round(4))

# root mean squared
def rms(raw, filtered, signal): 
      rms_raw = np.sqrt(np.mean(raw[f"{signal}"].values**2))
      rms_filtered = np.sqrt(np.mean(filtered[f"{signal}"].values**2))
      print("raw RMS:", rms_raw.round(4))
      print("filtered RMS:", rms_filtered.round(4))
      return rms_raw, rms_filtered
  
# wrapper function  
def noise_metrics(maneuver, setting, signal, data_dir="data"): # for 3 trials
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    for i, trial in enumerate([1, 2, 3]):
        clean_path = f"{data_dir}/clean/{maneuver}/{setting}/{maneuver}_{setting}_{trial}.csv"
        filtered_path = f"{data_dir}/filtered/{maneuver}/{setting}/{maneuver}_{setting}_{trial}.csv"

        # dataframes
        raw = pd.read_csv(clean_path)
        filtered = pd.read_csv(filtered_path)

        # metrics
        print(f"\nTrial {trial} | signal {signal}:")  
             
        # residuals
        print("=== Residuals ===")
        compute_residuals(raw, filtered, signal)
        
        # rms 
        print("=== RMS values ===")
        rms_raw, rms_filtered = rms(raw, filtered, signal)
        
        # plot
        axes[i].plot(raw["t"], raw[f"{signal}"], label=f"{signal} raw", alpha=0.5)
        axes[i].plot(filtered["t"], filtered[f"{signal}"], label=f"{signal} filtered")

        axes[i].set_title(f"Trial {trial}"); 
        axes[i].set_ylabel(f"{signal}"); axes[i].legend();axes[i].grid()

    axes[-1].set_xlabel("Time [s]")
    fig.suptitle(f"{maneuver} | {setting} | {signal}")
    plt.tight_layout()
    plt.show()


# Example usage
noise_metrics("braking", "soft", "ax")



