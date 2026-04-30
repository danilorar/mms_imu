# filter validation plots
import os
from os import path
import shutil 
from matplotlib.path import Path
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

def load_and_compress(maneuver, setting, trial, stage, data_dir="data/"):

    path = os.path.join(f"{data_dir}/{stage}/{maneuver}/{setting}/"f"{maneuver}_{setting}_trial{trial:02d}.csv")
    df = pd.read_csv(path)    
    
    # remove repeated timestamps by averaging values at same timestamp
    df = df.groupby("t", as_index=False).mean()

    return df

def moving_average(df, signal, window=25): 
     return df[signal].rolling(window=window, center=True, min_periods=1).mean()

def plot_filter_comparison(maneuver, setting, trial, signal, save=False):
     clean = load_and_compress(maneuver, setting, trial, stage="clean")
     filtered = load_and_compress(maneuver, setting, trial, stage="filtered")

     residual = clean[signal] - filtered[signal]
     
     ma_signal = moving_average(clean, signal, window=5) # moving average of clean signal
     
     fig, axs = plt.subplots(3,1, figsize=(10,8))
     
     # clean vs filtered vs ma
     sns.lineplot(x=clean["t"], y=clean[signal], label="Clean", ax=axs[0], alpha=0.7)
     sns.lineplot(x=filtered["t"], y=filtered[signal], label="Kalman", ax=axs[0], linestyle="--")
     sns.lineplot(x=clean["t"], y=ma_signal, label="Moving average",ax=axs[0], linestyle=":")
     
     axs[0].set_title(f"{signal} Comparison: Clean vs Filtered")
     axs[0].legend()
     axs[0].grid()
     
     # residual over time 
     sns.lineplot(x=clean["t"], y=residual, label="Residual", ax=axs[1])
     axs[1].set_title(f"{signal} Residual (Clean - Filtered)")
     axs[1].legend()
     axs[1].grid()   
     
     # histogram of residuals
     sns.histplot(residual, bins=30, kde=True, ax=axs[2])
     axs[2].set_xlabel(f"{signal} residual")
     axs[2].set_title(f"{signal} Residual Histogram")
     
     # zoom in on residuals
     axs[2].set_xlim(residual.quantile(0.01), residual.quantile(0.99))
     
     fig.suptitle(f"{maneuver} - {setting} - Trial {trial:02d}", fontsize=15)
     
     # save or show 
     if save:
          os.makedirs("data/results/figures", exist_ok=True)

          filename = ( f"{maneuver}_{setting}_trial{trial:02d}_"f"{signal}_filter_check.png")

          save_path = os.path.join("data/results/figures", filename)
          plt.savefig(save_path, dpi=300)
          plt.close()
          
          print(f"Saved plot to {save_path}")      
     else:
          plt.show()
          
if __name__ == "__main__":
     pass
     

