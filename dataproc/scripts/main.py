import pandas as pd
import os
from pathlib import Path
from io_utils import kalman_filter
from metrics import kpis
from validate_kf import plot_filter_comparison

BASE_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = BASE_DIR / "data"

# ================
# == PARAMETERS ==
# ================
batch_run = True    
q = 0.001
r = 0.1

# =====================
# == DATA PROCESSING ==
# =====================
maneuvers = ["acceleration", "braking", "cornering"]
settings = ["soft", "medium", "hard"]
trials = [1, 2, 3]

all_kpis = []

# filter & save 
if batch_run== True:
     for maneuver in maneuvers:  
          for setting in settings: 
               for trial in trials: 
                    csv_input = (f"{DATA_DIR}/raw/{maneuver}/{setting}/{maneuver}_{setting}_trial{trial:02d}.csv") 
                    
                    print(f"Missing file, skipping: {csv_input}") if not os.path.exists(csv_input) else None
                    print(f"\nProcessing: {csv_input}")
                    
                    # filter 
                    imu_kf = kalman_filter(csv_input, q=q, r=r, save=True)
                    
                    # store kpi
                    kpi_row = kpis(imu_kf, maneuver, setting, trial)
                    all_kpis.append(kpi_row)

else: # return print a warning that files exists 
     print("Set batch_run=True to process all files.")
     

# save in dataframe 
kpi_df = pd.DataFrame(all_kpis)

os.makedirs(f"{DATA_DIR}/results", exist_ok=True)
kpi_df.to_csv(f"{DATA_DIR}/results/kpis.csv", index=False)

print("kpis saved to data/results/kpis.csv")


# =========================
# == DATA VISUALIZATION ===
# =========================

# filter validation for one ex
plot_filter_comparison(
         maneuver="cornering",
         setting="hard",
         trial=1,
         signal="wz",
         save=False
     )
