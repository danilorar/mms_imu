import os 
import numpy as np
import pandas as pd

from pathlib import Path 
from load_data import load_imu
from trial_detection import detect_trials, split_trials

# define correct path
BASE_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = BASE_DIR / "data"

INBOX_DIR = DATA_DIR / "raw" / "opendlv"
METADATA_PATH = DATA_DIR / "metadata.csv"

# save split trial data to csv for post analysis
def save_trial_data(df, maneuver, setting, trial):
    out_dir = DATA_DIR / "raw" / maneuver / setting
    out_dir.mkdir(parents=True, exist_ok=True) # create the directory if it doesn't exist

    out_path = out_dir / f"{maneuver}_{setting}_trial{trial:02d}.csv" # save csv like "acc_brake_acceleration_trial01.csv"
    
    df.to_csv(out_path, index=False)
    print(f"Saved {out_path}")


def prepare_raw():
    metadata = pd.read_csv(METADATA_PATH)    

    for _, row in metadata.iterrows(): 
        
        # get info from metadata.csv
        source_file = str(row["source_file"]).strip() 
        setting = str(row["setting"]).strip() 
        maneuver = str(row["maneuver"]) 
        
        source_path = INBOX_DIR / source_file
        
        # check if source file exists
        if not source_path.exists():
            print(f"Source file {source_path} does not exist. Skipping.")
            continue 
         
        data = load_imu(source_path)
        runs = detect_trials(data, maneuver)   
        print(f"Detected {len(runs)} in: {setting}, {maneuver}")
        
        
        # main loop for splitting and saving trials from detect_trials output
        for trial, (start, end) in enumerate(runs, start=1): 
            trial_data = split_trials(data, start, end)
            save_trial_data(trial_data, maneuver, setting, trial)
            
            
if __name__ == "__main__":
    prepare_raw()