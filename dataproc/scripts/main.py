import os
from pathlib import Path
from kalman import kalman_filter
from plot_kalman import plot_filter_comparison
from plot_data import plot_trials
from prepare_raw import prepare_raw


BASE_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = BASE_DIR / "data"

# ==================
# === PARAMETERS ===
# ==================
q = 0.0001 # 0.001
r = 1.2    # 0.100
RUN_PREPARE_RAW = True
RUN_KALMAN = True

# =================
# === MAIN LOOP ===
# ================= 
def main():
    if RUN_PREPARE_RAW:
        prepare_raw()
        
    if RUN_KALMAN: 
        raw_dir = DATA_DIR / "raw"
        csv_files = sorted([
             path for path in raw_dir.rglob("*.csv")
            if "opendlv" not in path.parts])
    
        for csv_file in csv_files:
            print(f"\nProcessing: {csv_file}")
            kalman_filter(csv_path_input=csv_file, q=q, r=r)
        
        print("\nAll files processed ")

if __name__ == "__main__":
    main()
    
    plot_filter_comparison(
        maneuver="acc_brake",
        setting="soft",
        trial=3,
        signal="az",
        title=f"Kalman Filter (q={q}, r={r})")
    
    plot_trials(
        csv_file="data/raw/acc_brake/soft/acc_brake_soft_trial03.csv",
        title="")
    
    

