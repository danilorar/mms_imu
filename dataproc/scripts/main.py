import os
from pathlib import Path

from prepare_raw import prepare_raw
from io_utils import kalman_filter
from scripts.validate_kf import plot_filter_comparison

BASE_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = BASE_DIR / "data"

# to be finished when metadata is ready

# ================
# == PARAMETERS ==
# ================
batch_run = False
q = 0.001
r = 0.1
prepare_raw()

# =====================
# == DATA PROCESSING ==
# =====================
if batch_run:
    raw_dir = DATA_DIR / "raw"

    csv_files = sorted([
        path for path in raw_dir.rglob("*.csv")
        if "inbox" not in path.parts
    ])

    print(f"\nFound {len(csv_files)} raw files to process.")

    for csv_input in csv_files:
        print(f"\nProcessing: {csv_input}")

        kalman_filter(csv_path_input=csv_input,q=q,r=r,save=True) # save=True to save clean and filtered
else:
    print("Set batch_run=True to process all files.")
     

# =========================
# == DATA VISUALIZATION ===
# =========================

# # filter validation for one ex
plot_filter_comparison(
         maneuver="cornering",
         setting="hard",
         trial=1,
         signal="wz",
         save=False
     )
