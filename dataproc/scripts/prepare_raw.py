import os
import pandas as pd
import shutil
from pathlib import Path

# prepare raw data from metadata.csv
BASE_DIR = Path(__file__).resolve().parents[1]
DATA_DIR = BASE_DIR / "data"

RAW_INBOX_DIR = DATA_DIR / "raw" / "inbox"
RAW_DIR = DATA_DIR / "raw"
METADATA_PATH = DATA_DIR / "metadata.csv"

def prepare_raw():
    metadata = pd.read_csv(METADATA_PATH)

    for _, row in metadata.iterrows():
        source_file = str(row["source_file"]).strip()
        maneuver = str(row["maneuver"]).strip()
        setting = str(row["setting"]).strip()
        trial = int(row["trial"])

        # skip empty template rows
        if source_file == "" or source_file == "nan":
            continue

        source_path = RAW_INBOX_DIR / source_file

        output_folder = RAW_DIR / maneuver / setting
        output_folder.mkdir(parents=True, exist_ok=True)

        output_path = output_folder / f"{maneuver}_{setting}_trial{trial:02d}.csv"

        shutil.copy2(source_path, output_path)

        print(f"Data prepared to: {output_path}")
        
        
if __name__ == "__main__":
    prepare_raw()