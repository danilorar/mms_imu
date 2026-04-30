## MMS210

This folder contains the IMU data processing pipeline used for the suspension test data.

```text
Compare suspension settings: soft, medium, hard
Test maneuvers: acceleration, braking, cornering
Expected full dataset: 3 maneuvers × 3 settings × 3 trials = 27 CSV files
```

### Sparse clone only `dataproc`

If you only need the data processing folder:

```bash
git clone --filter=blob:none --sparse https://github.com/danilorar/mms_imu.git
cd mms_imu
git sparse-checkout set dataproc
cd dataproc
```

---

### Setup

Create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Create the data folder structure:

```bash
chmod +x create_data_dirs.sh
./create_data_dirs.sh
```

---

### Add data

Download the logger CSV files from `opendlv.io`.

The downloaded files will look like:

```text
ts_1747044188.csv
ts_1747044302.csv
ts_1747044550.csv
...
```

Place all downloaded files in:

```text
data/raw/inbox/
```

Example:

```text
data/raw/inbox/
├── ts_1747044188.csv
├── ts_1747044302.csv
├── ts_1747044550.csv
└── ...
```

---

### Fill `metadata.csv`

Open:

```text
data/metadata.csv
```

Use the metadata template to match each downloaded file to the correct maneuver, setting, and trial.

Example:

```csv
source_file,maneuver,setting,trial,notes
ts_1747044188.csv,acceleration,soft,1,
ts_1747044302.csv,braking,medium,1,
ts_1747044550.csv,cornering,hard,2,
```

Allowed values:

```text
maneuver: acceleration, braking, cornering
setting: soft, medium, hard
trial: 1, 2, 3
```

---

### Run the pipeline

From inside `dataproc/`:

```bash
python3 scripts/main.py
python3 scripts/prepare_raw.py
```

The script will:

```text
1. Read data/metadata.csv and organize files into data/raw/{maneuver}/{setting}/
2. Create synchronized IMU data in data/clean/
3. Create filtered IMU data in data/filtered/
4. Create KPI summary in data/results/kpis.csv
```

### Outputs

After running the pipeline:

```text
data/clean/              synchronized IMU data
data/filtered/           filtered IMU data
data/results/kpis.csv    KPI table
```

The KPI table has one row per run:

```text
maneuver | setting | trial | ax_rms | ay_rms | az_rms | ...
```
