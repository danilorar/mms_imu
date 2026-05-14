# MMS210

This folder contains the IMU data processing pipeline used for the suspension test data.

The goal is to compare different suspension settings using IMU data collected during driving maneuvers.

---

## Sparse clone `dataproc`

You only need the data processing folder:

```bash
git clone --filter=blob:none --sparse https://github.com/danilorar/mms_imu.git
cd mms_imu
git sparse-checkout set dataproc
cd dataproc
```

---

## Setup

Create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Create the data folder structure and import the OpenDLV logs:

```bash
chmod +x setup_bash.sh
./setup_bash.sh
```

The original log files will be placed in:

```text
data/raw/opendlv/
```

---

## Fill `metadata.csv`

Open:

```text
data/metadata.csv
```

Add each log file using this structure:

```csv
source_file,maneuver,setting,trial,notes
ts_1747044188.csv,acceleration,soft,1,
ts_1747044302.csv,braking,medium,1,
ts_1747044550.csv,cornering,hard,2,
```

Log names can be found [here](https://opendlv.io/7f3t3c79b9/logs).

Allowed values:

```text
maneuver: acceleration, braking, cornering
setting: soft, medium, hard
trial: 1, 2, 3
```

---

## Run the pipeline

From inside `dataproc/`:

```bash
python3 scripts/main.py
```

For the first run, make sure these flags are set to `True` inside `scripts/main.py`:

```python
RUN_PREPARE_RAW = True
RUN_KALMAN = True
```

The script will:

```text
1. Read the metadata file
2. Organize raw OpenDLV logs into data/raw/{maneuver}/{setting}/
3. Apply a simple Kalman filter to the IMU channels
4. Save filtered trial CSV files into data/filtered/{maneuver}/{setting}/
```

---

## Outputs

After running the pipeline, the folder structure will look like:

```text
data/
├── metadata.csv
├── raw/
│   ├── opendlv/              original downloaded OpenDLV logs
│   ├── acceleration/
│   ├── braking/
│   └── cornering/
└── filtered/
    ├── acceleration/
    ├── braking/
    └── cornering/
```

---

## Plotting

Plotting is available in:

```text
scripts/kalman_plot.py
scripts/load_plot.py
```

`kalman_plot.py` is mainly used to compare raw vs. filtered IMU signals.

`load_plot.py` is mainly used to load and visualize raw or processed trial data.

---

## Notes

The Kalman filter currently overwrites the filtered CSV file for the same maneuver, setting, and trial.

For Kalman tuning, use different output filenames or save tuning results with a suffix