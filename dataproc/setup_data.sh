-#!/usr/bin/env bash 

# Prepare folders, download selected CSV files  

BASE_URL="https://opendlv.io/7f3t3c79b9"

DATA_DIR="data"
INBOX_DIR="$DATA_DIR/raw/opendlv"
METADATA_FILE="$DATA_DIR/metadata.csv"

FILES=(
  "ts_1778672581.csv"   # cornering soft
  "ts_1778671987.csv"   # cornering medium  ❌
  "ts_1778673001.csv"   # cornering hard

  "ts_1778675255.csv"   # acc/brake soft
  "ts_1778674201.csv"   # acc/brake medium
  "ts_1778677685.csv"   # acc/brake hard ❌
)

# create directories 
mkdir -p "$INBOX_DIR"
mkdir -p "$DATA_DIR/filtered"
mkdir -p "$DATA_DIR/cm"
#mkdir -p "$DATA_DIR/results"

# create metadata.csv
cat > "$METADATA_FILE" << EOF
source_file,setting,maneuver
ts_1778672581.csv,soft,cornering
ts_1778673001.csv,hard,cornering
ts_1778675255.csv,soft,acc_brake
ts_1778674201.csv,medium,acc_brake
EOF

for file in "${FILES[@]}"; do
    output="$INBOX_DIR/$file"

    echo "Downloading: $file"
    curl -sL "$BASE_URL/log/$file" -o "$output"  # download file to inbox/
done

echo "Done."
