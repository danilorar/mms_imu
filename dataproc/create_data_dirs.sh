#!/bin/bash

# Create data folder structure
mkdir -p data/raw/inbox
mkdir -p data/{raw,clean,filtered}/{acceleration,braking,cornering}/{soft,medium,hard}
mkdir -p data/results/figures

# Create metadata template
cat > metadata.csv << EOF
source_file,maneuver,setting,trial,notes
ts_xxx.csv,acceleration,soft,1,
ts_xxx.csv,acceleration,soft,2,
ts_xxx.csv,acceleration,soft,3,
ts_xxx.csv,acceleration,medium,1,
ts_xxx.csv,acceleration,medium,2,
ts_xxx.csv,acceleration,medium,3,
ts_xxx.csv,acceleration,hard,1,
ts_xxx.csv,acceleration,hard,2,
ts_xxx.csv,acceleration,hard,3,
ts_xxx.csv,braking,soft,1,
ts_xxx.csv,braking,soft,2,
ts_xxx.csv,braking,soft,3,
ts_xxx.csv,braking,medium,1,
ts_xxx.csv,braking,medium,2,
ts_xxx.csv,braking,medium,3,
ts_xxx.csv,braking,hard,1,
ts_xxx.csv,braking,hard,2,
ts_xxx.csv,braking,hard,3,
ts_xxx.csv,cornering,soft,1,
ts_xxx.csv,cornering,soft,2,
ts_xxx.csv,cornering,soft,3,
ts_xxx.csv,cornering,medium,1,
ts_xxx.csv,cornering,medium,2,
ts_xxx.csv,cornering,medium,3,
ts_xxx.csv,cornering,hard,1,
ts_xxx.csv,cornering,hard,2,
ts_xxx.csv,cornering,hard,3,
EOF

# Create working metadata.csv only if it does not already exist
if [ ! -f data/metadata.csv ]; then
  cp metadata.csv data/metadata.csv
fi

echo "Data folder structure created."
echo "metadata_template.csv created."