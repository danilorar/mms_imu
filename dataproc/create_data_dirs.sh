#!/bin/bash

mkdir -p data/{raw,clean,filtered}/{acceleration,braking,cornering}/{soft,medium,hard}
mkdir -p data/results/figures

touch data/.gitkeep
touch data/results/.gitkeep

echo "Data folder structure recreated."
