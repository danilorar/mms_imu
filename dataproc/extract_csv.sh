#!/usr/bin/env bash

BASE_URL="https://opendlv.io/7f3t3c79b9"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="$SCRIPT_DIR/data/raw/"

mkdir -p "$OUT_DIR"

curl -sL "$BASE_URL/logs" \
  | grep -oE 'ts_[0-9]+\.log' \
  | sed 's/\.log$/.csv/' \
  | sort -u \
  | while read filename; do
      echo "Downloading $filename"
      curl -sL "$BASE_URL/log/$filename" \
        -o "$OUT_DIR/$filename"
    done