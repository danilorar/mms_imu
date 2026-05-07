#!/usr/bin/env bash

BASE_URL="https://opendlv.io/46nenaw0yc"
OUT_DIR="dataproc/data/raw/inbox"

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