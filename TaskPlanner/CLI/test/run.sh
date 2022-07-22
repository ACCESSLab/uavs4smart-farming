#!/usr/bin/env bash
clear
EXE="../cmake-build-debug/BenchmarkAreaCoverage"
IMAGE="$(pwd)/inputs/aggieFarm.png"
echo "[+] loading image " $IMAGE
methods=(1 2 3)
for file in inputs/*
do
    JSON_FILE=$(pwd)/$file
    echo "[+] loading JSON_FILE " $JSON_FILE
    for m in "${methods[@]}"; do
      OUT_DIR="$(pwd)/result/$m/"
      $EXE --j $JSON_FILE --m $m --i $IMAGE --o $OUT_DIR
    done
done