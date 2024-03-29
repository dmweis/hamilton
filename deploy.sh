#!/bin/bash

set -o errexit
set -o nounset
set -o pipefail
set -o xtrace

readonly targets="
vehicle_config_test
lidar_driver
hamilton_controller
"

readonly TARGET_HOST=hamilton2.local
readonly TARGET_ARCH=armv7-unknown-linux-musleabihf

readonly TARGET_PATH=/home/pi/
readonly SOURCE_PATH=./target/${TARGET_ARCH}/release/

cargo build --release --target=${TARGET_ARCH} --no-default-features

for target in $targets; do
  target_path="$TARGET_PATH$target"
  source_path="$SOURCE_PATH$target"
  rsync -c ${source_path} pi@${TARGET_HOST}:${target_path}
done
