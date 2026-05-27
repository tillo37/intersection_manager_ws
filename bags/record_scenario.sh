#!/usr/bin/env bash
# Member 5 — Record all key topics for a 60-second scenario
# Usage: bash bags/record_scenario.sh [duration]
DURATION=${1:-60}
OUTPUT="bags/scenario_$(date +%Y%m%d_%H%M%S)"
echo "Recording for ${DURATION}s → ${OUTPUT}"
ros2 bag record \
  /vehicle/pose \
  /vehicle/velocity \
  /traffic/phase \
  /obstacles/pose \
  /emergency_stop \
  /viz/markers \
  -o "${OUTPUT}" \
  --max-bag-duration "${DURATION}"
echo "Done. Bag saved to ${OUTPUT}"
