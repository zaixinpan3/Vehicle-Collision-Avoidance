#!/bin/bash
# Paired campaign with long-lived workers: three MATLAB sessions per arm, the
# signed-speed arm (git archive of ef6ca13) and the stop-at-rest arm (copy of
# the working tree) running the same encounter sequence side by side.
# A worker refused by the licensing service, or stopped, is restarted until its
# queue is done (completed encounters are skipped).
# Usage: run-workers.sh [frames] [workers per arm]   (defaults 2000, 3)
# ARMS selects the arms to run (default "signed stop").
set -u
FRAMES=${1:-2000}; W=${2:-3}
B=$HOME/.cache/collisionAvoidance/target_stop_20261011
D=$(cd "$(dirname "$0")" && pwd)
mkdir -p $B/out-signed $B/out-stop $B/workers
worker() {  # arm source output index
  local log=$B/workers/$1-$4.log
  for attempt in $(seq 1 30); do
    timeout 21600 matlab -batch "addpath('$D'); runEncounterQueue('$2','$3',$4,$W,$FRAMES)" >> $log 2>&1
    grep -q "QUEUE_DONE $4" $log && return
    sleep $((30 + RANDOM % 60))
  done
}
for i in $(seq 1 $W); do
  for arm in ${ARMS:-signed stop}; do
    worker $arm $B/$arm $B/out-$arm $i &
    sleep 25
  done
done
wait
echo CAMPAIGN_DONE
