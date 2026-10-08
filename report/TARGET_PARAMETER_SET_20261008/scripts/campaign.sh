#!/bin/bash
# campaign.sh <sourceCopy> <outDir> [scenarioList]; environment: D, VARIANT
# Runs noisy encounters (seeds 20261003,1..5 x speeds 8,15 x scenarios), 8 at a time.
src=$1; out=$2; scenarios=${3:-"headOn acceleratingHeadOn brakingLead crossing turningCrossing curvedHeadOn curvedCrossing"}
mkdir -p "$out"
export SRC=$src OUT=$out
for seed in 20261003 1 2 3 4 5; do for speed in 8 15; do for name in $scenarios; do
  echo "$seed $speed $name"; done; done; done | xargs -P 8 -L 1 bash -c 'seed=$0; speed=$1; name=$2; timeout 3600 matlab -batch "addpath('"'"'$D'"'"'); v=$VARIANT; runOneEncounter('"'"'$SRC'"'"','"'"'$OUT'"'"',$speed,'"'"'$name'"'"',$seed,2000,v)" > "$OUT/log-seed$seed-speed$speed-$name.txt" 2>&1'
echo CAMPAIGN_DONE
