#!/bin/bash
# campaign-pair.sh <srcA> <outA> <srcB> <outB>; environment: D, VARIANT
# Runs the same encounters (seeds 0, 20261003, 1..5 x speeds 8, 15 x seven
# scenarios) for two source copies, interleaved per encounter in one pool of
# 8 processes, so both variants see the same machine load.
srcA=$1; outA=$2; srcB=$3; outB=$4
scenarios="headOn acceleratingHeadOn brakingLead crossing turningCrossing curvedHeadOn curvedCrossing"
mkdir -p "$outA" "$outB"
for seed in ${SEEDS:-0 20261003 1 2 3 4 5}; do for speed in 8 15; do for name in $scenarios; do
  echo "$srcA $outA $seed $speed $name"; echo "$srcB $outB $seed $speed $name"; done; done; done \
 | xargs -P 8 -L 1 bash -c 'src=$0; out=$1; seed=$2; speed=$3; name=$4; timeout 3600 matlab -batch "addpath('"'"'$D'"'"'); v=$VARIANT; runOneEncounter('"'"'$src'"'"','"'"'$out'"'"',$speed,'"'"'$name'"'"',$seed,2000,v)" > "$out/log-seed$seed-speed$speed-$name.txt" 2>&1'
echo CAMPAIGN_DONE
