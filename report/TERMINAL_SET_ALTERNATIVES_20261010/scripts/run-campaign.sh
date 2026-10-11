#!/bin/bash
# Paired campaign: baseline 8faf395 (variantK0, git archive + solver symlink) against the
# working tree copy (variantL for the force-level controller of e2046e0, variantM for the
# steering-level controller adopted afterwards; tar copy made here), the same 98 encounters
# interleaved under the same load (campaign-pair.sh of report/TERMINAL_BACKUP_SET_20261008/scripts).
# Usage: run-campaign.sh <variantName>   (e.g. variantM)
set -u
V=${1:-variantM}
B=$HOME/.cache/collisionAvoidance/terminal_alternatives_20261010
REPO=$HOME/Downloads/ResearchProjects/collisionAvoidance
T=$REPO/report/TERMINAL_BACKUP_SET_20261008/scripts
mkdir -p $B
if [ ! -d $B/variantK0 ]; then
  mkdir -p $B/variantK0 && (cd $REPO && git archive 8faf395) | tar -x -C $B/variantK0
  ln -s $REPO/solver $B/variantK0/solver
fi
mkdir -p $B/$V
tar -C $REPO --exclude=./solver --exclude=./.git --exclude=./simulation_output --exclude=./paper --exclude=./reference -cf - . | tar -x -C $B/$V
[ -e $B/$V/solver ] || ln -s $REPO/solver $B/$V/solver
cd $B
D=$T VARIANT='struct()' bash $T/campaign-pair.sh $B/variantK0 $B/out-K0 $B/$V $B/out-${V#variant} > $B/campaign-pair-$V.log 2>&1
echo CAMPAIGN_PAIR_DONE >> $B/campaign-pair-$V.log
