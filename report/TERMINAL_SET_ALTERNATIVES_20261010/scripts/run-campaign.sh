#!/bin/bash
# Paired campaign: baseline 8faf395 (variantK0, git archive + solver symlink) against the
# working tree copy (variantL, tar copy made here), the same 98 encounters interleaved
# under the same load (campaign-pair.sh of report/TERMINAL_BACKUP_SET_20261008/scripts).
set -u
B=$HOME/.cache/collisionAvoidance/terminal_alternatives_20261010
REPO=$HOME/Downloads/ResearchProjects/collisionAvoidance
T=$REPO/report/TERMINAL_BACKUP_SET_20261008/scripts
mkdir -p $B/variantL
tar -C $REPO --exclude=./solver --exclude=./.git --exclude=./simulation_output --exclude=./paper --exclude=./reference -cf - . | tar -x -C $B/variantL
[ -e $B/variantL/solver ] || ln -s $REPO/solver $B/variantL/solver
cd $B
D=$T VARIANT='struct()' bash $T/campaign-pair.sh $B/variantK0 $B/out-K0 $B/variantL $B/out-L > $B/campaign-pair.log 2>&1
echo CAMPAIGN_PAIR_DONE >> $B/campaign-pair.log
