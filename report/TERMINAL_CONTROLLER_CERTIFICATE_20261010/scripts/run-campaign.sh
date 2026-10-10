#!/bin/bash
# Paired campaign: baseline deb4dac (variantI2) against the working tree copy (variantJ, tar copy made beforehand).
set -u
B=$HOME/.cache/collisionAvoidance/terminal_certificate_20261010
REPO=$HOME/Downloads/ResearchProjects/collisionAvoidance
T=$REPO/report/TERMINAL_BACKUP_SET_20261008/scripts
cd $B
D=$T VARIANT='struct()' bash $T/campaign-pair.sh $B/variantI2 $B/out-I2 $B/variantJ $B/out-J > $B/campaign-pair.log 2>&1
echo CAMPAIGN_PAIR_DONE >> $B/campaign-pair.log
