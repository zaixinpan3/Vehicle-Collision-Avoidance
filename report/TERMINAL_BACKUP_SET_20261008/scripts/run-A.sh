#!/bin/bash
t=$(dirname "$0")
D=$t SEEDS=0 VARIANT='struct()' $t/campaign.sh ~/.cache/collisionAvoidance/contract_set_impl_20261008/baseline $t/out-base > $t/campaign-base-exact.log 2>&1
D=$t VARIANT='struct()' $t/campaign.sh $t/variantA $t/out-A > $t/campaign-A.log 2>&1
echo RUN_A_DONE
