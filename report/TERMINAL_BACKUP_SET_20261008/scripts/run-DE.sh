#!/bin/bash
t=$(dirname "$0")
until grep -q RUN_A_DONE /tmp/claude-1000/-home-zai-Downloads-ResearchProjects-collisionAvoidance/6b13a40e-eaf5-43a6-a9bf-1a4b45984523/tasks/bedjko0h1.output 2>/dev/null; do sleep 20; done
D=$t VARIANT='struct()' $t/campaign.sh $t/variantD $t/out-D > $t/campaign-D.log 2>&1
D=$t VARIANT='struct()' $t/campaign.sh $t/variantE $t/out-E > $t/campaign-E.log 2>&1
echo RUN_DE_DONE
