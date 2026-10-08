#!/bin/bash
d=$(dirname "$0")
SET="struct('enabled',true,'projectForecast',true,'maximumMeasurements',32,'iterations',3,'courseSliceWidth',0.1,'curvatureSliceWidth',0.008,'maximumSlices',64)"
D=$d VARIANT="struct('estimator',struct('observer',struct('runtime',struct('targetParameterSet',$SET))))" $d/campaign.sh $d/variant $d/out-v1-set > $d/campaign-v1.log 2>&1
D=$d VARIANT="struct('estimator',struct('observer',struct('runtime',struct('targetParameterSet',$SET))),'controller',struct('collision',struct('targetTubeOrigin','currentSample')))" $d/campaign.sh $d/variant $d/out-v2-fullfan > $d/campaign-v2.log 2>&1
echo VARIANTS_DONE
