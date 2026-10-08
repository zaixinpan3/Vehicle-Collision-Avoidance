function probeParameterSet(repo)
% Synthetic check of nrmmTargetParameterSet: containment and widths.
addpath(fullfile(repo,'estimator'),fullfile(repo,'controller'));
rng(3);
h=1/80;T=2.0;lr=1.6;yawAcc=7.3;gyroNoise=0.0015;radar=0.04;gnss=0.04;b=0.0075;
domain=struct('speedMinimum',1,'speedMaximum',21.67,'scalarAccelerationMaximum',1.1,'sideslipMaximum',0.055,'rearAxleDistance',lr);
% ego: speed 15, gentle turn yaw rate 0.1; target: head-on-ish with A=-0.5, beta=0.03
egoV=15;egoR=0.1;
qT=[45;3;pi;12;-0.5;0.03;lr;2.4;.95;0;0];
times=0:h:T;n=numel(times);
recs=struct('time',{},'relativePosition',{},'egoPosition',{},'yawRate',{},'heading',{},'headingRadius',{});
psi=0;pE=[0;0];bias=0.6*b;
for k=1:n
  t=times(k);
  if k>1
    % exact ego motion over the step (constant yaw rate)
    psiNew=psi+egoR*h;pE=pE+egoV/egoR*[sin(psiNew)-sin(psi);cos(psi)-cos(psiNew)];psi=psiNew;
  end
  q=predictiveSafetyGeometry.predictTarget(qT,t);
  R=[cos(psi),-sin(psi);sin(psi),cos(psi)];
  rho=R.'*(q(1:2)-pE)+radar*localDisk();
  recs(end+1)=struct('time',t,'relativePosition',rho,'egoPosition',pE+gnss*localDisk(), ...
      'yawRate',egoR+gyroNoise*(2*rand-1),'heading',psi+bias*cos(3*t),'headingRadius',b); %#ok<AGROW>
end
history=struct('records',recs,'gyroscopeNoise',gyroNoise,'yawAccelerationMaximum',yawAcc);
previous=[];
fprintf('  t    avail  meas  it  maxRem  | x[m]       y[m]      course[rad]  V[m/s]    A[m/s2]     kappa[1/m]   | truth inside\n');
for tk=0.3:0.1:T
  k=find(abs(times-tk)<1e-9);
  window=max(1,k-160):k;
  hist=history;hist.records=recs(window);
  q=predictiveSafetyGeometry.predictTarget(qT,tk);
  psi_k=0;pk=[0;0];
  % recompute ego truth at tk
  psi_k=egoR*tk;pk=egoV/egoR*[sin(psi_k);1-cos(psi_k)];
  R=[cos(psi_k),-sin(psi_k);sin(psi_k),cos(psi_k)];
  truth=[R.'*(q(1:2)-pk);atan2(sin(q(3)+q(6)-psi_k),cos(q(3)+q(6)-psi_k));q(4);q(5);sin(q(6))/lr];
  % a deliberately imperfect center, as a tracker would provide
  center=struct('relativePosition',truth(1:2)+[0.03;-0.02],'course',truth(3)+0.02,'speed',truth(4)+0.3, ...
      'acceleration',truth(5)+0.4,'curvature',truth(6)-0.01);
  prior=struct('positionRadius',0.07,'courseRadius',0.43,'speedInterval',truth(4)+[-3.3;3.3], ...
      'accelerationInterval',[-1.1;1.1],'curvatureInterval',[-0.034;0.034]);
  opts=struct('radarNoise',radar,'gnssNoise',gnss,'maximumMeasurements',32,'iterations',3,'courseSliceWidth',0.1,'curvatureSliceWidth',0.008,'maximumSlices',64);
  tic;s=nrmmTargetParameterSet(hist,tk,center,domain,prior,previous,opts);el=toc;
  if s.available
    lo=s.center+s.lower;hi=s.center+s.upper;inside=all(truth>=lo-1e-9 & truth<=hi+1e-9);
    fprintf('%4.1f  %d  %3d  %d(%3d)  %6.3f  | %5.2f  %5.2f  %6.3f  %5.2f  %5.2f  %5.2f\t(%.2fs) inside=%d\n',tk,s.solverFailures,s.measurements,s.iterations,s.slices,s.maximumRemainder, ...
      (hi(1)-lo(1))/2,(hi(2)-lo(2))/2,(hi(3)-lo(3))/2,(hi(4)-lo(4))/2,(hi(5)-lo(5))/2,(hi(6)-lo(6))/2,el,inside);
    if ~inside, disp([truth lo hi]); end
    previous=s;
  else
    fprintf('%4.1f  unavailable: %s\n',tk,s.reason);
  end
end
end
function v=localDisk()
r=sqrt(rand);a=2*pi*rand;v=r*[cos(a);sin(a)];
end
