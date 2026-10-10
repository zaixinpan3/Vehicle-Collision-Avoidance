function sweep(which,outFile)
root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
addpath(fullfile(root,'scripts'),fullfile(root,'controller'),fullfile(root,'config'));
boxes={[1,.15,.75,.15,.08],[1.5,.15,.75,.15,.08],[1.5,.2,1,.1,.05],[2,.2,1,.1,.05],[.75,.1,.5,.1,.05],[1.25,.15,.75,.12,.06]};
names=["certificationLateralMeters","certificationHeadingRadians","certificationSpeedMetersPerSecond","certificationLateralVelocityMetersPerSecond","certificationYawRateRadiansPerSecond"];
points=struct('configuration',{},'curvature',{});
for b=which
  clf=struct();for i=1:5,clf.(names(i))=boxes{b}(i);end
  for speed=[8,15],for curvature=[0,.005]
    points(end+1)=struct('configuration',struct('referenceSpeed',speed,'clf',clf),'curvature',curvature);
  end,end
end
for j=1:numel(points)
  fprintf('\n##### box %s speed %g curvature %g\n',mat2str(struct2array(points(j).configuration.clf)),points(j).configuration.referenceSpeed,points(j).curvature);
  try
    tic;synthesizeClfMatrices(points(j),OutputFile=outFile,RefreshExisting=false);fprintf('elapsed %.0f s\n',toc);
  catch e
    fprintf('FAILED: %s\n',e.message);
  end
end
end
