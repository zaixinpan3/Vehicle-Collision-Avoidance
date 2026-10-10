function levers(outFile)
root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
addpath(fullfile(root,'scripts'),fullfile(root,'controller'),fullfile(root,'config'));
for T=[4 6 8]
for steer=[.075 .1]
for speed=[8 15]
  clf=struct('convergenceTimeConstantSeconds',T,'certificationSteeringRadians',steer);
  point=struct('configuration',struct('referenceSpeed',speed,'clf',clf),'curvature',0);
  fprintf('\n##### T %g s, steering box %g rad, speed %g, straight\n',T,steer,speed);
  try
    tic;synthesizeClfMatrices(point,OutputFile=outFile,RefreshExisting=false,Verbose=false);fprintf('elapsed %.0f s\n',toc);
  catch e
    fprintf('FAILED: %s\n',e.message);
  end
end
end
end
end
