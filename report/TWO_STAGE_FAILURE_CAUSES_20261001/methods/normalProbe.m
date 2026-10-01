function [stats,z]=normalProbe(p,objective)
    options=optimoptions('coneprog','Display','none','MaxIterations',400, ...
        'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-7,'MaxTime',5,'LinearSolver','normal');
    [z,f,flag,output]=coneprog(objective,p.cones,p.a,p.b,p.equal,p.rhs,p.lower,p.upper,options);
    stats=struct('flag',flag,'objective',f,'iterations',output.iterations, ...
        'primal',output.primalfeasibility,'dual',output.dualfeasibility,'gap',output.dualitygap, ...
        'message',string(output.message),'rho',NaN,'rhoNormalized',NaN,'safety',NaN, ...
        'rawResidual',NaN,'minimumCollisionMargin',NaN);
    if isempty(z),return;end
    stats.rhoNormalized=z(p.clfIndex);stats.rho=p.clfScale*max(0,z(p.clfIndex));
    stats.safety=sum(max(0,z(p.slackIndices)));
    values=[0;p.a*z-p.b;abs(p.equal*z-p.rhs);p.lower-z;z-p.upper];
    for cone=p.cones,values(end+1)=norm(cone.A*z-cone.b)-cone.d.'*z+cone.gamma;end
    stats.rawResidual=max(values);
    if ~isempty(p.collisionB),stats.minimumCollisionMargin=min(p.collisionB-p.collisionA*z);end
end
