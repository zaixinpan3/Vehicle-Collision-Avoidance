function inputs=pcbfFieldInputs(x,previous,target,curve,boxes,centers,count,cfg)
%pcbfFieldInputs Integrate the online vector field over the avoidance leg.
    lane=struct('referenceCurve',curve);
    frame=[curve.origin;curve.heading;curve.curvature];
    nominal=struct('state',[0;0;0;cfg.referenceSpeed;0;0],'curvature',frame(4));
    model=struct('cfg',cfg,'initialState',x,'previousInput',previous,'target',target, ...
        'lane',lane,'road',struct('lateralClearance',[]),'frame',frame,'nominalReference',nominal, ...
        'uncertainty',struct('collisionGenerator',zeros(6,0),'target',[]));
    model.motionCover=struct('boxes',boxes,'centers',centers);
    inputs=predictiveSafetyGeometry.avoidanceInputs(model,count);
end
