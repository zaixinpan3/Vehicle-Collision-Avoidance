function margin = scenarioRoadFootprintMargin(pose,shape,curve,halfWidth)
%scenarioRoadFootprintMargin Exact rectangle margin in a straight/circular road.
% The inner edge of a circular road is nonconvex: checking only the vehicle's
% vertices misses an edge entering the excluded inner disk. Use the nearest
% point of the complete rectangle and the furthest vertex from the road center.
    c=cos(pose(3));s=sin(pose(3));rotation=[c,-s;s,c];
    corners=pose(1:2)+rotation*(shape(3:4)+shape(1:2).*[1,1,-1,-1;1,-1,1,-1]);
    if curve.curvature==0
        lateral=[-sin(curve.heading),cos(curve.heading)]*(corners-curve.origin);
        margin=min(halfWidth-abs(lateral));return;
    end
    center=curve.origin+[-sin(curve.heading);cos(curve.heading)]/curve.curvature;
    radius=1/abs(curve.curvature);
    local=rotation.'*(center-pose(1:2))-shape(3:4);
    nearest=norm(max(abs(local)-shape(1:2),0));
    furthest=max(vecnorm(corners-center,2,1));
    margin=min(nearest-(radius-halfWidth),radius+halfWidth-furthest);
end
