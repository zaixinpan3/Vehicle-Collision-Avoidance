function margin = scenarioRoadFootprintMargin(pose,shape,curve,halfWidth)
%scenarioRoadFootprintMargin Offline full-footprint road containment audit.
% The inner edge of a circular road is nonconvex: checking only the vehicle's
% vertices misses an edge entering the excluded inner disk. Use the nearest
% point of the complete rectangle and the furthest vertex from the road center.
    c=cos(pose(3));s=sin(pose(3));rotation=[c,-s;s,c];
    corners=pose(1:2)+rotation*(shape(3:4)+shape(1:2).*[1,1,-1,-1;1,-1,1,-1]);
    if isfield(curve,'drivablePolygons')
        margin=localPolygonMargin(pose,shape,curve,corners);return;
    end
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

function margin=localPolygonMargin(pose,shape,road,corners)
    % Offline truth geometry only. Boundary segments intersecting the open
    % body interior catch corner cutting and holes missed by vertex tests.
    persistent cachedPolygons map start finish
    if isempty(cachedPolygons) || ~isequaln(cachedPolygons,road.drivablePolygons)
        map=polyshape();
        for j=1:numel(road.drivablePolygons)
            p=road.drivablePolygons{j};map=union(map,polyshape(p(:,1),p(:,2)));
        end
        [xx,yy]=boundary(map);cuts=[0;find(isnan(xx));numel(xx)+1];
        start=zeros(0,2);finish=zeros(0,2);
        for j=1:numel(cuts)-1
            p=[xx(cuts(j)+1:cuts(j+1)-1),yy(cuts(j)+1:cuts(j+1)-1)];
            if size(p,1)<3,continue;end
            if norm(p(1,:)-p(end,:))>1e-10,p(end+1,:)=p(1,:);end
            start=[start;p(1:end-1,:)];finish=[finish;p(2:end,:)]; %#ok<AGROW>
        end
        cachedPolygons=road.drivablePolygons;
    end
    r=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
    a=(start-pose(1:2).')*r-shape(3:4).';b=(finish-pose(1:2).')*r-shape(3:4).';
    v=b-a;low=zeros(size(a,1),1);high=ones(size(low));valid=true(size(low));
    for axis=1:2
        moving=abs(v(:,axis))>1e-12;half=max(0,shape(axis)-1e-9);
        valid(~moving)=valid(~moving) & abs(a(~moving,axis))<half;
        t1=(-half-a(moving,axis))./v(moving,axis);t2=(half-a(moving,axis))./v(moving,axis);
        low(moving)=max(low(moving),min(t1,t2));high(moving)=min(high(moving),max(t1,t2));
    end
    cuts=any(valid & high-low>1e-10);
    inside=all(isinterior(map,corners(1,:),corners(2,:))) && ~cuts;
    delta=finish-start;lengthSquared=sum(delta.^2,2);nearest=Inf;
    for j=1:4
        fraction=min(1,max(0,sum((corners(:,j).'-start).*delta,2)./lengthSquared));
        nearest=min(nearest,min(vecnorm(corners(:,j).'-start-fraction.*delta,2,2)));
    end
    nearest=min(nearest,min([vecnorm(max(abs(a)-shape(1:2).',0),2,2); ...
        vecnorm(max(abs(b)-shape(1:2).',0),2,2)]));
    % Negative values indicate noncontainment, not penetration depth.
    if inside,margin=nearest;else,margin=-max(1e-9,nearest);end
end
