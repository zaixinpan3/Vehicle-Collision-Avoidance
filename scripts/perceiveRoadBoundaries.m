function observed = perceiveRoadBoundaries(road,position,range,time)
%perceiveRoadBoundaries Ideal range-limited road scan for simulation.
% Full map polygons remain in this offline sensor. Only visible outer-boundary
% chains reach the controller, with drivable space on their left. Endpoints
% clipped by the perception disk are marked for tangent continuation. There
% is no disk-edge wall, occlusion model, future-map query or optional mode.
    arguments
        road (1,1) struct
        position (2,1) double {mustBeReal,mustBeFinite}
        range (1,1) double {mustBePositive,mustBeFinite}
        time (1,1) double {mustBeReal,mustBeFinite} = 0
    end
    persistent cachedKey cachedRings
    key=road;
    if isempty(cachedKey) || ~isequaln(cachedKey,key)
        map=localMap(road);[x,y]=boundary(map);cuts=[0;find(isnan(x));numel(x)+1];
        cachedRings=cell(0,1);
        for j=1:numel(cuts)-1
            points=[x(cuts(j)+1:cuts(j+1)-1),y(cuts(j)+1:cuts(j+1)-1)];
            if size(points,1)<3,continue;end
            if norm(points(1,:)-points(end,:))<1e-10,points(end,:)=[];end
            edge=points(2,:)-points(1,:);left=[-edge(2),edge(1)]/norm(edge);
            probe=(points(1,:)+points(2,:))/2+1e-5*left;
            if ~isinterior(map,probe(1),probe(2)),points=flipud(points);end
            cachedRings{end+1}=points; %#ok<AGROW>
        end
        cachedKey=key;
    end
    chains=struct('points',{},'extendStart',{},'extendEnd',{});
    for j=1:numel(cachedRings)
        points=cachedRings{j};points(end+1,:)=points(1,:);parts=chains([]);
        for k=1:size(points,1)-1
            a=points(k,:);v=points(k+1,:)-a;delta=a-position.';
            aa=v*v.';bb=2*delta*v.';cc=delta*delta.'-range^2;
            discriminant=bb^2-4*aa*cc;
            if discriminant<=0,continue;end
            roots=sort((-bb+[-1,1]*sqrt(discriminant))/(2*aa));
            low=max(0,roots(1));high=min(1,roots(2));
            if high-low<=1e-12,continue;end
            start=a+low*v;finish=a+high*v;
            firstCut=abs(norm(start-position.')-range)<1e-7;
            lastCut=abs(norm(finish-position.')-range)<1e-7;
            if ~isempty(parts) && norm(parts(end).points(end,:)-start)<1e-8
                parts(end).points(end+1,:)=finish;parts(end).extendEnd=lastCut;
            else
                parts(end+1)=struct('points',[start;finish],'extendStart',firstCut,'extendEnd',lastCut); %#ok<AGROW>
            end
        end
        if numel(parts)>1 && norm(parts(end).points(end,:)-parts(1).points(1,:))<1e-8
            parts(1).points=[parts(end).points;parts(1).points(2:end,:)];
            parts(1).extendStart=parts(end).extendStart;parts(end)=[];
        end
        chains=[chains,parts]; %#ok<AGROW>
    end
    if ~isempty(chains)
        starts=cell2mat(arrayfun(@(c)c.points(1,:),chains,'UniformOutput',false).');
        [~,order]=sortrows(starts);chains=chains(order);
    end
    observed=struct('lateralClearance',[], ...
        'boundaryObservation',struct('time',time,'origin',position,'range',range, ...
            'extension',"tangent",'chains',chains));
    for field=["centerline","points","referenceCurve"]
        if isfield(road,field),observed.(field)=road.(field);end
    end
end

function map=localMap(road)
    if isfield(road,'drivablePolygons')
        polygons=road.drivablePolygons;map=polyshape();
        for j=1:numel(polygons)
            vertices=polygons{j};map=union(map,polyshape(vertices(:,1),vertices(:,2)));
        end
        return;
    end
    width=9;if isfield(road,'scenarioHalfWidthMeters'),width=road.scenarioHalfWidthMeters;
    elseif isfield(road,'lateralClearance') && ~isempty(road.lateralClearance),width=road.lateralClearance;end
    if isscalar(width),width=[width;width];end
    if isfield(road,'referenceCurve')
        curve=road.referenceCurve;station=-100:.5:curve.length+200;
        [upper,~]=laneGeometry.referencePose(station,width(2),curve);
        [lower,~]=laneGeometry.referencePose(station,-width(1),curve);
        vertices=[upper,fliplr(lower)].';
    else
        points=road.centerline;direction=points(end,:)-points(1,:);direction=direction/norm(direction);
        normal=[-direction(2),direction(1)];
        vertices=[points(1,:)+width(2)*normal;points(end,:)+width(2)*normal; ...
            points(end,:)-width(1)*normal;points(1,:)-width(1)*normal];
    end
    map=polyshape(vertices(:,1),vertices(:,2));
end
