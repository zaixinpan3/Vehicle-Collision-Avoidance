function [normals,information] = optimizeSeparationNormals(model,prediction,plan)
%optimizeSeparationNormals Continuous maximum-margin cell-normal proposals.
% A small SOCP separates synchronous relative Bernstein footprint enclosures.
% Returned normals are proposals; complete trajectory checking remains required.
    cfg=model.cfg;
    if exist("solveAvoidanceSocpMex","file")~=3
        addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))),'solver','clarabel','matlab'));
    end
    cells=prediction.cells;
    if isfield(prediction,'geometryFrames')
        frames=prediction.geometryFrames;
    else
        frames=laneGeometry.sweptCellFrames(model,cells,plan);
    end
    normals=cell(numel(cells),1);margins=zeros(numel(cells),numel(model.encounters));
    flags=zeros(size(margins));
    signs=[1,1,-1,-1;1,-1,1,-1];
    egoBody=[cfg.vehicle.length/2;cfg.vehicle.width/2].*signs;
    for index=1:numel(cells)
        tube=cells(index);frame=frames(index);
        nominal=reshape(pagemtimes(tube.map,plan),6,[])+tube.offset;
        map=[frame.tangent,frame.lateral];
        egoPosition=frame.origin+map*nominal(1:2,:);
        egoRadius=abs(map)*tube.radius(1:2,:)+frame.positionErrorBound;
        egoHeading=frame.heading+mean(nominal(3,:));
        headingRadius=max(abs(frame.heading+nominal(3,:)-egoHeading)+tube.radius(3,:))+frame.headingErrorBound;
        egoCorners=localRotation(egoHeading)*egoBody;
        normals{index}=zeros(2,numel(model.encounters));
        for targetIndex=1:numel(model.encounters)
            target=model.encounters(targetIndex);
            [center,radius]=targetPrediction.finiteFlow(target,tube.start);
            [last,lastRadius]=targetPrediction.finiteFlow(target,tube.start+tube.duration);
            degree=size(tube.offset,2)-1;
            transform=stateUncertainty.bernsteinTransform(degree,tube.duration);
            targetPosition=[center(1:2),center(3:4),center(5:6)/2,zeros(2,degree-2)]*transform.';
            targetRadius=[radius(1:2),radius(3:4),radius(5:6)/2, ...
                target.contract.jerkBound/6,zeros(2,degree-3)]*transform.';
            targetHeading=(center(7)+last(7))/2;
            targetHeadingRadius=abs(last(7)-center(7))/2+lastRadius(7);
            targetCorners=localRotation(targetHeading)*([target.halfLength;target.halfWidth].*signs);
            rotationRadius=hypot(cfg.vehicle.length/2,cfg.vehicle.width/2)*headingRadius ...
                +hypot(target.halfLength,target.halfWidth)*targetHeadingRadius;
            points=zeros(2,16*size(nominal,2));radii=points;
            cursor=0;
            for point=1:size(nominal,2)
                for a=1:4
                    selected=cursor+(1:4);
                    points(:,selected)=egoPosition(:,point)-targetPosition(:,point)+egoCorners(:,a)-targetCorners;
                    radii(:,selected)=repmat(egoRadius(:,point)+targetRadius(:,point),1,4);
                    cursor=cursor+4;
                end
            end
            matrix=[-points.',radii.',ones(cursor,1);eye(2),-eye(2),zeros(2,1); ...
                -eye(2),-eye(2),zeros(2,1)];
            bound=[-(cfg.collision.clearanceMargin+rotationRadius)*ones(cursor,1);zeros(4,1)];
            cone=[zeros(1,5);-eye(2),zeros(2,3)];
            [decision,status]=solveAvoidanceSocpMex(sparse(5,5),[0;0;0;0;-1], ...
                sparse([matrix;cone]),[bound;1;0;0],[0;numel(bound);3], ...
                [cfg.solver.constraintTolerance,cfg.solver.optimalityTolerance,cfg.solver.maxIterations]);
            flags(index,targetIndex)=status.status;
            if any(status.status==[1,4]) && norm(decision(1:2))>1e-8
                normals{index}(:,targetIndex)=decision(1:2)/norm(decision(1:2));
                margins(index,targetIndex)=decision(5);
            else
                [~,normals{index}(:,targetIndex)]=rectangleConfigurationDistance(mean(egoPosition,2),egoHeading, ...
                    mean(targetPosition,2),targetHeading,[cfg.vehicle.length/2;cfg.vehicle.width/2;target.halfLength;target.halfWidth]);
                margins(index,targetIndex)=-inf;
            end
        end
    end
    information=struct('proposalMargins',margins,'nativeStatus',flags, ...
        'scope',"normal proposals only; full original swept certificate must be checked");
end

function rotation=localRotation(angle)
    rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];
end
