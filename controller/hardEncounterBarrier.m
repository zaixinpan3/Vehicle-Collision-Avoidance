classdef hardEncounterBarrier
    %hardEncounterBarrier Sampled obstacle barrier plus swept intersample rows.
    % A single direction is chosen geometrically per target and hold. The
    % footprints are enclosed by orientation-independent circumdisks. This
    % conservative inner approximation keeps every inequality affine in u.
    methods (Static)
        function [matrix,bound,barrier] = rows(model,prediction,frames)
            targetCount=numel(model.encounters);
            degree=model.cfg.encounter.taylorOrder+1;
            rowCount=targetCount*(numel(prediction.cells)*(degree+1)+2);
            matrix=zeros(rowCount,2);bound=zeros(rowCount,1);cursor=0;
            barrier = struct('normal',zeros(2,targetCount),'initialLower',zeros(targetCount,1), ...
                'initialUpper',zeros(targetCount,1),'contraction',exp(-model.cfg.collision.cbfRate*model.sampleTime));
            [egoPosition,~] = laneGeometry.fromFrenet(model.initialEgoState,model.lane);
            for targetIndex = 1:numel(model.encounters)
                target = model.encounters(targetIndex);
                relative = egoPosition-target.center(1:2);
                normal = relative/max(norm(relative),realmin);
                if ~any(normal),normal=[1;0];end
                support = hypot(model.cfg.vehicle.length/2,model.cfg.vehicle.width/2) ...
                    +hypot(target.halfLength,target.halfWidth)+model.cfg.collision.clearanceMargin;
                frame = frames(1);
                positionMap = [frame.tangent,frame.lateral,zeros(2,4)];
                stateRow = normal.'*positionMap;
                center = normal.'*(frame.origin+positionMap*model.initialEgoState-target.center(1:2))-support;
                uncertainty = abs(stateRow)*model.initialFrenetErrorBound ...
                    +abs(normal).'*target.radius(1:2)+abs(normal).'*frame.positionErrorBound;
                initialLower = center-uncertainty;initialUpper = center+uncertainty;
                % Initial safe-set membership is itself a hard constant row.
                cursor=cursor+1;bound(cursor)=initialLower;
                for cellIndex = 1:numel(prediction.cells)
                    tube = prediction.cells(cellIndex);frame = frames(cellIndex);
                    positionMap = [frame.tangent,frame.lateral,zeros(2,4)];
                    stateRow = normal.'*positionMap;
                    [center,radius] = targetPrediction.finiteFlow(target,tube.start);
                    transform = stateUncertainty.bernsteinTransform(degree,tube.duration);
                    position = [center(1:2),center(3:4),center(5:6)/2,zeros(2,degree-2)]*transform.';
                    targetRadius = [radius(1:2),radius(3:4),radius(5:6)/2, ...
                        target.contract.jerkBound/6,zeros(2,degree-3)]*transform.';
                    map = reshape(pagemtimes(stateRow,tube.map),2,[]).';
                    limit = normal.'*frame.origin+stateRow*tube.offset-normal.'*position-support ...
                        -abs(stateRow)*tube.radius-abs(normal).'*targetRadius ...
                        -abs(normal).'*frame.positionErrorBound;
                    selected=cursor+(1:size(map,1));cursor=selected(end);
                    matrix(selected,:)=-map;bound(selected)=limit(:);
                    if cellIndex==numel(prediction.cells)
                        % b(h) >= exp(-rate*h) b(0), robust to both endpoint
                        % boxes. Bernstein endpoint enclosures include the
                        % held-flow truncation error already in tube.radius.
                        cursor=cursor+1;matrix(cursor,:)=-map(end,:);
                        bound(cursor)=limit(end)-barrier.contraction*initialUpper;
                    end
                end
                barrier.normal(:,targetIndex) = normal;
                barrier.initialLower(targetIndex) = initialLower;
                barrier.initialUpper(targetIndex) = initialUpper;
            end
        end
    end
end
