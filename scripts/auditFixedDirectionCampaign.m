function auditFixedDirectionCampaign(directory)
%auditFixedDirectionCampaign Independently inspect saved avoidance footprints.
% Use 5 ms samples and 0.1 ms refinement near closest approaches. This finite
% audit is not a continuous-time certificate. Timing experiments must finish
% before this offline reconstruction starts.
    arguments
        directory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
    files = dir(fullfile(directory,'diagnostic','*-1','*-exact-state.mat'));
    audits = cell(numel(files),1);
    for index = 1:numel(files)
        source = fullfile(files(index).folder,files(index).name);
        loaded = load(source,'report');
        report = loaded.report;
        cfg = report.configuration;
        h = cfg.controller.sampleTime;
        [a,b,c] = ltvBicycleModel.continuousMatrices(report.roadCurvature,cfg.referenceSpeed,cfg,[],0, ...
            struct('state',report.cruiseState,'input',report.cruiseInput));
        generator = [a,b,c;zeros(3,9)];
        flows = zeros(9,9,11);
        for part = 0:10
            flows(:,:,part+1) = expm(part*h/10*generator);
        end
        item = struct('scenario',report.scenario,'curvature',report.roadCurvature, ...
            'source',source,'executedHolds',report.executedHolds, ...
            'coarseStepSeconds',h/10,'refinedStepSeconds',0.0001, ...
            'endpointMaximumError',0,'minimumSampledSatGap',NaN, ...
            'minimumRefinedSatGap',NaN,'refinedMinimumTime',NaN, ...
            'minimumRefinedBodyGap',NaN,'refinedBodyMinimumTime',NaN, ...
            'minimumNodeSatGap',NaN,'refinedHolds',[], ...
            'nominalMinimumSatGap',Inf,'nominalMinimumTime',NaN, ...
            'times',[],'bodyGaps',[],'satGaps',[]);
        assert(all(report.targetMotion.jerkAmplitude==0) && report.targetMotion.yawAccelerationAmplitude==0);
        % Reconstruct the same held affine flow and use an independent SAT
        % implementation for overlap acceptance, outside all timing regions.
        sampleCount = report.executedHolds*10+1;
        stateSamples = zeros(6,sampleCount);
        times = (0:sampleCount-1)*h/10;
        holdMinimum = inf(1,report.executedHolds);
        endpointError = 0;
        for stage = 1:report.executedHolds
            initial = [report.state(:,stage);report.input(:,stage);1];
            for part = 0:10
                state = flows(:,:,part+1)*initial;
                stateSamples(:,(stage-1)*10+part+1) = state(1:6);
            end
            endpointError = max(endpointError,max(abs(state(1:6)-report.state(:,stage+1))));
        end
        if report.executedHolds>0
            assert(endpointError<1e-9,'Saved affine endpoints do not match reconstruction.');
            [satGaps,bodyGaps] = localGaps(stateSamples,times,report);
            assert(abs(min(bodyGaps)-report.minimumSampledBodyGap)<1e-9);
            assert(all((satGaps>0)==(bodyGaps>0)),'Independent overlap signs disagree.');
            for stage = 1:report.executedHolds
                holdMinimum(stage) = min(satGaps((stage-1)*10+(1:11)));
            end
            nodeMinimum = min(satGaps(1:10:end));
            [~,worstHold] = min(holdMinimum);
            [~,worstBodySample] = min(bodyGaps);
            worstBodyHold = min(report.executedHolds,floor((worstBodySample-1)/10)+1);
            selected = unique([find(holdMinimum<0.25),worstHold,worstBodyHold]);
            selected = unique([selected-1,selected,selected+1]);
            selected = selected(selected>=1 & selected<=report.executedHolds);
            refinedMinimum = Inf;
            refinedBodyMinimum = Inf;
            parts = round(h/0.0001);
            denseFlows = zeros(9,9,parts+1);
            for part = 0:parts
                denseFlows(:,:,part+1) = expm(part*h/parts*generator);
            end
            for stage = selected
                initial = [report.state(:,stage);report.input(:,stage);1];
                denseStates = zeros(6,parts+1);
                for part = 0:parts
                    state = denseFlows(:,:,part+1)*initial;
                    denseStates(:,part+1) = state(1:6);
                end
                denseTimes = (stage-1)*h+(0:parts)*h/parts;
                [denseSat,denseBody] = localGaps(denseStates,denseTimes,report);
                assert(all((denseSat>0)==(denseBody>0)));
                [gap,at] = min(denseSat);
                if gap<refinedMinimum
                    refinedMinimum = gap;
                    item.refinedMinimumTime = denseTimes(at);
                end
                [gap,at] = min(denseBody);
                if gap<refinedBodyMinimum
                    refinedBodyMinimum = gap;
                    item.refinedBodyMinimumTime = denseTimes(at);
                end
            end
            item.endpointMaximumError = endpointError;
            item.minimumSampledSatGap = min(satGaps);
            item.minimumRefinedSatGap = refinedMinimum;
            item.minimumRefinedBodyGap = refinedBodyMinimum;
            item.minimumNodeSatGap = nodeMinimum;
            item.refinedHolds = selected;
            item.times = times;
            item.bodyGaps = bodyGaps;
            item.satGaps = satGaps;
        end
        % Cruise counterfactual uses the same initial state and declared plant.
        % It establishes whether this case actually requires avoidance.
        nominal = report.state(:,1);
        nominalStates = zeros(6,report.sampleCount*10+1);
        nominalStates(:,1) = nominal;
        nominalTimes = (0:report.sampleCount*10)*h/10;
        for sample = 2:size(nominalStates,2)
            next = flows(:,:,2)*[nominal;report.cruiseInput;1];
            nominal = next(1:6);
            nominalStates(:,sample) = nominal;
        end
        nominalGaps = localGaps(nominalStates,nominalTimes,report);
        [item.nominalMinimumSatGap,at] = min(nominalGaps);
        item.nominalMinimumTime = nominalTimes(at);
        item.nominalSampledCollision = item.nominalMinimumSatGap<=0;
        item.nominalTimes = nominalTimes;
        item.nominalSatGaps = nominalGaps;
        audits{index} = item;
        fprintf('%s k=%g: refined SAT=%g m at %g s; nominal SAT=%g m; endpoint error=%g\n', ...
            report.scenario,report.roadCurvature,item.minimumRefinedSatGap, ...
            item.refinedMinimumTime,item.nominalMinimumSatGap,endpointError);
    end
    file = fopen(fullfile(directory,'independent-audit.json'),'w');
    assert(file>=0);
    cleanup = onCleanup(@()fclose(file));
    fprintf(file,'%s\n',jsonencode(audits,PrettyPrint=true));
end

function [satGaps,bodyGaps] = localGaps(states,times,report)
    if report.roadCurvature==0
        positions = report.road.centerline(1,:).'+states(1:2,:);
        headings = states(3,:);
    else
        [positions,headings] = laneGeometry.fromFrenet(states,report.road);
    end
    center = report.targetMotion.center;
    targetPositions = center(1:2)+center(3:4).*times+center(5:6).*times.^2/2;
    targetHeadings = center(7)+center(8).*times;
    cfg = report.configuration;
    satGaps = rectangleSeparationMargin(positions.',headings.',targetPositions.',targetHeadings.', ...
        cfg.vehicle.length,cfg.vehicle.width,2*report.targetMotion.halfLength,2*report.targetMotion.halfWidth).';
    if nargout>1
        bodyGaps = zeros(size(times));
        for sample = 1:numel(times)
            bodyGaps(sample) = avoidanceSafetyGeometry.rectangleDistance(positions(:,sample),headings(sample), ...
                targetPositions(:,sample),targetHeadings(sample), ...
                [cfg.vehicle.length/2;cfg.vehicle.width/2;report.targetMotion.halfLength;report.targetMotion.halfWidth]);
        end
    end
end
