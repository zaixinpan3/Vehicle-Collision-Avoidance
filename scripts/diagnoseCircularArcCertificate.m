function items = diagnoseCircularArcCertificate(options)
%diagnoseCircularArcCertificate Audit chart conservatism without issuing inputs.
% For each original inequality A*U<=b, b+abs(A)*inputRadius is its largest
% possible margin over an OUTER actuator box. A negative value proves this
% row infeasible; a positive value does not prove the full program feasible.
% Directions are fixed on the cruise nominal; no branch search is performed.
    arguments
        options.OutputFile (1,1) string = ""
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'solver/bicycle'));
    items={};
    for curvature=[0,.0025,.01,.02]
        cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'controller',struct('sampleTime',.1,'minimumHorizonSteps',1),'model',struct('lateralDomainRadius',4)));
        curve=struct('origin',[0;0],'heading',0,'curvature',curvature,'length',min(340,1.9*pi/max(abs(curvature),eps)));
        road=struct('referenceCurve',curve,'centerline',laneGeometry.referencePose(linspace(0,curve.length,201),0,curve).');
        [z,~]=ltvBicycleModel.cruiseEquilibrium(curvature,cfg);
        [p,yaw]=laneGeometry.referencePose(15,0,curve);
        target=struct('trackId',1,'targetPositionInertial',p,'targetVelocityInertial',[0;0],'targetAccelerationInertial',[0;0],'targetHeadingInertial',yaw,'targetYawRate',0,'predictionMotion',struct('kind','nrmm-motion-v1','curvatureMaximum',0.05));
        measurement=struct('position',[0;0],'yaw',z(3),'speed',z(4),'lateralVelocity',z(5),'yawRate',z(6),'stateTime',0,'perception',struct('time',0,'range',16,'completeWithinRange',true));
        [ego,lane,parsedRoad,observations]=readPlanningInputs(measurement,target,road,cfg);
        model=struct('cfg',cfg,'lane',lane,'road',parsedRoad,'stateTime',0,'sampleTime',.1,'horizonSteps',16,'referenceSpeed',8,'initialEgoState',z,'initialFrenetErrorBound',zeros(6,1),'longitudinalAccelerationBias',0,'previousInput',[0;0],'requiredMargin',0,'confirmation',[]);
        [model,~]=hardEncounterBarrier.prepare(model,ego,observations,[],struct());
        proposed=model.horizonSteps;
        for steps=[16,24,32,40,48,56,64]
            model.horizonSteps=steps;
            [program,~]=formulateAvoidanceProblem(model);
            a=program.physicalMatrix(:,program.layout.planIndex);b=program.physicalBound;
            reach=program.decisionRadius;
            bestMargin=b+abs(a)*reach;
            collision=startsWith(program.physicalLabels,'collision:');exitRows=startsWith(program.physicalLabels,"exit:");
            assert(any(exitRows),'The experiment requires an active finite-exit row.');
            f=program.geometry.frames;
            errors=reshape([f.positionErrorBound],2,[]);
            cf=program.completion.frame;
            item=struct('curvature',curvature,'steps',steps,'proposedSteps',proposed,'minimumCollisionRowBestBoxMargin',min(bestMargin(collision)), ...
                'impossibleCollisionRows',nnz(bestMargin(collision)<-1e-8),'exitBestBoxMargin',bestMargin(exitRows), ...
                'maximumCellChartPositionError',max(errors,[],'all'),'finalCellChartPositionError',errors(:,end), ...
                'terminalChartPositionError',cf.positionErrorBound,'terminalChartStationSpan',cf.stationUpper-cf.stationLower, ...
                'minimumTerminalSocRadius',min(program.terminalCone.bound(1:3:end)), ...
                'firstCellChartPositionError',errors(:,1));
            items{end+1}=item;
            fprintf('k=%g N=%d cellError=%g exitError=%g impossibleRows=%d exitBest=%g\n',curvature,steps,item.maximumCellChartPositionError,max(cf.positionErrorBound),item.impossibleCollisionRows,item.exitBestBoxMargin);
        end
    end
    if strlength(options.OutputFile)>0
        file=fopen(options.OutputFile,'w');assert(file>=0);
        cleanup=onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(items,PrettyPrint=true));
    end
end
