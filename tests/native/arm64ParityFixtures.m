function arm64ParityFixtures(folder)
%arm64ParityFixtures Write reference values from the production MATLAB functions.
    root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(fullfile(root,'scripts','native'),fullfile(root,'scripts'),fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'estimator'));
    setupJointNlpSolver;
    saved=load(fullfile(folder,'generated','configuration.mat'),'cfg');
    cfg=saved.cfg;
    kernels=solvePredictiveControl("outputFeedback",struct('operation',"predictionKernels",'cfg',cfg));
    file=fopen(fullfile(folder,'parity.bin'),'w','ieee-le');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));
    fwrite(file,cfg.controller.sampleTime,'double');
    for mode=1:3
        track=[];
        for k=0:49
            time=k*cfg.outputFeedback.sensorPeriod;
            position=[23+8*time+.25*time^2;2+.5*time];
            heading=NaN;radius=.02;
            if mode==2
                position=[12;4];heading=.33;
            elseif mode==3 && k>10
                time=time+.6;
            end
            if k==12 || k==13,position=[];end
            [track,motion]=onlineNrmmTrackingRuntime('targetPoint',track,time,position,.1,cfg,heading,radius);
            detected=~isempty(position);
            if ~detected,position=[0;0];end
            if isempty(motion),motion=zeros(4,1);available=0;else,available=1;end
            bounds=track.motionBounds;bounded=~isempty(bounds);
            if ~bounded,bounds=zeros(2);end
            components=zeros(3,1);
            if ~isempty(track.errorBound),components=track.errorBound.targetComponents;end
            values=[time;position;.1;heading;radius;detected;track.state;motion;track.positionBounds(:); ...
                bounds(:);components;track.initialized;available;bounded;track.restCompatible];
            fwrite(file,values,'double');
        end
    end
    for member=1:3
        x=[1;-2;.3;8*(member-1);.05;.02];u=[.04;-.12];p=[1;.98;1.01;1];
        cap=kernels.rollout.size2_in(1);
        active=double((1:cap)<=min(cap,member+2));
        writeGraph(kernels.rollout,{x,repmat(u,1,cap),p,active});
        writeGraph(kernels.sensitivity,{x,u,p,1});
        writeGraph(kernels.inference,{x,u,p});
        writeGraph(kernels.hold,{x,u,p});
    end
    for speed=[0,3,15]
        x=[0;1;.2;speed;0;0];q=[12;2;.3;speed;-2;-.02;1.6;2.4;.9;0;0];
        shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
        [gap,g]=predictiveSafetyGeometry.rectangleClearance(x,q,shape);
        sector=terminalSafeSet.velocitySector(0,speed,.3,.15);
        [v,n,d,exitSteps]=terminalSafeSet.sectorSeparation(x,q,sector,.03,cfg);
        times=[0,.3,1,3,10];limits=[-2.1,-.03;-1.9,-.01];
        [boxes,centers]=predictiveSafetyGeometry.targetMotionEnvelope(q,limits,times,[],cfg.target.observerDomain);
        fwrite(file,[x;q;shape;gap;g;v;n;d(:);exitSteps;boxes(:);centers(:)],'double');
        curve=struct('origin',[-100;0],'heading',0,'curvature',.001,'length',1000);
        [v,n,d,exitSteps]=terminalSafeSet.movingValue(x,q,cfg);
        fwrite(file,[v;n;d(:);exitSteps],'double');
        steps=12;time=(0:steps)*cfg.controller.sampleTime;
        [boxes,centers]=predictiveSafetyGeometry.targetMotionEnvelope(q,limits,time,[],cfg.target.observerDomain);
        for covered=[false,true]
            if covered,box=boxes;center=centers;else,box=zeros(11,0);center=zeros(4,0);end
            inputs=pcbfFieldInputs(x,[0;0],q,curve,box,center,steps,cfg);
            fwrite(file,inputs,'double');
        end
    end
    function writeGraph(f,args)
        out=cell(1,f.n_out());[out{:}]=f(args{:});
        for j=1:numel(args),writeArray(args{j});end
        for j=1:numel(out),writeArray(full(out{j}));end
    end
    function writeArray(v)
        fwrite(file,numel(v),'int32');fwrite(file,v,'double');
    end
end
