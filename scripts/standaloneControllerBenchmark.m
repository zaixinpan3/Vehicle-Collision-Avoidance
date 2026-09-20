classdef standaloneControllerBenchmark
%standaloneControllerBenchmark Preparation and capture for native frame replay.
% Generated artifacts belong outside the source tree. Only already prepared
% joint-certificate frames are exported; this is not a deployment interface.
    methods (Static)
        function p=pack(p)
            keep={'P','q','A','b','cones','anchorPlan','terminalOptimization','prediction','geometry','terminal', ...
                'referenceMatrices','referenceStates','referenceInputs','inputWeight','slackWeight','decisionRadius', ...
                'jointCertificate','physicalMatrix','physicalBound','safetyBound','terminalCone','terminalConePhysicalBound', ...
                'layout','feasibleWitness','clfNumericalReserve','inheritedPredictionFamily'};
            p=rmfield(p,setdiff(fieldnames(p),keep));
            p.prediction=rmfield(p.prediction,setdiff(fieldnames(p.prediction), ...
                {'stageCount','stageMatrixA','stageMatrixB','stageAffine','egoStateMatrix','egoStateOffset'}));
            g=p.geometry;
            p.geometry=struct('label',zeros(numel(g.label),1),'local',g.local, ...
                'frames',g.frames,'physicalBound',g.physicalBound);
            p.geometry.local=rmfield(g.local,setdiff(fieldnames(g.local), ...
                {'stage','inputMatrix','stateMatrix','bound'}));
            frame=struct('heading',0,'domainCenter',zeros(3,1),'domainRadius',Inf(3,1));
            frames=repmat(frame,numel(g.frames),1);
            for index=1:numel(frames)
                frames(index).heading=g.frames(index).heading;
                if isfield(g.frames,'domainCenter')
                    frames(index).domainCenter=g.frames(index).domainCenter;
                    frames(index).domainRadius=g.frames(index).domainRadius;
                end
            end
            p.geometry.frames=frames;
            p.terminal=rmfield(p.terminal,setdiff(fieldnames(p.terminal), ...
                {'modalMatrix','stateIndex','reference','input'}));
            keys=string({p.jointCertificate.records.key});
            [~,~,indices]=unique(keys,'stable');
            for index=1:numel(indices),p.jointCertificate.records(index).key=double(indices(index));end
        end

        function c=configuration(cfg)
            c=struct('jointCertificate',cfg.jointCertificate,'admission',cfg.admission, ...
                'encounter',struct('inputRateWeight',cfg.encounter.inputRateWeight), ...
                'controller',struct('sampleTime',cfg.controller.sampleTime), ...
                'solver',struct('constraintTolerance',cfg.solver.constraintTolerance, ...
                'optimalityTolerance',cfg.solver.optimalityTolerance,'maxIterations',cfg.solver.maxIterations));
        end

        function type=numericType(value)
            if isstruct(value)
                fields=fieldnames(value);types=struct();
                for index=1:numel(fields)
                    types.(fields{index})=standaloneControllerBenchmark.numericType(value(1).(fields{index}));
                end
                dimensions=size(value);variable=dimensions~=1;dimensions(variable)=Inf;
                type=coder.newtype('struct',types,dimensions,variable);
            else
                assert(isnumeric(value)||islogical(value),'Native fixtures must contain only numeric data.');
                dimensions=size(value);variable=dimensions~=1;dimensions(variable)=Inf;
                type=coder.typeof(value,dimensions,variable);
            end
        end

        function build(program,cfg,directory)
            root=fileparts(fileparts(mfilename('fullpath')));
            addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
            p=standaloneControllerBenchmark.pack(program);
            c=standaloneControllerBenchmark.configuration(cfg);
            settings=coder.config('lib');settings.TargetLang='C';settings.GenerateReport=true;
            settings.EnableOpenMP=false;
            settings.CustomInclude={fullfile(root,'scripts'),fullfile(root,'solver','clarabel','include')};
            settings.CustomSource={fullfile(root,'scripts','nativeControllerBenchmarkBridge.c')};
            codegen('-config',settings,'standaloneControllerFrame', ...
                '-args',{standaloneControllerBenchmark.numericType(p),coder.Constant(c)},'-d',directory);
            % Link outside MATLAB to avoid its private host-library environment.
            command="env -u LD_LIBRARY_PATH -u LD_PRELOAD python3 " ...
                +standaloneControllerBenchmark.quote(fullfile(root,'scripts','buildStandaloneReplay.py')) ...
                +" "+standaloneControllerBenchmark.quote(directory)+" --root " ...
                +standaloneControllerBenchmark.quote(root)+" --matlab-root " ...
                +standaloneControllerBenchmark.quote(matlabroot);
            [status,output]=system(command);assert(status==0,'%s',output);
            save(fullfile(directory,'configuration.mat'),'c');
        end

        function buildMex(program,cfg,directory)
            root=fileparts(fileparts(mfilename('fullpath')));
            addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
            p=standaloneControllerBenchmark.pack(program);c=standaloneControllerBenchmark.configuration(cfg);
            settings=coder.config('mex');settings.TargetLang='C';settings.GenerateReport=false;settings.EnableOpenMP=false;
            settings.CustomInclude={fullfile(root,'scripts'),fullfile(root,'solver','clarabel','include')};
            settings.CustomSource={fullfile(root,'scripts','nativeControllerBenchmarkBridge.c')};
            settings.CustomLibrary={fullfile(root,'solver','clarabel','rust_wrapper','target','release','libclarabel_c.a')};
            codegen('-config',settings,'standaloneControllerFrame', ...
                '-args',{standaloneControllerBenchmark.numericType(p),coder.Constant(c)}, ...
                '-d',fullfile(directory,'mex'),'-o',fullfile(directory,'standaloneControllerFrameMex'));
        end

        function records=capture(program,cfg)
            % The scenario copy calls this before solving. Capture I/O is never
            % included in a claimed controller timing measurement.
            persistent directory count recordsInternal
            if ischar(program) || isstring(program)
                directory=char(program);count=0;recordsInternal={};
                if ~isfolder(directory),mkdir(directory);end
                records=recordsInternal;return;
            end
            if isempty(program),records=recordsInternal;return;end
            if isempty(program.jointCertificate.records),records=recordsInternal;return;end
            count=count+1;
            file=fullfile(directory,sprintf('frame-%04d.mat',count));
            c=standaloneControllerBenchmark.configuration(cfg);p=standaloneControllerBenchmark.pack(program);
            save(file,'p','c','-v7');
            recordsInternal{end+1}=struct('file',file,'inherited',p.inheritedPredictionFamily, ...
                'horizonSteps',p.prediction.stageCount,'terminalOptimization',p.terminalOptimization);
            records=recordsInternal;
        end

        function [program,result,search]=nativeJoint(program,~,cfg)
            % Experimental integration only: the unchanged MATLAB driver
            % still prepares, transfers and finally verifies the certificate.
            assert(isempty(cfg.solver.jointFunction),'The native benchmark cannot execute a MATLAB solver hook.');
            start=tic;initialAngles=program.jointCertificate.angles;
            [decision,angles,status,metrics]=standaloneControllerFrameMex( ...
                standaloneControllerBenchmark.pack(program),standaloneControllerBenchmark.configuration(cfg));
            elapsed=toc(start);called=double(status==2 || status==3);
            search=struct('hardSolves',called,'restorationSolves',0,'baseSolves',0,'nativeSolves',called, ...
                'familyAttempts',double(status==1),'horizonAttempts',1, ...
                'formulationSeconds',elapsed-metrics(2),'solveSeconds',metrics(2), ...
                'initialOverlappingNodes',program.supportGeometry.overlappingNodes, ...
                'policy',"affineSectionAdmission",'violationHistory',{{}},'usedCertifiedIncumbent',status==3, ...
                'issuedAdmissionWitness',status==1,'initialCertificateAngles',initialAngles, ...
                'nativeKernel',true,'nativePhaseSeconds',metrics);
            result=struct('decision',decision,'fullDecision',decision,'feasible',status>0, ...
                'exitFlag',double(status>0),'message',"Generated numerical kernel status "+string(status), ...
                'output',struct());
            if status<=0,return;end
            program.jointCertificate.angles=angles;
            program=solveHardCbfClf.certify(program,decision);
            program.supportGeometry.witnessPreserved=program.inheritedPredictionFamily;
            program.inheritedFeasibleFamily=program.inheritedPredictionFamily;
        end

        function quoted=quote(value)
            mark=char(39);replacement=[mark,char(34),mark,char(34),mark];
            quoted=string([mark,strrep(char(value),mark,replacement),mark]);
        end
    end
end
