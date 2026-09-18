classdef avoidanceStageQp
    %avoidanceStageQp Equivalent sparse realization of the certified plan.
    methods (Static)
        function lifted=build(program)
            lifted=program;
            if ~isfield(program,'prediction') || program.terminalOptimization,return;end
            prediction=program.prediction;count=prediction.stageCount;
            n=2*count;original=numel(program.q);total=original+6*count;
            indices=reshape(original+(1:6*count),6,count);
            anchor=program.anchorPlan(1:n);
            centers=prediction.egoStateOffset+reshape( ...
                pagemtimes(prediction.egoStateMatrix,anchor),6,[]);
            % Stage dynamics x_k - A_k x_{k-1} - B_k u_k = B_k*anchor_k, assembled
            % from triplets in one sparse call.
            stageRows=reshape(1:6*count,6,count);
            [blockRow,blockColumn]=ndgrid(1:6,1:6);
            [inputRow,inputColumn]=ndgrid(1:6,1:2);
            inputRows=stageRows(inputRow(:),:);
            inputColumns=inputColumn(:)+2*((1:count)-1);
            inputValues=-reshape(prediction.stageMatrixB,12,count);
            stateRows=stageRows(blockRow(:),2:count);
            stateColumns=indices(blockColumn(:),1:count-1);
            stateValues=-reshape(prediction.stageMatrixA(:,:,2:count),36,max(count-1,0));
            dynamics=sparse([stageRows(:);inputRows(:);stateRows(:)],[indices(:);inputColumns(:);stateColumns(:)], ...
                [ones(6*count,1);inputValues(:);stateValues(:)],6*count,total);
            rhs=-reshape(pagemtimes(prediction.stageMatrixB,reshape(anchor,2,1,count)),6*count,1);
            geometric=numel(program.geometry.label);cursor=0;
            rowBlocks=cell(numel(program.geometry.local),1);columnBlocks=rowBlocks;valueBlocks=rowBlocks;
            bound=program.b;
            for cellIndex=1:numel(program.geometry.local)
                localRows=program.geometry.local(cellIndex);stage=localRows.stage;
                rows=cursor+(1:numel(localRows.bound));cursor=cursor+numel(rows);
                columns=2*stage-1:2*stage;coefficients=localRows.inputMatrix;
                if stage>1
                    columns=[columns,indices(:,stage-1).'];coefficients=[coefficients,localRows.stateMatrix];
                end
                % find returns row vectors for a one-row cell; keep columns.
                [r,c,v]=find(coefficients);
                rowBlocks{cellIndex}=reshape(rows(r),[],1);
                columnBlocks{cellIndex}=reshape(columns(c),[],1);valueBlocks{cellIndex}=reshape(v,[],1);
                bound(rows)=localRows.bound-localRows.stateMatrix*centers(:,stage) ...
                    +program.safetyBound(rows)-program.geometry.physicalBound(rows);
            end
            assert(cursor==geometric,'avoidanceStageQp:geometryRows','Inconsistent stage row mapping.');
            % Append the CLF slack column, which is zero in physical geometry rows.
            [extraRow,extraColumn,extraValue]=find(program.A(1:geometric,n+1:original));
            geometricMatrix=sparse([vertcat(rowBlocks{:});extraRow(:)],[vertcat(columnBlocks{:});n+extraColumn(:)], ...
                [vertcat(valueBlocks{:});extraValue(:)],geometric,total);
            matrix=[geometricMatrix;program.A(geometric+1:end,:),sparse(size(program.A,1)-geometric,6*count)];
            % The terminal modal cone acts on the final state. Its stored RHS
            % and numerical reserve are transferred without reconstruction.
            modal=program.terminal.modalMatrix;
            modeCount=size(modal,1);terminalMap=zeros(3*modeCount,6);
            terminalMap(2:3:end,program.terminal.stateIndex)=-real(modal);
            terminalMap(3:3:end,program.terminal.stateIndex)=-imag(modal);
            rows=size(matrix,1)-3*modeCount+1:size(matrix,1);
            matrix(rows,1:n)=0;matrix(rows,indices(:,end))=terminalMap;
            bound(rows)=bound(rows)+terminalMap*(prediction.egoStateOffset(:,end)-centers(:,end));
            % Stage blocks 2*blkdiag(0,P_k) on the lifted state columns,
            % assembled from triplets in one sparse call.
            values=zeros(36,count);stateLinear=zeros(6,count);
            for stage=1:count
                p=program.referenceMatrices(:,:,stage+1);block=zeros(6);block(2:6,2:6)=2*p;
                values(:,stage)=block(:);
                stateLinear(2:6,stage)=2*p*(centers(2:6,stage+1)-program.referenceStates(2:6,stage+1));
            end
            [blockRow,blockColumn]=ndgrid(1:6,1:6);
            stateRows=indices(blockRow(:),:);stateColumns=indices(blockColumn(:),:);
            hessian=sparse([1:n,n+1,stateRows(:).'],[1:n,n+1,stateColumns(:).'], ...
                [2*program.inputWeight(:).',2*program.slackWeight,values(:).'],total,total);
            linear=[-2*program.inputWeight.*program.referenceInputs(:);0;stateLinear(:)];
            lifted.P=hessian;lifted.q=linear;
            linearCount=program.cones(2);
            retained=localDistinctRows(matrix(1:linearCount,:),bound(1:linearCount));
            matrix=[matrix(retained,:);matrix(linearCount+1:end,:)];
            bound=[bound(retained);bound(linearCount+1:end)];
            lifted.A=[dynamics;matrix];lifted.b=[rhs;bound];
            lifted.cones=program.cones;lifted.cones(1)=6*count;
            lifted.cones(2)=numel(retained);
            lifted.retainedRows=[retained;(linearCount+1:numel(program.b)).'];
            lifted.anchorPlan=[program.anchorPlan;zeros(total-numel(program.anchorPlan),1)];
            lifted.stateIndex=indices;lifted.stateCenter=centers;
            lifted.physicalDecisionCount=original;
        end
    end
end

function retained=localDistinctRows(matrix,bound)
% Identical left sides need only their tightest RHS. No coefficients are
% rounded; the complete original rows remain in the independent verifier.
% Two fixed projections propose candidate groups; an exact comparison of
% the complete rows confirms every merge, so a projection collision can only
% retain a duplicate, never drop a distinct row.
    count=numel(bound);
    if count==0,retained=zeros(0,1);return;end
    columns=(1:size(matrix,2)).';
    projection=matrix*[cos(columns),sin(2*columns)];
    [~,~,group]=unique(projection,'rows');
    ordered=sortrows([group,bound,(1:count).'],[1,2,3]);
    rows=ordered(:,3);
    candidate=[false;diff(ordered(:,1))==0];
    duplicate=false(count,1);
    if any(candidate)
        transposed=matrix.';
        later=rows(candidate);earlier=rows([candidate(2:end);false]);
        difference=transposed(:,later)-transposed(:,earlier);
        duplicate(candidate)=~any(difference,1).';
    end
    retained=sort(rows(~duplicate));
end
