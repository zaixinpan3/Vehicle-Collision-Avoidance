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
            dynamics=sparse(6*count,total);rhs=zeros(6*count,1);
            for stage=1:count
                rows=6*(stage-1)+(1:6);inputs=2*stage-1:2*stage;
                dynamics(rows,indices(:,stage))=eye(6);
                dynamics(rows,inputs)=-prediction.stageMatrixB(:,:,stage);
                if stage>1
                    dynamics(rows,indices(:,stage-1))=-prediction.stageMatrixA(:,:,stage);
                end
                rhs(rows)=-prediction.stageMatrixB(:,:,stage)*anchor(inputs);
            end
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
                [r,c,v]=find(coefficients);
                rowBlocks{cellIndex}=reshape(rows(r),[],1);
                columnBlocks{cellIndex}=reshape(columns(c),[],1);valueBlocks{cellIndex}=v;
                bound(rows)=localRows.bound-localRows.stateMatrix*centers(:,stage) ...
                    +program.safetyBound(rows)-program.geometry.physicalBound(rows);
            end
            assert(cursor==geometric,'avoidanceStageQp:geometryRows','Inconsistent stage row mapping.');
            geometricMatrix=sparse(vertcat(rowBlocks{:}),vertcat(columnBlocks{:}), ...
                vertcat(valueBlocks{:}),geometric,total);
            geometricMatrix(:,n+1:original)=program.A(1:geometric,n+1:original);
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
            if isfield(program,'restoration') && program.restoration
                hessian=blkdiag(program.P,sparse(6*count,6*count));linear=[program.q;zeros(6*count,1)];
            else
                blocks=cell(count,1);stateLinear=zeros(6,count);
                for stage=1:count
                    p=program.referenceMatrices(:,:,stage+1);blocks{stage}=2*blkdiag(0,p);
                    stateLinear(2:6,stage)=2*p*(centers(2:6,stage+1)-program.referenceStates(2:6,stage+1));
                end
                hessian=blkdiag(2*spdiags(program.inputWeight,0,n,n), ...
                    2*program.slackWeight,sparse(blkdiag(blocks{:})));
                linear=[-2*program.inputWeight.*program.referenceInputs(:);0;stateLinear(:)];
            end
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
    count=numel(bound);[column,row,value]=find(matrix.');
    lengths=accumarray(row,1,[count,1]);width=max([lengths;0]);
    signature=zeros(count,2*width+1);signature(:,1)=lengths;
    ordinal=(1:numel(value)).'-repelem(cumsum(lengths)-lengths,lengths);
    signature(sub2ind(size(signature),row,2*ordinal))=column;
    signature(sub2ind(size(signature),row,2*ordinal+1))=value;
    [~,~,group]=unique(signature,'rows');
    ordered=sortrows([group,bound,(1:count).'],[1,2,3]);
    retained=sort(ordered([true;diff(ordered(:,1))~=0],3));
end
