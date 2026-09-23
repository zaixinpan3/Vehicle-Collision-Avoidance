classdef avoidanceStageQp
    %avoidanceStageQp Equivalent sparse realization of the certified plan.
    methods (Static)
        function conic=fixedDirections(program,point,angles,cfg)
        % Optimize the complete trajectory with the supplied normals fixed.
            conic=localFixedDirectionConic(program,point,angles,cfg);
        end

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
            entries=0;
            for cellIndex=1:numel(program.geometry.local)
                localRows=program.geometry.local(cellIndex);
                entries=entries+nnz(localRows.inputMatrix);
                if localRows.stage>1,entries=entries+nnz(localRows.stateMatrix);end
            end
            rowEntries=zeros(entries,1);columnEntries=rowEntries;valueEntries=rowEntries;
            entryCursor=0;
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
                positions=entryCursor+(1:numel(v));entryCursor=entryCursor+numel(v);
                rowEntries(positions)=reshape(rows(r),[],1);
                columnEntries(positions)=reshape(columns(c),[],1);valueEntries(positions)=reshape(v,[],1);
                bound(rows)=localRows.bound-localRows.stateMatrix*centers(:,stage) ...
                    +program.safetyBound(rows)-program.geometry.physicalBound(rows);
            end
            assert(cursor==geometric,'avoidanceStageQp:geometryRows','Inconsistent stage row mapping.');
            % Append the CLF slack column, which is zero in physical geometry rows.
            [extraRow,extraColumn,extraValue]=find(program.A(1:geometric,n+1:original));
            geometricMatrix=sparse([rowEntries;extraRow(:)],[columnEntries;n+extraColumn(:)], ...
                [valueEntries;extraValue(:)],geometric,total);
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

function conic=localFixedDirectionConic(program,point,angles,cfg)
    program.anchorPlan=point(program.layout.planIndex);
    base=avoidanceStageQp.build(program);
    records=program.jointCertificate.records;count=numel(records);
    baseCount=numel(base.q);cursor=baseCount;
    recordCounts=localAuxiliaryCount(records);
    total=cursor+sum(recordCounts);
    equalityCount=base.cones(1);linearCount=base.cones(2);
    [linearRow,linearColumn,linearValue]=find(base.A(equalityCount+(1:linearCount),:));
    linearRow=linearRow(:);linearColumn=linearColumn(:);linearValue=linearValue(:);
    linearBounds=reshape(base.b(equalityCount+(1:linearCount)),[],1);
    [coneRow,coneColumn,coneValue]=find(base.A(equalityCount+linearCount+1:end,:));
    coneRow=coneRow(:);coneColumn=coneColumn(:);coneValue=coneValue(:);
    coneBounds=reshape(base.b(equalityCount+linearCount+1:end),[],1);
    coneSizes=reshape(base.cones(3:end),[],1);
    % Reserve triplets once. A support row touches at most six states and
    % its epigraph variables. Each active record adds at most two cones.
    % Auxiliary variables bound the absolute-value and yaw-hull row counts.
    activeCount=count;auxiliaryCount=total-cursor;
    linearNonzeros=numel(linearValue);linearRows=numel(linearBounds);
    coneNonzeros=numel(coneValue);coneRows=numel(coneBounds);coneCount=numel(coneSizes);
    linearExtra=17*auxiliaryCount+8*activeCount;
    coneExtra=3*auxiliaryCount+48*activeCount;
    linearRow=[linearRow;zeros(linearExtra,1)];
    linearColumn=[linearColumn;zeros(linearExtra,1)];
    linearValue=[linearValue;zeros(linearExtra,1)];
    linearBounds=[linearBounds;zeros(2*auxiliaryCount+activeCount,1)];
    coneRow=[coneRow;zeros(coneExtra,1)];
    coneColumn=[coneColumn;zeros(coneExtra,1)];
    coneValue=[coneValue;zeros(coneExtra,1)];
    coneBounds=[coneBounds;zeros(14*activeCount,1)];
    coneSizes=[coneSizes;zeros(4*activeCount,1)];
    for index=1:count
        item=records(index);stateIndex=1:6;
        % Assemble one record in its own coordinates, then map its triplets
        % once. Sparse temporaries need only six states and this
        % record's epigraph variables, rather than the full horizon width.
        width=6+recordCounts(index);localCursor=6;
        columns=[base.stateIndex(:,item.stage).',cursor+(1:recordCounts(index))];
        state=base.stateCenter(:,item.stage+1);theta=angles(index);
        normal=[cos(theta);sin(theta)];
        relative=item.positionOffset+item.positionMap*state;
        yaw=item.yawOffset+item.yawRow*state;
        positionMap=sparse(2,width);positionMap(:,stateIndex)=item.positionMap;
        yawRow=row(stateIndex,item.yawRow);
        egoAngle=theta-yaw;obstacleAngle=theta-item.targetYaw;
        constant=item.clearance+item.positionBall-program.jointCertificate.upperBound(index) ...
            +avoidanceSafetyGeometry.supportValue(item.targetHalfSize,item.targetYawRadius, ...
                [cos(obstacleAngle);sin(obstacleAngle)])+sum(abs(item.generators.'*normal));
        support=sparse(1,width);zrow=sparse(1,width);
        egoRadius=norm(item.egoHalfSize);
        if egoRadius>0 && any(item.yawRow~=0)
            argument=-[-sin(egoAngle);cos(egoAngle)]*yawRow;
            support=shapeSupport(item.egoHalfSize,item.egoYawRadius, ...
                [cos(egoAngle);sin(egoAngle)],argument);
            % The normal is constant. Only ego rotation needs a remainder:
            % ||R(-dpsi)n - (n-t*dpsi)|| <= dpsi^2/2 globally.
            z=allocate(1);zrow=row(z,1);
            addCone([1;-1;0],[zrow;zrow;sqrt(2*egoRadius)*yawRow]);
        else
            constant=constant+avoidanceSafetyGeometry.supportValue( ...
                item.egoHalfSize,item.egoYawRadius,[cos(egoAngle);sin(egoAngle)]);
        end
        inequality=support+zrow-normal.'*positionMap;
        addLinear(inequality,normal.'*relative-constant);
        assert(localCursor==width,'avoidanceStageQp:jointLayout','Inconsistent record layout.');
        cursor=cursor+recordCounts(index);
    end
    assert(cursor==total,'avoidanceStageQp:jointLayout','Inconsistent auxiliary-variable count.');
    linearMatrix=sparse(linearRow(1:linearNonzeros),linearColumn(1:linearNonzeros), ...
        linearValue(1:linearNonzeros),linearRows,total);
    coneMatrix=sparse(coneRow(1:coneNonzeros),coneColumn(1:coneNonzeros), ...
        coneValue(1:coneNonzeros),coneRows,total);
    linearBounds=linearBounds(1:linearRows);coneBounds=coneBounds(1:coneRows);
    coneSizes=coneSizes(1:coneCount);
    conic=struct('P',sparse(total,total),'q',zeros(total,1), ...
        'A',[[base.A(1:equalityCount,:),sparse(equalityCount,total-baseCount)];linearMatrix;coneMatrix], ...
        'b',[base.b(1:equalityCount);linearBounds;coneBounds], ...
        'cones',[equalityCount;numel(linearBounds);coneSizes], ...
        'anchorPlan',program.anchorPlan,'fixedCertificateAngles',angles, ...
        'primaryCount',numel(program.q));
    conic.P(1:baseCount,1:baseCount)=base.P;conic.q(1:baseCount)=base.q;
    weight=cfg.jointCertificate.proximalWeight;
    primary=1:numel(point);scale=[program.decisionRadius;1];
    diagonal=weight./max(scale,1e-3).^2;
    conic.P(primary,primary)=conic.P(primary,primary)+spdiags(diagonal,0,numel(point),numel(point));
    conic.q(primary)=conic.q(primary)-diagonal.*point;

    function indices=allocate(number)
        indices=localCursor+(1:number);localCursor=localCursor+number;
    end
    function out=row(indices,values)
        out=sparse(ones(size(indices)),indices,values,1,width);
    end
    function addLinear(matrix,bound)
        [ri,ci,vi]=find(matrix);
        selected=linearNonzeros+(1:numel(vi));
        linearRow(selected)=linearRows+ri(:);
        linearColumn(selected)=columns(ci);linearValue(selected)=vi(:);
        linearBounds(linearRows+(1:numel(bound)))=bound(:);
        linearNonzeros=linearNonzeros+numel(vi);linearRows=linearRows+numel(bound);
    end
    function addCone(offset,map)
        [ri,ci,vi]=find(-map);
        selected=coneNonzeros+(1:numel(vi));
        coneRow(selected)=coneRows+ri(:);
        coneColumn(selected)=columns(ci);coneValue(selected)=vi(:);
        coneBounds(coneRows+(1:numel(offset)))=offset(:);
        coneCount=coneCount+1;coneSizes(coneCount)=numel(offset);
        coneNonzeros=coneNonzeros+numel(vi);coneRows=coneRows+numel(offset);
    end
    function support=absoluteSupport(offset,map,weights)
        active=offset~=0 | any(map~=0,2);
        active=active & weights(:)~=0;
        offset=offset(active);map=map(active,:);weights=weights(active);
        number=numel(offset);
        if number==0,support=sparse(1,width);return;end
        indices=allocate(number);
        selector=sparse(1:number,indices,ones(1,number),number,width);
        addLinear([map-selector;-map-selector],[-offset;offset]);
        support=row(indices,weights(:).');
    end
    function support=shapeSupport(halfSize,radius,offset,map)
        if all(halfSize==0),support=sparse(1,width);return;end
        if radius==0
            support=absoluteSupport(offset,map,halfSize);return;
        end
        [directions,bounds,bodyRadius]=avoidanceSafetyGeometry.yawHull(halfSize,radius);
        multipliers=allocate(numel(bounds));supportIndex=allocate(1);
        support=row(supportIndex,1);top=support-row(multipliers,bounds.');
        bottom=bodyRadius*map;
        bottom(:,multipliers)=bottom(:,multipliers)-bodyRadius*directions.';
        addCone([0;bodyRadius*offset],[top;bottom]);
        addLinear(sparse(1:numel(bounds),multipliers,-ones(1,numel(bounds)),numel(bounds),width),zeros(numel(bounds),1));
    end
end

function counts=localAuxiliaryCount(records)
% Count each record once for both local and global sparse coordinates.
    counts=zeros(numel(records),1);
    for index=1:numel(records)
        item=records(index);
        count=0;halfSize=item.egoHalfSize;
        if any(halfSize~=0) && any(item.yawRow~=0)
            count=1;
            if item.egoYawRadius==0
                count=count+nnz(halfSize);
            else
                [~,bounds]=avoidanceSafetyGeometry.yawHull(halfSize,item.egoYawRadius);
                count=count+numel(bounds)+1;
            end
        end
        counts(index)=count;
    end
end

function retained=localDistinctRows(matrix,bound)
% Identical left sides need only their tightest RHS. No coefficients are
% rounded; duplicate rows with a looser RHS are implied by the retained row.
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
        duplicate(candidate)=full(~any(difference,1).');
    end
    retained=sort(rows(~duplicate));
end
