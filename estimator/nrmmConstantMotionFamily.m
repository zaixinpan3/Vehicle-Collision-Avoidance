function value = nrmmConstantMotionFamily(action,varargin)
%nrmmConstantMotionFamily Retain a cover of one constant-NRMM parameter set.
% Each column is a box in [epoch speed; tangential acceleration; curvature;
% epoch velocity course]. The same parameters generate every past and future
% position. Measurement rejection removes whole boxes only; unresolved boxes
% survive refinement. Bounds are analytic real-arithmetic outer bounds with
% roundoff padding, not interval-library floating-point certificates.
    switch string(action)
        case "condition"
            value=localCondition(varargin{:});
        case "image"
            value=localImage(varargin{:});
        otherwise
            error('nrmmConstantMotionFamily:invalidAction','Unknown family operation.');
    end
end

function family=localCondition(family,time,intervals,measurementTimes,centers,radii)
    validateattributes(intervals,{'double'},{'size',[2,4],'finite','real'});
    assert(all(intervals(1,:)<=intervals(2,:)), ...
        'nrmmConstantMotionFamily:emptyPrior','The parameter prior is empty.');
    if isempty(family)
        family=struct('kind',"constantNrmmCover-v1",'epochTime',time, ...
            'lower',intervals(1,:).','upper',intervals(2,:).', ...
            'scale',max(diff(intervals,1,1).',1e-10), ...
            'available',true,'lastTime',time,'checks',0,'cellLimit',512);
    end
    assert(time>=family.lastTime-1e-10,'nrmmConstantMotionFamily:timeOrder', ...
        'A parameter family cannot be updated backwards.');
    % Do not replace a retained joint cover by its coordinate-wise hull.
    age=time-family.epochTime;
    for pass=1:24
        lo=family.lower;hi=family.upper;keep=true(1,size(lo,2));
        % Current speed and constant parameters are further observations of
        % the original epoch variables, including the forward-stop extension.
        speedLow=max(0,lo(1,:)+age*lo(2,:));
        speedHigh=max(0,hi(1,:)+age*hi(2,:));
        keep=keep & speedHigh>=intervals(1,1) & speedLow<=intervals(2,1);
        moving=speedLow>0;
        keep=keep & (~moving | (hi(2,:)>=intervals(1,2) & lo(2,:)<=intervals(2,2)));
        keep=keep & hi(3,:)>=intervals(1,3) & lo(3,:)<=intervals(2,3);
        for j=1:numel(measurementTimes)
            image=localImage(family,measurementTimes(j),time,0);
            guard=512*eps(max(1,max(abs([centers(:,j);radii(:,j)]))));
            disjoint=any(image.upper<centers(:,j)-radii(:,j)-guard ...
                | image.lower>centers(:,j)+radii(:,j)+guard,1);
            % A pre-epoch stopped branch has no unique backward extension.
            % Such a branch cannot be rejected by this displacement test.
            keep=keep & (~disjoint | image.backwardUnavailable);
            family.checks=family.checks+size(lo,2);
        end
        family.lower=lo(:,keep);family.upper=hi(:,keep);
        if isempty(family.lower)
            family.available=false;
            error('nrmmConstantMotionFamily:inconsistentHistory', ...
                'No retained constant-motion parameter box fits the measurement bounds.');
        end
        count=size(family.lower,2);
        if 2*count>family.cellLimit || isempty(measurementTimes),break;end
        width=(family.upper-family.lower)./family.scale;
        [largest,coordinate]=max(width,[],1);
        split=find(largest>1/128);
        if isempty(split),break;end
        lower=family.lower(:,split);upper=family.upper(:,split);
        indices=sub2ind(size(lower),coordinate(split),1:numel(split));
        midpoint=(lower(indices)+upper(indices))/2;
        leftUpper=upper;leftUpper(indices)=midpoint;
        rightLower=lower;rightLower(indices)=midpoint;
        family.upper(:,split)=leftUpper;
        family.lower=[family.lower,rightLower];family.upper=[family.upper,upper];
    end
    family.lastTime=time;
end

function image=localImage(family,startTime,endTime,anglePadding)
    if nargin<4,anglePadding=0;end
    assert(endTime>=startTime,'nrmmConstantMotionFamily:timeOrder','Expected a forward interval.');
    lo=family.lower;hi=family.upper;t0=startTime-family.epochTime;t1=endTime-family.epochTime;
    backwardUnavailable=false(1,size(lo,2));
    if t0<0
        backwardUnavailable=lo(1,:)<=0 | lo(1,:)+t0*hi(2,:)<=0;
    end
    % Arc length is monotone in epoch V and A for nonnegative times. Before
    % the epoch use only wholly forward-moving boxes; unresolved stopped
    % prehistories are explicitly flagged above.
    if t0>=0
        s0lo=localArc(lo(1,:),lo(2,:),t0);s0hi=localArc(hi(1,:),hi(2,:),t0);
        dslo=localArc(lo(1,:),lo(2,:),t1)-s0lo;
        dshi=localArc(hi(1,:),hi(2,:),t1)-s0hi;
    else
        s0lo=t0*hi(1,:)+t0^2*lo(2,:)/2;
        s0hi=t0*lo(1,:)+t0^2*hi(2,:)/2;
        duration=t1-t0;coefficient=(t1^2-t0^2)/2;
        dslo=duration*lo(1,:)+min(coefficient*lo(2,:),coefficient*hi(2,:));
        dshi=duration*hi(1,:)+max(coefficient*lo(2,:),coefficient*hi(2,:));
        if t1>=0
            mayStop=lo(1,:)+lo(2,:)*t1<0;
            dslo(mayStop)=localArc(lo(1,mayStop),lo(2,mayStop),t1)-s0hi(mayStop);
            dshi(mayStop)=localArc(hi(1,mayStop),hi(2,mayStop),t1)-s0lo(mayStop);
        end
    end
    [turnLo,turnHi]=localProduct(lo(3,:),hi(3,:),dslo/2,dshi/2);
    far=max(abs(turnLo),abs(turnHi));near=max(0,max(turnLo,-turnHi));
    sincLo=-ones(size(far));sincHi=ones(size(far));inside=far<=pi;
    sincLo(inside)=localSinc(far(inside));sincHi(inside)=localSinc(near(inside));
    [lengthLo,lengthHi]=localProduct(dslo,dshi,sincLo,sincHi);
    [phaseLo,phaseHi]=localProduct(lo(3,:),hi(3,:),s0lo+dslo/2,s0hi+dshi/2);
    angleLo=lo(4,:)+phaseLo-anglePadding;angleHi=hi(4,:)+phaseHi+anglePadding;
    [cosLo,cosHi]=localCos(angleLo,angleHi);[sinLo,sinHi]=localCos(angleLo-pi/2,angleHi-pi/2);
    [xl,xh]=localProduct(lengthLo,lengthHi,cosLo,cosHi);
    [yl,yh]=localProduct(lengthLo,lengthHi,sinLo,sinHi);
    [phaseLo,phaseHi]=localProduct(lo(3,:),hi(3,:),s0lo+dslo,s0hi+dshi);
    image=struct('lower',[xl;yl],'upper',[xh;yh], ...
        'courseLower',lo(4,:)+phaseLo-anglePadding, ...
        'courseUpper',hi(4,:)+phaseHi+anglePadding, ...
        'curvatureLower',lo(3,:),'curvatureUpper',hi(3,:), ...
        'backwardUnavailable',backwardUnavailable);
    guard=512*eps(max(1,max(abs([image.lower;image.upper]),[],1)));
    image.lower=image.lower-guard;image.upper=image.upper+guard;
end

function arc=localArc(speed,acceleration,time)
    duration=repmat(time,size(speed));braking=acceleration<0;
    duration(braking)=min(time,-speed(braking)./acceleration(braking));
    arc=speed.*duration+acceleration.*duration.^2/2;
end

function value=localSinc(angle)
    value=ones(size(angle));nonzero=angle~=0;value(nonzero)=sin(angle(nonzero))./angle(nonzero);
end

function [lo,hi]=localProduct(al,ah,bl,bh)
    products=[al.*bl;al.*bh;ah.*bl;ah.*bh];lo=min(products,[],1);hi=max(products,[],1);
end

function [lo,hi]=localCos(a,b)
    lo=min(cos(a),cos(b));hi=max(cos(a),cos(b));
    hi(ceil(a/(2*pi))<=floor(b/(2*pi)))=1;
    lo(ceil((a-pi)/(2*pi))<=floor((b-pi)/(2*pi)))=-1;
end
