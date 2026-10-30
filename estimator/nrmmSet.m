classdef nrmmSet
    %nrmmSet Correlated first-order enclosure used by joint prediction.
    % A value is c + G*z + [-r,r], |z|<=1. Nonlinear operations enclose
    % their complete input ranges, not a finite selection of perturbations.
    % The inclusion proofs are in real arithmetic. MATLAB elementary
    % functions are not a directed-rounding interval library; machine-level
    % certification is deliberately not claimed.
    properties
        center
        generator
        remainder
    end
    methods
        function b=part(a,varargin)
            % Explicit dispatch is needed inside this class's own methods.
            b=subsref(a,substruct('()',varargin));
        end
        function a=nrmmSet(c,g,r)
            if nargin==0,c=[];end
            if nargin<2,g=zeros(numel(c),0);end
            if nargin<3,r=zeros(size(c));end
            assert(size(g,1)==numel(c) && isequal(size(r),size(c)) && all(r(:)>=0));
            assert(isreal(c) && isreal(g) && isreal(r) && all(isfinite([c(:);g(:);r(:)])), ...
                'nrmmSet:unboundedImage','The enclosure must remain finite and real.');
            if any(r(:)>0)
                indices=find(r(:)>0);columns=nrmmSet.symbolPool('allocate',numel(indices));
                if ~isempty(columns)
                    g(:,end+1:columns(end))=0;
                    g(sub2ind(size(g),indices,columns(:)))=r(indices);r(:)=0;
                end
            end
            a.center=c;a.generator=g;a.remainder=r;
        end
        function r=radius(a)
            r=reshape(sum(abs(a.generator),2),size(a.center))+a.remainder;
        end
        function [lo,hi]=bounds(a)
            r=radius(a);lo=a.center-r;hi=a.center+r;
        end
        function a=promote(a)
            % Exact reparameterization of the residual box. Its new signed
            % columns can then be contracted by later linear feedback maps.
            keep=find(a.remainder(:)>0);columns=nrmmSet.symbolPool('allocate',numel(keep));
            if isempty(columns),columns=size(a.generator,2)+(1:numel(keep));end
            if ~isempty(columns)
                a.generator(:,end+1:columns(end))=0;
                a.generator(sub2ind(size(a.generator),keep,columns(:)))=a.remainder(keep);
            end
            a.remainder(:)=0;
        end
        function a=compress(a,fraction,transform,protectedRows)
            % Outer reduction of negligible residual directions. The total
            % discarded support in EACH transformed coordinate is at most
            % fraction of its current support; there is no forced order cap.
            % Persistent physical-parameter columns remain explicit.
            a=promote(a);g=a.generator;n=size(g,1);
            if nargin<3,transform=eye(n);end
            if nargin<4,protectedRows=[];end
            assert(fraction>=0 && fraction<1);
            mapped=transform*g;scale=max(sum(abs(mapped),2),realmin);
            protected=any(g(protectedRows,:)~=0,1);
            budget=fraction*scale;used=zeros(n,1);
            indices=find(~protected & any(g,1));
            if numel(indices)>n
                % Near-parallel columns describe the same shared motion.
                % Aggregate their signed common direction and retain the
                % COMPLETE discrepancy as a box, instead of boxing their
                % much larger absolute contributions independently.
                normalized=mapped(:,indices)./scale;
                [mass,pivot]=max(abs(normalized),[],1);
                orientation=sign(normalized(sub2ind(size(normalized),pivot,1:numel(indices))));
                key=round(normalized./(mass.*orientation)/.01).';
                [~,~,group]=unique([pivot.',key],'rows');
                count=accumarray(group,1);
                membership=sparse(1:numel(group),group,1,numel(group),numel(count));
                total=(mapped(:,indices).*orientation)*membership;
                groupMass=accumarray(group,mass.');
                discrepancy=abs(mapped(:,indices).*orientation ...
                    -total(:,group).*(mass./groupMass(group).'))*membership;
                score=sum(discrepancy./scale,1).'./max(1,count-1);
                [~,order]=sort(score);order=order(count(order)>1);
                selected=false(numel(count),1);
                for j=order.'
                    if all(used+discrepancy(:,j)<=budget/2)
                        selected(j)=true;used=used+discrepancy(:,j);
                    end
                end
                keep=true(1,size(g,2));keep(indices(selected(group)))=false;
                g=[g(:,keep),transform\total(:,selected)];
                mapped=transform*g;
            end
            % Spend only the remaining support budget on negligible terms.
            protected=any(g(protectedRows,:)~=0,1);
            weight=abs(mapped)./scale;
            candidates=find(~protected);[~,order]=sort(sum(weight(:,candidates),1),'ascend');
            indices=candidates(order);accumulated=cumsum(weight(:,indices),2);
            count=find(all(accumulated<=(budget-used)./scale,1),1,'last');
            if isempty(count) || count<=n,count=0;end
            keep=true(1,size(g,2));keep(indices(1:count))=false;
            box=used+sum(abs(mapped(:,~keep)),2);active=box>0;
            remainder=diag(box);remainder=remainder(:,active);
            a.generator=[g(:,keep),transform\remainder];
        end
        function varargout=size(a,varargin)
            [varargout{1:nargout}]=size(a.center,varargin{:});
        end
        function a=reshape(a,varargin)
            a.center=reshape(a.center,varargin{:});a.remainder=reshape(a.remainder,varargin{:});
        end
        function b=subsref(a,s)
            if strcmp(s(1).type,'()')
                index=reshape(1:numel(a.center),size(a.center));index=index(s(1).subs{:});
                b=nrmmSet(a.center(s(1).subs{:}),a.generator(index(:),:),a.remainder(s(1).subs{:}));
                if numel(s)>1,b=subsref(b,s(2:end));end
            else
                b=builtin('subsref',a,s);
            end
        end
        function a=subsasgn(a,s,b)
            if strcmp(s(1).type,'()')
                b=nrmmSet.convert(b);[a,b]=nrmmSet.columns(a,b);
                index=reshape(1:numel(a.center),size(a.center));index=index(s(1).subs{:});
                a.center(s(1).subs{:})=b.center;a.remainder(s(1).subs{:})=b.remainder;
                a.generator(index(:),:)=b.generator;
            else
                a=builtin('subsasgn',a,s,b);
            end
        end
        function c=plus(a,b)
            [a,b]=nrmmSet.expand(a,b);
            c=nrmmSet(a.center+b.center,a.generator+b.generator,a.remainder+b.remainder);
        end
        function c=minus(a,b)
            c=plus(a,uminus(nrmmSet.convert(b)));
        end
        function a=uminus(a)
            a.center=-a.center;a.generator=-a.generator;
        end
        function c=times(a,b)
            if ~isa(a,'nrmmSet') && isscalar(a) && isa(b,'nrmmSet')
                c=nrmmSet(a*b.center,a*b.generator,abs(a)*b.remainder);return;
            elseif ~isa(b,'nrmmSet') && isscalar(b) && isa(a,'nrmmSet')
                c=nrmmSet(b*a.center,b*a.generator,abs(b)*a.remainder);return;
            end
            [a,b]=nrmmSet.expand(a,b);
            g=b.center(:).*a.generator+a.center(:).*b.generator;
            shift=zeros(size(a.center));
            r=abs(b.center).*a.remainder+abs(a.center).*b.remainder;
            % The two deviations form a TWO-dimensional zonotope even
            % when hundreds of primitive columns are shared. The extrema
            % of their product lie on its polygon edges. Bound this single
            % correlated quadratic instead of adding independent |z_i z_j|
            % terms; every primitive value remains included.
            active=find(radius(a)>0 & radius(b)>0);
            for k=active(:).'
                [lo,hi]=nrmmSet.productRange(a.generator(k,:),b.generator(k,:),a.remainder(k),b.remainder(k));
                shift(k)=(lo+hi)/2;
                r(k)=abs(b.center(k))*a.remainder(k)+abs(a.center(k))*b.remainder(k)+(hi-lo)/2;
            end
            c=nrmmSet(a.center.*b.center+shift,g,max(0,r));
        end
        function c=mtimes(a,b)
            % A deterministic linear map acts directly on the shared
            % columns. Expanding it into scalar affine products adds no
            % information and needlessly rebuilds thousands of constants.
            if ~isa(a,'nrmmSet') && isa(b,'nrmmSet') && ~isscalar(b.center)
                if isscalar(a),c=times(a,b);return;end
                map=kron(eye(size(b.center,2)),a);
                c=nrmmSet(a*b.center,map*b.generator,abs(a)*b.remainder);return;
            elseif isa(a,'nrmmSet') && ~isa(b,'nrmmSet') && ~isscalar(a.center)
                if isscalar(b),c=times(a,b);return;end
                map=kron(b.',eye(size(a.center,1)));
                c=nrmmSet(a.center*b,map*a.generator,a.remainder*abs(b));return;
            end
            a=nrmmSet.convert(a);b=nrmmSet.convert(b);
            if isscalar(a.center)||isscalar(b.center),c=times(a,b);return;end
            assert(size(a.center,2)==size(b.center,1));
            c=nrmmSet(zeros(size(a.center,1),size(b.center,2)));
            for j=1:size(b.center,2)
                for i=1:size(a.center,1)
                    value=nrmmSet(0);
                        for k=1:size(a.center,2),value=value+part(a,i,k).*part(b,k,j);end
                    c=subsasgn(c,substruct('()',{i,j}),value);
                end
            end
        end
        function c=rdivide(a,b)
            if isa(a,'nrmmSet') && ~isa(b,'nrmmSet') && isscalar(b)
                if b==0,error('nrmmSet:singularDomain','A reciprocal interval contains zero.');end
                c=nrmmSet(a.center/b,a.generator/b,a.remainder/abs(b));return;
            end
            [a,b]=nrmmSet.expand(a,b);[lo,hi]=bounds(b);
            if any(lo(:)<=0 & hi(:)>=0)
                error('nrmmSet:singularDomain','A reciprocal interval contains zero.');
            end
            slope=-1./(lo.*hi);
            critical=sign(lo).*sqrt(-1./slope);
            reciprocal=nrmmSet.affineImage(b,@(x)1./x,slope,critical(:));
            c=times(a,reciprocal);
            % Cancel the shared part of numerator and denominator before
            % bounding their product. Select between two sound images.
            ratio=a.center./b.center;
            centered=ratio+times(a-ratio.*b,reciprocal);
            use=radius(centered)<radius(c);
            if any(use,'all'),c=subsasgn(c,substruct('()',{use}),part(centered,use));end
        end
        function c=mrdivide(a,b)
            if ~isa(b,'nrmmSet'),assert(isscalar(b));c=rdivide(a,b);return;end
            b=nrmmSet.convert(b);assert(isscalar(b.center));c=rdivide(a,b);
        end
        function c=power(a,k)
            assert(isscalar(k)&&k>=0&&k==floor(k));
            if all(radius(a)==0,'all')
                % Evaluate a point once. Repeated multiplication and .^ can
                % differ by rounding and falsely give disjoint singleton ranges.
                c=nrmmSet(a.center.^k,zeros(size(a.generator)));return;
            end
            if k==0,c=nrmmSet(ones(size(a.center)),zeros(size(a.generator)));return;end
            if k==1,c=a;return;end
            fun=@(x)x.^k;slope=nrmmSet.secantSlope(a,fun,k*a.center.^(k-1));
            critical=sign(slope).*abs(slope/k).^(1/(k-1));
            points=critical(:);
            if mod(k,2)==1,points=[points,-points];end
            c=nrmmSet.affineImage(a,fun,slope,points);
        end
        function c=mpower(a,k)
            assert(isscalar(a.center));c=power(a,k);
        end
        function b=sin(a)
            slope=nrmmSet.secantSlope(a,@sin,cos(a.center));
            phase=acos(min(1,max(-1,slope(:))));
            points=nrmmSet.periodicCritical(a,[phase,-phase],2*pi);
            b=nrmmSet.affineImage(a,@sin,slope,points);
        end
        function b=cos(a)
            slope=nrmmSet.secantSlope(a,@cos,-sin(a.center));
            phase=asin(min(1,max(-1,-slope(:))));
            points=nrmmSet.periodicCritical(a,[phase,pi-phase],2*pi);
            b=nrmmSet.affineImage(a,@cos,slope,points);
        end
        function b=tan(a)
            [lo,hi]=bounds(a);
            if any(floor((lo(:)+pi/2)/pi)~=floor((hi(:)+pi/2)/pi)) ...
                    || any(mod(lo(:)+pi/2,pi)==0 | mod(hi(:)+pi/2,pi)==0)
                error('nrmmSet:singularDomain','A tangent interval crosses a pole.');
            end
            slope=nrmmSet.secantSlope(a,@tan,1+tan(a.center).^2);
            phase=atan(sqrt(max(0,slope(:)-1)));
            points=nrmmSet.periodicCritical(a,[phase,-phase],pi);
            b=nrmmSet.affineImage(a,@tan,slope,points);
        end
        function b=atan(a)
            slope=nrmmSet.secantSlope(a,@atan,1./(1+a.center.^2));
            critical=sqrt(max(0,1./slope-1));
            b=nrmmSet.affineImage(a,@atan,slope,[critical(:),-critical(:)]);
        end
        function b=asin(a)
            [lo,hi]=bounds(a);
            if any(lo(:)<-1 | hi(:)>1),error('nrmmSet:singularDomain','An arcsine interval leaves [-1,1].');end
            slope=nrmmSet.secantSlope(a,@asin,1./sqrt(max(realmin,1-a.center.^2)));
            critical=sqrt(max(0,1-1./slope.^2));
            b=nrmmSet.affineImage(a,@asin,slope,[critical(:),-critical(:)]);
        end
        function b=tanh(a)
            slope=nrmmSet.secantSlope(a,@tanh,1-tanh(a.center).^2);
            critical=atanh(sqrt(min(1,max(0,1-slope))));
            b=nrmmSet.affineImage(a,@tanh,slope,[critical(:),-critical(:)]);
        end
        function b=smoothstep(a)
            % One full-domain image of the clamped cubic transition. Keeping
            % its repeated speed argument avoids independent uncertainty in
            % clamp(s), s^2 and 3-2s at the Fiala/kinematic transition.
            fun=@(x)min(1,max(0,x)).^2.*(3-2*min(1,max(0,x)));
            t=min(1,max(0,a.center));slope=nrmmSet.secantSlope(a,fun,6*t.*(1-t));
            offset=sqrt(max(0,1-2*slope(:)/3));
            points=[(1-offset)/2,(1+offset)/2,zeros(numel(t),1),ones(numel(t),1)];
            b=nrmmSet.affineImage(a,fun,slope,points);
        end
        function b=fialaFraction(a)
            % P(q)=3q-3|q|q+q^3 on [-1,1], sign(q) outside.
            % P is C1 and P' is globally 6-Lipschitz, including q=0 and
            % both saturation transitions. Taylor's integral bound applies
            % without assuming a nonexistent classical Hessian at q=0.
            fun=@nrmmSet.fialaValue;
            slope=nrmmSet.secantSlope(a,fun,3*max(0,1-abs(a.center)).^2);
            critical=1-sqrt(max(0,slope)/3);count=numel(a.center);
            % Extremizers of P(q)-slope*q are endpoints, joins, and the
            % roots P'(q)=slope. This covers the complete interval exactly
            % in real arithmetic, including both saturation transitions.
            points=[critical(:),-critical(:),zeros(count,1),ones(count,1),-ones(count,1)];
            b=nrmmSet.affineImage(a,fun,slope,points);
        end
        function b=sinc(a)
            % Unnormalized sinc = integral_0^1 cos(t*x) dt; |sinc''|<=1/3.
            x=a.center;v=ones(size(x));s=zeros(size(x));large=abs(x)>1e-3;
            v(large)=sin(x(large))./x(large);
            s(large)=(x(large).*cos(x(large))-sin(x(large)))./x(large).^2;
            z=x(~large);v(~large)=1-z.^2/6+z.^4/120-z.^6/5040;
            s(~large)=-z/3+z.^3/30-z.^5/840;
            b=nrmmSet.unary(a,v,s,ones(size(x))/3);
            % Truncation of the center/slope series is enclosed as well.
            b.remainder(~large)=b.remainder(~large)+abs(z).^8/362880 ...
                +abs(z).^7/45360.*radius(part(a,~large));
        end
        function b=sqrt(a)
            [lo,hi]=bounds(a);
            if any(hi(:)<0),error('nrmmSet:singularDomain','Negative square-root domain.');end
            % Callers must establish the nonnegative domain, e.g. a square
            % or max(0,a). A Taylor radius can extend outside that domain.
            lo=max(0,lo);hi=max(0,hi);
            if any(bounds(a)<0,'all')
                b=nrmmSet.range(sqrt(lo),sqrt(hi),size(a.generator,2));return;
            end
            slope=zeros(size(lo));active=hi>0;
            slope(active)=1./(sqrt(lo(active))+sqrt(hi(active)));
            critical=zeros(size(lo));critical(active)=1./(4*slope(active).^2);
            b=nrmmSet.affineImage(a,@sqrt,slope,critical(:));
        end
        function b=abs(a)
            slope=nrmmSet.secantSlope(a,@abs,sign(a.center));
            b=nrmmSet.affineImage(a,@abs,slope,zeros(numel(a.center),1));
        end
        function c=max(a,b)
            [a,b]=nrmmSet.expand(a,b);[al,ah]=bounds(a);[bl,bh]=bounds(b);
            c=(a+b+abs(a-b))/2;
            for k=1:numel(c.center)
                if al(k)>=bh(k),c=subsasgn(c,substruct('()',{k}),part(a,k));
                elseif bl(k)>=ah(k),c=subsasgn(c,substruct('()',{k}),part(b,k));end
            end
            c=nrmmSet.intersectRange(c,max(al,bl),max(ah,bh));
        end
        function c=min(a,b)
            c=-max(-nrmmSet.convert(a),-nrmmSet.convert(b));
        end
        function b=sum(a)
            assert(isvector(a.center));b=nrmmSet(sum(a.center(:)),sum(a.generator,1),sum(a.remainder(:)));
        end
        function b=transpose(a)
            index=reshape(1:numel(a.center),size(a.center)).';
            b=nrmmSet(a.center.',a.generator(index(:),:),a.remainder.');
        end
        function b=ctranspose(a),b=transpose(a);end
        function c=vertcat(varargin),c=nrmmSet.concatenate(1,varargin{:});end
        function c=horzcat(varargin),c=nrmmSet.concatenate(2,varargin{:});end
    end
    methods (Static)
        function [lo,hi]=productRange(first,second,firstRemainder,secondRemainder)
            % Exact real-arithmetic range of x*y on the planar zonotope.
            % Opposite edges have identical products, so one half boundary
            % suffices. Include the stationary point on EVERY edge.
            g=[first,firstRemainder,0;second,0,secondRemainder];
            g=g(:,any(g,1));
            orientation=ones(1,size(g,2));orientation(g(2,:)<0 | (g(2,:)==0 & g(1,:)<0))=-1;
            g=g.*orientation;[~,order]=sort(atan2(g(2,:),g(1,:)));g=g(:,order);
            vertex=[-sum(g,2),-sum(g,2)+2*cumsum(g,2)];
            values=vertex(1,:).*vertex(2,:);
            start=vertex(:,1:end-1);step=2*g;
            quadratic=step(1,:).*step(2,:);
            linear=start(1,:).*step(2,:)+start(2,:).*step(1,:);
            t=-linear./(2*quadratic);inside=t>0 & t<1 & quadratic~=0;
            point=start(:,inside)+step(:,inside).*t(inside);
            values=[values,point(1,:).*point(2,:),0];
            lo=min(values);hi=max(values);
        end
        function result=symbolPool(action,value)
            % A hold-local pool gives each nonlinear residual ONE identity.
            % Reusing a computed force then reuses that same signed column.
            % Explicit noise/decision columns are reserved before entry.
            persistent state
            if isempty(state),state=struct('enabled',false,'next',0);end
            switch action
                case 'begin'
                    result=state;state=struct('enabled',true,'next',value);
                case 'pause'
                    result=state;state.enabled=false;
                case 'restore'
                    result=state;state=value;
                case 'allocate'
                    result=[];
                    if state.enabled,result=state.next+(1:value);state.next=state.next+value;end
            end
        end
        function a=convert(a)
            if ~isa(a,'nrmmSet'),a=nrmmSet(a);end
        end
        function a=range(lo,hi,columns)
            if nargin<3,columns=0;end
            assert(all(lo(:)<=hi(:)) && all(isfinite([lo(:);hi(:)])));
            a=nrmmSet((lo+hi)/2,zeros(numel(lo),columns),(hi-lo)/2);
        end
        function b=unary(a,value,slope,second)
            r=abs(slope).*a.remainder+.5*second.*radius(a).^2;
            b=nrmmSet(value,slope(:).*a.generator,r);
        end
        function slope=secantSlope(a,fun,slope)
            [lo,hi]=bounds(a);width=hi-lo;
            use=width>sqrt(eps)*max(1,abs(a.center));
            slope(use)=(fun(hi(use))-fun(lo(use)))./width(use);
        end
        function points=periodicCritical(a,phase,period)
            % Across periods the residual differs by -slope*period, so its
            % first and last stationary point of each phase suffice.
            [lo,hi]=bounds(a);
            points=[phase+period*ceil((lo(:)-phase)/period), ...
                phase+period*floor((hi(:)-phase)/period)];
        end
        function value=fialaValue(q)
            q=min(1,max(-1,q));value=3*q-3*abs(q).*q+q.^3;
        end
        function [lo,hi]=ratioRange(a,b)
            % Range of two correlated affine forms with positive denominator.
            % A linear-fractional function on this planar zonotope attains
            % its extrema at vertices. Enumerate its complete boundary,
            % including independent remainder axes; no sampling or iterative
            % root search is needed.
            [a,b]=nrmmSet.expand(a,b);[bl,~]=bounds(b);
            assert(all(bl>0,'all'),'nrmmSet:singularDomain','A positive denominator is required.');
            lo=zeros(size(a.center));hi=lo;
            for k=1:numel(lo)
                g=[a.generator(k,:),a.remainder(k),0;b.generator(k,:),0,b.remainder(k)];
                g=g(:,any(g,1));
                orientation=ones(1,size(g,2));orientation(g(2,:)<0 | (g(2,:)==0 & g(1,:)<0))=-1;
                g=g.*orientation;[~,order]=sort(atan2(g(2,:),g(1,:)));g=g(:,order);
                half=[-sum(g,2),-sum(g,2)+2*cumsum(g,2)];
                vertex=[a.center(k);b.center(k)]+[half,-half];
                values=vertex(1,:)./vertex(2,:);
                lo(k)=min(values);hi(k)=max(values);
            end
            guard=64*eps(max(1,max(abs(lo),abs(hi))));
            lo=lo-guard;hi=hi+guard;
        end
        function force=fialaForce(stress,capacity)
            % The perspective -c*P(s/(3*c)) is affine in (s,c) except
            % for the scalar residual P(q)-a*q. Cancel c analytically
            % before enclosing that residual; evaluating c, 1/c and c
            % separately loses their shared physical realization.
            [stress,capacity]=nrmmSet.expand(stress,capacity);
            [cl,ch]=bounds(capacity);
            assert(all(cl>0,'all'),'nrmmSet:singularDomain', ...
                'The perspective enclosure needs positive tire capacity.');
            % Do not obtain the ratio domain from an independent reciprocal
            % product: its artificial residual feeds back into every tire
            % image. Correlated affine supports give a tighter full range.
            [lo,hi]=nrmmSet.ratioRange(stress,3*capacity);
            q=nrmmSet.range(lo,hi,size(stress.generator,2));
            slope=nrmmSet.secantSlope(q,@nrmmSet.fialaValue,3*max(0,1-abs(q.center)).^2);
            critical=1-sqrt(max(0,slope)/3);count=numel(q.center);
            points=[lo(:),hi(:),critical(:),-critical(:),zeros(count,1),ones(count,1),-ones(count,1)];
            points=min(hi(:),max(lo(:),points));
            residual=nrmmSet.fialaValue(points)-slope(:).*points;
            low=min(residual,[],2);high=max(residual,[],2);
            midpoint=reshape((low+high)/2,size(q.center));
            error=ch.*reshape((high-low)/2,size(q.center));
            a=-slope/3;b=-midpoint;
            [residualLower,residualUpper]=nrmmSet.perspectiveResidual(stress,capacity,a,b);
            residualLower=max(residualLower,-error);residualUpper=min(residualUpper,error);
            force=a.*stress+b.*capacity ...
                +nrmmSet.range(residualLower,residualUpper,size(q.generator,2));
        end
        function [lower,upper]=perspectiveResidual(stress,capacity,a,b)
            % Enclose F(s,c)-a*s-b*c on the COMPLETE planar zonotope.
            % Homogeneity makes every differentiable interior stationary
            % value zero. On an edge, split at q=s/(3c)=-1,0,1. The residual
            % is convex or concave on each resulting segment, so a change
            % of derivative sign identifies its only interior extremum.
            % Bisection supplies a bracket, not an assumed exact root:
            % |dF/ds|<=1 and |dF/dc|<=1 bound the omitted bracket variation.
            lower=zeros(size(a));upper=lower;
            for row=1:numel(a)
                g=[stress.generator(row,:),stress.remainder(row),0; ...
                    capacity.generator(row,:),0,capacity.remainder(row)];
                g=g(:,any(g,1));orientation=ones(1,size(g,2));
                orientation(g(2,:)<0 | (g(2,:)==0 & g(1,:)<0))=-1;
                g=g.*orientation;[~,order]=sort(atan2(g(2,:),g(1,:)));g=g(:,order);
                half=[-sum(g,2),-sum(g,2)+2*cumsum(g,2)];
                vertex=[stress.center(row);capacity.center(row)]+[half,-half];
                vertex=[vertex,vertex(:,1)];base=vertex(:,1:end-1);edge=diff(vertex,1,2);
                cuts=[zeros(1,size(edge,2));ones(1,size(edge,2))];
                for q=[-1,0,1]
                    t=(3*q*base(2,:)-base(1,:))./(edge(1,:)-3*q*edge(2,:));
                    t(~isfinite(t))=0;cuts(end+1,:)=min(1,max(0,t));
                end
                cuts=sort(cuts,1);left=cuts(1:end-1,:);right=cuts(2:end,:);
                sl=base(1,:)+edge(1,:).*left;cl=base(2,:)+edge(2,:).*left;
                sr=base(1,:)+edge(1,:).*right;cr=base(2,:)+edge(2,:).*right;
                vl=-cl.*nrmmSet.fialaValue(sl./(3*cl))-a(row)*sl-b(row)*cl;
                vr=-cr.*nrmmSet.fialaValue(sr./(3*cr))-a(row)*sr-b(row)*cr;
                low=min([vl(:);vr(:);0]);high=max([vl(:);vr(:);0]);
                ql=sl./(3*cl);qr=sr./(3*cr);
                pl=nrmmSet.fialaValue(ql);pr=nrmmSet.fialaValue(qr);
                dl=3*max(0,1-abs(ql)).^2;dr=3*max(0,1-abs(qr)).^2;
                fl=(-dl/3-a(row)).*edge(1,:)+(-pl+ql.*dl-b(row)).*edge(2,:);
                fr=(-dr/3-a(row)).*edge(1,:)+(-pr+qr.*dr-b(row)).*edge(2,:);
                active=fl.*fr<0 & right>left;
                [~,column]=find(active);column=column.';
                tl=left(active).';tr=right(active).';fs=sign(fl(active).');
                c0=base(2,column);s0=base(1,column);dc=edge(2,column);ds=edge(1,column);
                for iteration=1:24
                    tm=(tl+tr)/2;cm=c0+dc.*tm;sm=s0+ds.*tm;qm=sm./(3*cm);
                    pm=nrmmSet.fialaValue(qm);dm=3*max(0,1-abs(qm)).^2;
                    fm=(-dm/3-a(row)).*ds+(-pm+qm.*dm-b(row)).*dc;
                    same=sign(fm)==fs;tl(same)=tm(same);tr(~same)=tm(~same);
                end
                tm=(tl+tr)/2;cm=c0+dc.*tm;sm=s0+ds.*tm;
                vm=-cm.*nrmmSet.fialaValue(sm./(3*cm))-a(row)*sm-b(row)*cm;
                allowance=((1+abs(a(row)))*abs(ds)+(1+abs(b(row)))*abs(dc)).*(tr-tl)/2;
                lower(row)=min([low,vm-allowance]);upper(row)=max([high,vm+allowance]);
            end
        end
        function b=affineImage(a,fun,slope,critical)
            % Bound f(x)-f(c)-slope*(x-c) at ALL of its analytic extrema.
            % Callers supply every interior stationary point and branch join.
            % Keep the signed affine dependence; only the residual is new.
            [lo,hi]=bounds(a);points=[lo(:),hi(:),a.center(:),critical];
            points=min(hi(:),max(lo(:),points));value=fun(a.center);
            residual=fun(points)-value(:)-slope(:).*(points-a.center(:));
            lower=min(residual,[],2);upper=max(residual,[],2);
            shift=reshape((lower+upper)/2,size(value));
            rest=reshape((upper-lower)/2,size(value));
            b=nrmmSet(value+shift,slope(:).*a.generator,abs(slope).*a.remainder+rest);
        end
        function a=intersectRange(a,lo,hi)
            % Intersection can shrink an outer interval, but a changed
            % interval is represented independently, never by scaling G.
            [al,ah]=bounds(a);
            for k=1:numel(al)
                l=max(al(k),lo(k));h=min(ah(k),hi(k));
                if l>h,error('nrmmSet:emptyIntersection','Disjoint certified ranges.');end
                % Losing a signed generator to save an infinitesimal
                % interval width destroys closed-loop cancellation. Keep
                % the affine image unless the range contraction is useful
                % or establishes a sign needed by a subsequent chart.
                useful=(h-l)<.8*(ah(k)-al(k)) || (al(k)<=0 && l>0) || (ah(k)>=0 && h<0);
                if useful,a=subsasgn(a,substruct('()',{k}),nrmmSet.range(l,h,size(a.generator,2)));end
            end
        end
        function [a,b]=columns(a,b)
            n=max(size(a.generator,2),size(b.generator,2));
            a.generator(:,end+1:n)=0;b.generator(:,end+1:n)=0;
        end
        function [a,b]=expand(a,b)
            a=nrmmSet.convert(a);b=nrmmSet.convert(b);[a,b]=nrmmSet.columns(a,b);
            if isscalar(a.center)&&~isscalar(b.center)
                a=nrmmSet(repmat(a.center,size(b.center)),repmat(a.generator,numel(b.center),1),repmat(a.remainder,size(b.center)));
            elseif isscalar(b.center)&&~isscalar(a.center)
                b=nrmmSet(repmat(b.center,size(a.center)),repmat(b.generator,numel(a.center),1),repmat(b.remainder,size(a.center)));
            end
            assert(isequal(size(a.center),size(b.center)));
        end
        function c=concatenate(dim,varargin)
            items=cellfun(@nrmmSet.convert,varargin,'UniformOutput',false);
            n=max(cellfun(@(x)size(x.generator,2),items));
            values=cell(size(items));errors=values;indices=values;g=[];offset=0;
            for k=1:numel(items)
                a=items{k};a.generator(:,end+1:n)=0;
                values{k}=a.center;errors{k}=a.remainder;
                indices{k}=reshape(offset+(1:numel(a.center)),size(a.center));
                g=[g;a.generator];offset=offset+numel(a.center); %#ok<AGROW>
            end
            index=cat(dim,indices{:});c=nrmmSet(cat(dim,values{:}),g(index(:),:),cat(dim,errors{:}));
        end
    end
end
