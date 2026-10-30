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
            keep=find(a.remainder(:)>0);extra=zeros(numel(a.center),numel(keep));
            extra(sub2ind(size(extra),keep,(1:numel(keep)).'))=a.remainder(keep);
            a.generator=[a.generator,extra];a.remainder(:)=0;
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
            [a,b]=nrmmSet.expand(a,b);
            g=b.center(:).*a.generator+a.center(:).*b.generator;
            r=abs(b.center).*a.remainder+abs(a.center).*b.remainder+radius(a).*radius(b);
            c=nrmmSet(a.center.*b.center,g,r);
        end
        function c=mtimes(a,b)
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
            b=nrmmSet.convert(b);[lo,hi]=bounds(b);
            if any(lo(:)<=0 & hi(:)>=0)
                error('nrmmSet:singularDomain','A reciprocal interval contains zero.');
            end
            second=2./min(abs(lo),abs(hi)).^3;
            reciprocal=nrmmSet.unary(b,1./b.center,-1./b.center.^2,second);
            reciprocal=nrmmSet.intersectRange(reciprocal,1./hi,1./lo);
            c=times(a,reciprocal);
        end
        function c=mrdivide(a,b)
            b=nrmmSet.convert(b);assert(isscalar(b.center));c=rdivide(a,b);
        end
        function c=power(a,k)
            assert(isscalar(k)&&k>=0&&k==floor(k));
            if all(radius(a)==0,'all')
                % Evaluate a point once. Repeated multiplication and .^ can
                % differ by rounding and falsely give disjoint singleton ranges.
                c=nrmmSet(a.center.^k,zeros(size(a.generator)));return;
            end
            c=nrmmSet(ones(size(a.center)));
            for i=1:k,c=c.*a;end
            if k>0 && mod(k,2)==0
                [lo,hi]=bounds(a);lower=min(abs(lo),abs(hi)).^k;
                lower(lo<=0 & hi>=0)=0;
                c=nrmmSet.intersectRange(c,lower,max(abs(lo),abs(hi)).^k);
            end
        end
        function c=mpower(a,k)
            assert(isscalar(a.center));c=power(a,k);
        end
        function b=sin(a)
            b=nrmmSet.unary(a,sin(a.center),cos(a.center),ones(size(a.center)));
            b=nrmmSet.intersectRange(b,-ones(size(a.center)),ones(size(a.center)));
        end
        function b=cos(a)
            b=nrmmSet.unary(a,cos(a.center),-sin(a.center),ones(size(a.center)));
            b=nrmmSet.intersectRange(b,-ones(size(a.center)),ones(size(a.center)));
        end
        function b=tan(a)
            [lo,hi]=bounds(a);
            if any(floor((lo(:)+pi/2)/pi)~=floor((hi(:)+pi/2)/pi)) ...
                    || any(mod(lo(:)+pi/2,pi)==0 | mod(hi(:)+pi/2,pi)==0)
                error('nrmmSet:singularDomain','A tangent interval crosses a pole.');
            end
            t=max(abs(tan(lo)),abs(tan(hi)));
            b=nrmmSet.unary(a,tan(a.center),1+tan(a.center).^2,2*t.*(1+t.^2));
        end
        function b=atan(a)
            b=nrmmSet.unary(a,atan(a.center),1./(1+a.center.^2),ones(size(a.center)));
            [lo,hi]=bounds(a);b=nrmmSet.intersectRange(b,atan(lo),atan(hi));
        end
        function b=tanh(a)
            t=tanh(a.center);b=nrmmSet.unary(a,t,1-t.^2,2*ones(size(a.center)));
            b=nrmmSet.intersectRange(b,-ones(size(a.center)),ones(size(a.center)));
        end
        function b=fialaFraction(a)
            % P(q)=3q-3|q|q+q^3 on [-1,1], sign(q) outside.
            % P is C1 and P' is globally 6-Lipschitz, including q=0 and
            % both saturation transitions. Taylor's integral bound applies
            % without assuming a nonexistent classical Hessian at q=0.
            q=min(1,max(-1,a.center));value=3*q-3*abs(q).*q+q.^3;
            slope=3*max(0,1-abs(a.center)).^2;
            [lo,hi]=bounds(a);minimumMagnitude=min(abs(lo),abs(hi));
            minimumMagnitude(lo<=0 & hi>=0)=0;
            % On this complete interval, |P''| <= 6(1-min|q|)_+ a.e.
            % A wholly saturated interval is exactly constant, with no
            % artificial remainder or dependence on its slip variables.
            second=6*max(0,1-minimumMagnitude);
            b=nrmmSet.unary(a,value,slope,second);
            lower=min(1,max(-1,lo));upper=min(1,max(-1,hi));
            lower=3*lower-3*abs(lower).*lower+lower.^3;
            upper=3*upper-3*abs(upper).*upper+upper.^3;
            b=nrmmSet.intersectRange(b,lower,upper);
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
            if any(lo(:)==0)
                b=nrmmSet.range(sqrt(lo),sqrt(hi),size(a.generator,2));return;
            end
            b=nrmmSet.unary(a,sqrt(a.center),.5./sqrt(a.center),.25./lo.^1.5);
            b=nrmmSet.intersectRange(b,sqrt(lo),sqrt(hi));
        end
        function b=abs(a)
            b=a;
            for k=1:numel(a.center)
                v=part(a,k);[lo,hi]=bounds(v);
                if lo>=0,continue;
                elseif hi<=0,b=subsasgn(b,substruct('()',{k}),-v);
                elseif v.center>=0
                    v.center=v.center-lo;v.remainder=v.remainder-lo;b=subsasgn(b,substruct('()',{k}),v);
                else
                    v=-v;v.center=v.center+hi;v.remainder=v.remainder+hi;b=subsasgn(b,substruct('()',{k}),v);
                end
            end
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
