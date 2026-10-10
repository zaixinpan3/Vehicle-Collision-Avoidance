function out = polytopeCeiling(resultFile,label,options)
%polytopeCeiling Largest contractive polytope against the largest ellipsoid on one Jacobian vertex set.
% Reads the result of alternativeCertificate with the given label (common
% Lyapunov function) from the JSON-lines file, rebuilds the zonotope vertices
% of its final enclosure (scaled coordinates) and its gain K, drops the
% residual (so both results are ceilings, not certificates), and compares
%   ellipsoid  the largest-fill ellipsoid S of the LMI with K fixed, eps = 0;
%   polytope   the maximal lambda-contractive polytope inside the box and the
%              input rows under u = K e for the same vertices (Blanchini's
%              set recursion X+ = X intersect {x : M_v x in lambda X} until
%              it stops changing), lambda = sqrt(design contraction).
% Extents of both are reported in physical units.
    arguments
        resultFile (1,1) string
        label (1,1) string
        options.MaxIterations (1,1) double = 80
        options.Tolerance (1,1) double = 1e-7
        options.Verbose (1,1) logical = true
    end
    root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    if isempty(which('sdpvar')),addpath(genpath(fullfile(root,'solver','YALMIP')));end
    if isempty(which('sedumi')),addpath(genpath(fullfile(root,'solver','sedumi')));end
    lines=splitlines(string(fileread(resultFile)));lines=lines(strlength(lines)>0);
    result=[];
    for line=lines.'
        r=jsondecode(line);
        if string(r.label)==label,result=r;break;end
    end
    assert(~isempty(result),'polytopeCeiling:label','No result with label %s.',label);
    k=reshape(result.scaledGain,2,5);centre=reshape(result.scaledCentre,5,7);
    directions=reshape(result.enclosureDirections,5,7,[]);m=size(directions,3);
    signs=(dec2bin(0:2^m-1)-'0')*2-1;closed=zeros(5,5,2^m);
    for v=1:2^m
        ab=centre;for index=1:m,ab=ab+signs(v,index)*directions(:,:,index);end
        closed(:,:,v)=ab(:,1:5)+ab(:,6:7)*k;
    end
    box=result.box(:);h=.05;T=result.requiredTimeConstantSeconds;rho=exp(-2*h/T)*(1-1e-3);lambda=sqrt(rho);
    % Ellipsoid ceiling: LMI with K fixed and no residual.
    s=sdpvar(5,5);t=sdpvar(1);y=k*s;constraints=[s>=t*eye(5),t>=0];
    for i=1:5,constraints=[constraints,s(i,i)<=1];end %#ok<AGROW>
    for j=1:2,constraints=[constraints,[1,y(j,:);y(j,:).',s]>=0];end %#ok<AGROW>
    for v=1:2^m,constraints=[constraints,[rho*s,(closed(:,:,v)*s).';closed(:,:,v)*s,s]>=0];end %#ok<AGROW>
    diagnostics=optimize(constraints,-t,sdpsettings('solver','sedumi','verbose',0));
    ellipsoid=sqrt(diag(value(s))).*box;
    if options.Verbose,fprintf('%s: ellipsoid ceiling (K fixed, no residual): ok %d, extents %s\n',label,diagnostics.problem==0,mat2str(ellipsoid.',3));end
    % Polytope: rows h' x <= 1.
    rows=[eye(5);-eye(5);k;-k];
    lp=optimoptions('linprog','Display','off','Algorithm','dual-simplex');
    converged=false;
    for iteration=1:options.MaxIterations
        candidates=rows;
        for v=1:2^m,candidates=[candidates;rows*closed(:,:,v)/lambda];end %#ok<AGROW>
        candidates=unique(round(candidates,10),'rows','stable');
        kept=localIrredundant(candidates,lp,options.Tolerance);
        extents=localExtents(kept,lp).*box;
        if options.Verbose,fprintf('  iteration %d: %d candidate rows, %d kept, extents %s\n',iteration,size(candidates,1),size(kept,1),mat2str(extents.',3));end
        % Converged when every candidate from the kept rows is redundant.
        if size(kept,1)==size(rows,1) && localContained(kept,rows,lp,options.Tolerance)
            converged=true;rows=kept;break;
        end
        rows=kept;
    end
    extents=localExtents(rows,lp).*box;
    if options.Verbose,fprintf('%s: polytope ceiling: converged %d, %d facets, extents %s\n',label,converged,size(rows,1),mat2str(extents.',3));end
    out=struct('label',label,'ellipsoidExtents',ellipsoid,'polytopeExtents',extents,'facets',size(rows,1),'converged',converged, ...
        'iterations',iteration,'lambda',lambda,'vertices',2^m);
end

function kept=localIrredundant(rows,lp,tolerance)
    % Drop the rows whose maximum over the polytope of the others is <= 1.
    keep=true(size(rows,1),1);
    for index=1:size(rows,1)
        others=rows(keep & (1:size(rows,1)).'~=index,:);
        [~,fval,flag]=linprog(-rows(index,:).',others,ones(size(others,1),1),[],[],[],[],lp);
        if flag==1 && -fval<=1+tolerance,keep(index)=false;end
    end
    kept=rows(keep,:);
end

function ok=localContained(a,b,lp,tolerance)
    % {a x <= 1} contains {b x <= 1} and conversely: equal polytopes.
    ok=true;
    for index=1:size(a,1)
        [~,fval,flag]=linprog(-a(index,:).',b,ones(size(b,1),1),[],[],[],[],lp);
        if ~(flag==1 && -fval<=1+tolerance),ok=false;return;end
    end
    for index=1:size(b,1)
        [~,fval,flag]=linprog(-b(index,:).',a,ones(size(a,1),1),[],[],[],[],lp);
        if ~(flag==1 && -fval<=1+tolerance),ok=false;return;end
    end
end

function extents=localExtents(rows,lp)
    extents=zeros(5,1);
    for d=1:5
        c=zeros(5,1);c(d)=-1;[~,fval]=linprog(c,rows,ones(size(rows,1),1),[],[],[],[],lp);extents(d)=-fval;
    end
end
