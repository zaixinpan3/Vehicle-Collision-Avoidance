% Where the time goes in the three recovered 8-m/s cases (no-risk phase only).
D='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001';
tol=[.1;pi/180;.1;.05;.01];
for name=["headOn","acceleratingHeadOn","brakingLead"]
    load(fullfile(D,'replay',name+"-8-full.mat"),'record');f=record.frame;n=numel(f);
    free=[f.encounterExit]==0;rho=[f.clfSlack];V0=[f.clfInitialValue];V1=[f.actualClfNextValue];V1a=[f.clfNextValue];time=[f.time];
    d0=zeros(1,n);e=zeros(5,n);for k=1:n,d0(k)=f(k).input(1)-f(k).anchorInputs(1,1);e(:,k)=f(k).error1;end
    % The campaign criterion: state time after the hold >= 8 s, all errors inside
    % the tolerances, held for 5 s; entry is the first state time inside.
    stateTime=time+0.05;inside=all(abs(e)<=tol,1) & stateTime>=8;entry=NaN;rec=NaN;
    for k=1:n
        if ~inside(k),entry=NaN;elseif isnan(entry),entry=stateTime(k);end
        if inside(k) && stateTime(k)-entry>=5-1e-10,rec=stateTime(k);break;end
    end
    first=find(free,1);B=free & rho<=1e-6*max(1,V0);A=free & ~B & abs(d0)>0.0749;
    lastA=find(A & time<rec,1,'last');
    inB=B & time<rec & time>time(lastA);
    fprintf('%s: encounter ends %.2f s, recovery confirmed %.2f s (band entered %.2f s); trapped (steering edge, rho>0) frames before recovery %d (last at %.2f s, e_y=%.2f m, V0=%.4g)\n', ...
        name,time(first),rec,entry,nnz(A & time<rec),time(lastA),f(lastA).error0(1),V0(lastA));
    fprintf('   after the last trapped frame: %d frames with rho=0; actual V ratio median %.4f (p10 %.4f, p90 %.4f); predicted ratio median %.4f; V from %.4g to %.4g\n', ...
        nnz(inB),median(V1(inB)./V0(inB)),prctile(V1(inB)./V0(inB),10),prctile(V1(inB)./V0(inB),90),median(V1a(inB)./V0(inB)),V0(lastA+1),V0(find(time<rec,1,'last')));
end
