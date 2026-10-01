% Box position of the issued and the second planned input relative to the anchor.
load(sprintf('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/replay/%s-%d.mat',name,speed),'record');
f=record.frame;n=numel(f);d0=zeros(2,n);d1=zeros(2,n);e=zeros(5,n);
for k=1:n
    d0(:,k)=f(k).input-f(k).anchorInputs(:,1);d1(:,k)=f(k).plan(:,2)-f(k).anchorInputs(:,2);e(:,k)=f(k).error0;
end
for k=[1:10:n,n]
    fprintf('t=%5.2f ey=%8.2f epsi=%6.3f | steer u0=%7.4f anchor=%7.4f u0-anchor=%+7.4f | u1-anchor1=%+7.4f | brake u0-anchor=%+7.4f u1-anchor1=%+7.4f | V0=%9.3g dVaff=%+9.3g dVact=%+9.3g\n', ...
        f(k).time,e(1,k),e(2,k),f(k).input(1),f(k).anchorInputs(1,1),d0(1,k),d1(1,k),d0(2,k),d1(2,k), ...
        f(k).clfInitialValue,f(k).clfNextValue-f(k).clfInitialValue,f(k).actualClfNextValue-f(k).clfInitialValue);
end
late=[f.time]>=4;
fprintf('frames t>=4 s: %d; steering at upper edge (u0-anchor>0.0749): %d, at lower edge: %d; median u1-anchor1 %.4f; actual dV>0 in %d frames\n', ...
    nnz(late),nnz(d0(1,late)>0.0749),nnz(d0(1,late)<-0.0749),median(d1(1,late)),nnz([f(late).actualClfNextValue]>[f(late).clfInitialValue]));
