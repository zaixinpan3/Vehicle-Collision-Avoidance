% Trust-box usage of one record file (set file).
load(file,'record');f=record.frame;n=numel(f);
d0=zeros(2,n);d1=zeros(2,n);dsum=zeros(1,n);time=[f.time];
for k=1:n
    d=f(k).plan-f(k).anchorInputs;d0(:,k)=d(:,1);d1(:,k)=d(:,2);dsum(k)=sum(d(1,2:end));
end
late=time>=10;
fprintf('%s: frames %d (t>=10: %d) | steer upper edge %d lower edge %d | median d1 %+.4f, median sum of future steering corrections %+.4f | final heading err %.3f ey %.1f\n', ...
    file,n,nnz(late),nnz(d0(1,:)>0.0749&late),nnz(d0(1,:)<-0.0749&late),median(d1(1,late)),median(dsum(late)),f(end).error1(2),f(end).error1(1));
