% Trust-box position of the issued input after 10 s, irrespective of encounter status.
files=dir('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/replay/*-full.mat');
for i=1:numel(files)
    load(fullfile(files(i).folder,files(i).name),'record');f=record.frame;n=numel(f);
    d0=zeros(2,n);d1=zeros(2,n);e0=zeros(5,n);time=[f.time];exitStep=[f.encounterExit];
    for k=1:n,d0(:,k)=f(k).input-f(k).anchorInputs(:,1);d1(:,k)=f(k).plan(:,2)-f(k).anchorInputs(:,2);e0(:,k)=f(k).error0;end
    late=time>=10;
    sUp=d0(1,:)>0.0749;sLo=d0(1,:)<-0.0749;bUp=d0(2,:)>0.1249;bLo=d0(2,:)<-0.1249;
    fprintf('%-24s t>=10 s frames %4d | steering: upper %4d lower %4d interior %4d | braking: upper %4d lower %4d | u1 counter-correction (sign opposite to u0) %4d | in encounter %4d | speed error median %+.2f m/s\n', ...
        erase(files(i).name,'-full.mat'),nnz(late),nnz(sUp&late),nnz(sLo&late),nnz(~sUp&~sLo&late),nnz(bUp&late),nnz(bLo&late), ...
        nnz(sign(d1(1,:))==-sign(d0(1,:))&late),nnz(exitStep~=0&late),median(e0(3,late)));
end
