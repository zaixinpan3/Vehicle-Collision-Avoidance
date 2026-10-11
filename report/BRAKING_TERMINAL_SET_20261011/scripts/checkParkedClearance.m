function result = checkParkedClearance(outputFile,cases,seed)
%checkParkedClearance Closed-form parked check against dense sampling (study script).
% For random parked ego poses and random targets on lines and circles (signed
% speed, some turning back), compares terminalSafeSet.parkedClearance with
% the least rectangle distance over 200001 samples of the swept stretch in arc
% length (the whole circle when the closed form reports it), for the cases in
% which the two rectangles are apart at the start and the target is in range.
% Writes the summary as JSON to outputFile.
    if nargin<2,cases=3000;end
    if nargin<3,seed=11;end
    root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'solver','controller'));
    context=struct('range',50,'shape',[2.4;.95;0;0]);reach=2*norm([2.4;.95]);
    rng(seed);rows=zeros(0,5);
    for trial=1:cases
        pose=[0;0;2*pi*rand];
        q=[(rand(2,1)-.5)*40;2*pi*rand;(rand-.3)*15;(rand-.5)*6;0;1.6;2.4;.95;0;0];
        if rand<.6,q(6)=(rand-.5)*.4;end
        if rand<.1,q(5)=0;end
        start=predictiveSafetyGeometry.rectangle(pose,context.shape,q(1:3),q(8:11));
        if start<=.05 || norm(q(1:2))>50,continue;end
        [distance,witness,track]=terminalSafeSet.parkedClearance(context,pose,q);
        k=sin(q(6))/q(7);
        if track.fullCircle,s=linspace(0,2*pi/abs(k),200001);else,s=linspace(track.lower,track.upper,200001);end
        half=k*s/2;scale=ones(size(half));bend=half~=0;scale(bend)=sin(half(bend))./half(bend);
        position=q(1:2)+(s.*scale).*predictiveSafetyGeometry.direction(q(3)+q(6)+half);heading=q(3)+k*s;
        dense=Inf;
        for j=find(vecnorm(position-pose(1:2),2,1)<distance+reach+.5)
            dense=min(dense,predictiveSafetyGeometry.rectangle(pose,context.shape,[position(:,j);heading(j)],q(8:11)));
        end
        attained=predictiveSafetyGeometry.rectangle(pose,context.shape,witness(1:3),witness(8:11));
        rows(end+1,:)=[dense-distance,abs(attained-distance),track.fullCircle,q(4)*q(5)<0,s(2)-s(1)]; %#ok<AGROW>
    end
    result=struct('seed',seed,'cases',size(rows,1),'fullCircles',nnz(rows(:,3)),'turning',nnz(rows(:,4)), ...
        'largestDenseMinusClosedMeters',max(rows(:,1)),'smallestDenseMinusClosedMeters',min(rows(:,1)), ...
        'largestWitnessMismatchMeters',max(rows(:,2)),'largestSampleSpacingMeters',max(rows(:,5)));
    fid=fopen(outputFile,'w');fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));fclose(fid);
    disp(result);
end
