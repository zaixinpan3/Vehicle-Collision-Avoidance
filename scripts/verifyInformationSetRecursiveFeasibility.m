function results = verifyInformationSetRecursiveFeasibility(outputFile)
%verifyInformationSetRecursiveFeasibility Exact scalar checks of the joint-set shift.
% Straight, forward-only kinematics and a stationary target; the rectangles'
% summed longitudinal half-length is 4.8 m. This checks the mathematical
% construction with nonzero estimation error, not the six-state Fiala model.
    if nargin<1,outputFile="";end
    h=.2;braking=4;bodyLength=4.8;margin=.2;
    settled=localCase("zeroSlack",[-.2,.6;7.9,8.5;17.7,18.3], ...
        [1.1,3.1;7.8,8.8;17.6,18.6],[0;8;18],[.2;8.2], ...
        [2.1;8.3],[0,2,4],2,h,braking,bodyLength,margin);
    relaxed=localCase("positiveSlack",[15.8,16.6;14.7,15.3;17.7,18.3], ...
        [18.6,20.6;14.7,15.7;17.6,18.6],[16.2;15;18],[16.2;15], ...
        [19.6;15.2],[0,0,0,0],4,h,braking,bodyLength,margin);
    results=[settled,relaxed];
    if strlength(string(outputFile))>0
        file=fopen(outputFile,'w');assert(file>=0,'Cannot open result file.');
        cleaner=onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(results,PrettyPrint=true));
    end
    disp(struct2table(results));
end

function result=localCase(name,initial,measurement,truth,oldEstimate,newEstimate, ...
        inputs,prefix,h,braking,bodyLength,margin)
    old=localRollout(initial,inputs,h);
    nextTruth=localAdvance([truth,truth],inputs(1),h);
    assert(all(truth>=initial(:,1) & truth<=initial(:,2)));
    assert(all(nextTruth(:,1)>=measurement(:,1) & nextTruth(:,1)<=measurement(:,2)));
    posterior=[max(old{2}(:,1),measurement(:,1)),min(old{2}(:,2),measurement(:,2))];
    assert(all(posterior(:,1)<=posterior(:,2)));
    assert(localSubset(posterior,old{2}));
    candidateInputs=[inputs(2:end),braking];
    candidate=localRollout(posterior,candidateInputs,h);
    count=numel(inputs);inclusion=true;
    for i=1:count,inclusion=inclusion && localSubset(candidate{i},old{i+1});end
    assert(inclusion);
    oldDeficits=localDeficits(old,bodyLength,margin);
    candidateDeficits=localDeficits(candidate,bodyLength,margin);
    assert(all(oldDeficits(prefix+1:end)==0));
    assert(all(candidateDeficits(prefix+1:end)==0));
    oldTerminal=localTerminalMargin(old{end},braking,bodyLength,margin);
    candidateTerminal=localTerminalMargin(candidate{end},braking,bodyLength,margin);
    assert(oldTerminal>=0 && candidateTerminal>=0);
    assert(candidateDeficits(prefix)==0);
    inheritedBudget=sum(oldDeficits(2:prefix));
    newCost=sum(candidateDeficits(1:prefix));
    assert(newCost<=inheritedBudget+1e-12);
    raw=localRollout(measurement,candidateInputs,h);
    predictedEstimate=localAdvance([oldEstimate,oldEstimate;truth(3),truth(3)],inputs(1),h);
    result=struct('scenario',name,'sampleTimeSeconds',h,'prefixSteps',prefix, ...
        'horizonSteps',count,'brakingMetersPerSecondSquared',braking, ...
        'initialPositionWidthMeters',diff(initial(1,:)), ...
        'initialSpeedWidthMetersPerSecond',diff(initial(2,:)), ...
        'positionInnovationMeters',newEstimate(1)-predictedEstimate(1,1), ...
        'rawMeasurementNested',localSubset(measurement,old{2}), ...
        'posteriorNested',localSubset(posterior,old{2}),'shiftSetsIncluded',inclusion, ...
        'oldCostMeters',sum(oldDeficits(1:prefix)),'firstDeficitMeters',oldDeficits(1), ...
        'candidateCostMeters',newCost,'inheritedBudgetMeters',inheritedBudget, ...
        'oldTerminalMarginMeters',oldTerminal,'candidateTerminalMarginMeters',candidateTerminal, ...
        'rawResetTerminalMarginMeters',localTerminalMargin(raw{end},braking,bodyLength,margin));
end

function sets=localRollout(initial,inputs,h)
    sets=cell(1,numel(inputs)+1);sets{1}=initial;
    for i=1:numel(inputs),sets{i+1}=localAdvance(sets{i},inputs(i),h);end
end

function next=localAdvance(set,braking,h)
    v=set(2,:);duration=h+zeros(1,2);
    if braking>0,duration=min(duration,v/braking);end
    next=set;next(1,:)=set(1,:)+v.*duration-.5*braking*duration.^2;
    next(2,:)=max(0,v-braking*h);
end

function deficits=localDeficits(sets,bodyLength,margin)
    deficits=zeros(1,numel(sets)-1);
    for i=1:numel(deficits)
        % The position moves forward, so this interval covers the whole hold.
        positions=[sets{i}(1,1),sets{i+1}(1,2)];
        distance=localDistance(positions,sets{i}(3,:),bodyLength);
        deficits(i)=max(0,margin-distance);
    end
end

function value=localTerminalMargin(set,braking,bodyLength,margin)
    % All braking continuations and their parked poses are covered exactly
    % by this monotone interval hull; the target is stationary forever.
    positions=[set(1,1),set(1,2)+set(2,2)^2/(2*braking)];
    value=localDistance(positions,set(3,:),bodyLength)-margin;
end

function distance=localDistance(positions,target,bodyLength)
    relative=[target(1)-positions(2),target(2)-positions(1)];
    separation=max([relative(1),-relative(2),0]);
    distance=max(0,separation-bodyLength);
end

function subset=localSubset(first,second)
    subset=all(first(:,1)>=second(:,1)-1e-12 & first(:,2)<=second(:,2)+1e-12);
end
