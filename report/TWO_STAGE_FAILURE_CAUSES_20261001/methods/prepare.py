from pathlib import Path
root=Path('/home/zai/Downloads/ResearchProjects/collisionAvoidance');out=Path(__file__).parent
s=(root/'controller/solvePredictiveControl.m').read_text();tail=s[s.index('function [anchor,source]=localInitialization'):]
tail=tail.replace('rows=cell(1,5*count+3);bounds=cell(size(rows));rowCount=0;','rows=cell(1,5*count+3);bounds=cell(size(rows));rowCount=0;groups=cell(size(rows));stages=zeros(size(rows));')
tail=tail.replace('bounds{rowCount}=[rate(finite)-difference(finite);rate(finite)+difference(finite)];','bounds{rowCount}=[rate(finite)-difference(finite);rate(finite)+difference(finite)];groups{rowCount}="slew";stages(rowCount)=index;')
old='rows{rowCount}=r;bounds{rowCount}=g;'
for label,stage in [('collisionStart','index'),('collisionMid','index'),('collisionEnd','count+1')]:
 pos=tail.index(old)
 tail=tail[:pos]+tail[pos:].replace(old,old+f'groups{{rowCount}}="{label}";stages(rowCount)={stage};',1).replace(old,old.replace('bounds','bounds'),0)
 # Find the next unlabelled occurrence on the following replacement.
 if label!='collisionEnd':
  tail=tail[:pos]+tail[pos:].replace(old,old.replace('=r','= r'),1)
tail=tail.replace('bounds{rowCount}=[physicalUpper-middle(4:6);middle(4:6)-physicalLower];','bounds{rowCount}=[physicalUpper-middle(4:6);middle(4:6)-physicalLower];groups{rowCount}="stateMid";stages(rowCount)=index;')
tail=tail.replace('rows{rowCount}=terminalA;bounds{rowCount}=terminalB;','rows{rowCount}=terminalA;bounds{rowCount}=terminalB;groups{rowCount}="terminalGeometry";stages(rowCount)=count;')
needle="'terminalA',terminalA,'terminalB',terminalB,'encounterExit',departure);"
tail=tail.replace(needle,needle+"\n    lengths=cellfun(@(x)size(x,1),rows(1:rowCount));\n    problem.rowGroups=repelem(string(groups(1:rowCount)).',lengths.');\n    problem.rowStages=repelem(stages(1:rowCount).',lengths.');")
head="""function [solution,search,model]=captureOnly(model,previousState,timer) %#ok<INUSD>
    [anchor,source]=localInitialization(model,previousState);
    [problem,model]=localFormulate(anchor,model);
    capture=struct('model',model,'anchor',anchor,'problem',problem,'source',source);
    assignin('base','captured',capture);
    solution=[];search=[];
    error('diagnostic:captured','Captured the unchanged affine problem.');
end

"""
(out/'captureOnly.m').write_text(head+tail)
s=(root/'controller/collisionAvoidanceController.m').read_text().replace('        collisionAvoidanceController(egoState,targetEstimate,laneCenterline,cfg,previousState)','        captureController(egoState,targetEstimate,laneCenterline,cfg,previousState)',1).replace('=solvePredictiveControl(model,previousState,timer);','=captureOnly(model,previousState,timer);')
(out/'captureController.m').write_text(s)
