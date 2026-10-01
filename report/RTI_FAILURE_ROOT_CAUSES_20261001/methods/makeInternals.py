#!/usr/bin/env python3
"""Generate methods/rtiInternals.m from controller/solvePredictiveControl.m.

The primary function is replaced by a dispatcher; every local function is copied
unchanged except that localFormulate also records constraint-row labels
[kind, stage]: 1 braking slew, 2 start collision, 3 midpoint collision,
4 midpoint state bounds, 5 terminal clearance, 6 endpoint collision.
Usage: makeInternals.py <solvePredictiveControl.m> <rtiInternals.m>
"""
import sys
src = open(sys.argv[1]).read()
body = src[src.index('function attempt=localAttempt(search)'):]
f0 = body.index('function [problem,model]=localFormulate(anchor,model)')
f1 = body.index('function u=localClip(u,previous,cfg)')
form = body[f0:f1]
form = form.replace("rows=cell(1,5*count+3);bounds=cell(size(rows));rowCount=0;",
                    "rows=cell(1,5*count+3);bounds=cell(size(rows));rowCount=0;labels=cell(size(rows));")
def tag(after, label):
    global form
    assert form.count(after) == 1, after
    form = form.replace(after, after + label)
tag("rowCount=rowCount+1;rows{rowCount}=[r(finite,:);-r(finite,:)];bounds{rowCount}=[rate(finite)-difference(finite);rate(finite)+difference(finite)];",
    "labels{rowCount}=repmat([1,index],2*nnz(finite),1);")
stmt = "rowCount=rowCount+1;rows{rowCount}=r;bounds{rowCount}=g;"
assert form.count(stmt) == 3
p = form.split(stmt)
form = (p[0] + stmt + "labels{rowCount}=repmat([2,index],4,1);" + p[1] + stmt + "labels{rowCount}=repmat([3,index],4,1);"
        + p[2] + stmt + "labels{rowCount}=repmat([6,count],size(g,1),1);" + p[3])
tag("bounds{rowCount}=[physicalUpper-middle(4:6);middle(4:6)-physicalLower];", "labels{rowCount}=repmat([4,index],6,1);")
tag("rowCount=rowCount+1;rows{rowCount}=terminalA;bounds{rowCount}=terminalB;", "labels{rowCount}=repmat([5,count],size(terminalB,1),1);")
old = "'terminalA',terminalA,'terminalB',terminalB,'encounterExit',departure);"
assert form.count(old) == 1
form = form.replace(old, "'terminalA',terminalA,'terminalB',terminalB,'encounterExit',departure, ...\n        'rowLabels',vertcat(labels{1:rowCount}));")
body = body[:f0] + form + body[f1:]
dispatcher = '''function varargout = rtiInternals(action,varargin)
%rtiInternals DIAGNOSTIC copy of controller/solvePredictiveControl.m local functions.
% All local functions are copied unchanged except that localFormulate also
% returns constraint-row labels: [kind,stage] with kind 1 braking slew,
% 2 start collision, 3 midpoint collision, 4 midpoint state bounds,
% 5 terminal clearance, 6 endpoint collision. No numerical statement differs.
    switch action
        case 'initialization',[varargout{1:nargout}]=localInitialization(varargin{:});
        case 'flowSeed',[varargout{1:nargout}]=localFlowSeed(varargin{:});
        case 'formulate',[varargout{1:nargout}]=localFormulate(varargin{:});
        case 'step',[varargout{1:nargout}]=localStep(varargin{:});
        case 'safetyRows',[varargout{1:nargout}]=localSafetyRows(varargin{:});
        case 'beyondRange',[varargout{1:nargout}]=localBeyondRange(varargin{:});
        case 'terminalClearance',[varargout{1:nargout}]=localTerminalClearance(varargin{:});
        otherwise,error('rtiInternals:unknownAction','Unknown action %s.',action);
    end
end

'''
open(sys.argv[2], 'w').write(dispatcher + body)
