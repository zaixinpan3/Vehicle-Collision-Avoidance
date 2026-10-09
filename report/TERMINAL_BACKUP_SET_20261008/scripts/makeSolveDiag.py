"""Write solvePredictiveControlDiag.m: a copy of the committed
controller/solvePredictiveControl.m that also stores the startup seed in the
base workspace (seedAnchor). Usage: python3 makeSolveDiag.py <repo> <outDir>."""
import sys
repo, out = sys.argv[1], sys.argv[2]
s = open(repo + '/controller/solvePredictiveControl.m').read()
s = s.replace('solvePredictiveControl(', 'solvePredictiveControlDiag(', 1)
# Source of commit f5e1ab7 (one startup rollout) or later (retry toward the
# collision cone's lane): the seed, and the retry when there is one, go to the
# base workspace as seedAnchor and seedRetry.
old = "    if member,anchor=localReturnToNominal(anchor,model);end\nend"
if s.count(old) == 1:
    s = s.replace(old, old[:-4] + "\n    assignin('base','seedAnchor',anchor);\nend")
else:
    old = "        retry.seedRetried=true;\n"
    assert s.count(old) == 1
    s = s.replace(old, old + "        assignin('base','seedRetry',retry);\n")
    old = "    if anchor.seedReachedTerminalSet,anchor=localReturnToNominal(anchor,model);end\nend"
    assert s.count(old) == 1
    s = s.replace(old, old[:-4] + "\n    assignin('base','seedAnchor',anchor);\nend")
open(out + '/solvePredictiveControlDiag.m', 'w').write(s)
