"""Write solvePredictiveControlDiag.m: a copy of the committed
controller/solvePredictiveControl.m that also stores the startup seed in the
base workspace (seedAnchor). Usage: python3 makeSolveDiag.py <repo> <outDir>."""
import sys
repo, out = sys.argv[1], sys.argv[2]
s = open(repo + '/controller/solvePredictiveControl.m').read()
s = s.replace('solvePredictiveControl(', 'solvePredictiveControlDiag(', 1)
old = """    if member,anchor=localReturnToNominal(anchor,model);end
end"""
assert s.count(old) == 1
s = s.replace(old, old[:-4] + "\n    assignin('base','seedAnchor',anchor);\nend")
open(out + '/solvePredictiveControlDiag.m', 'w').write(s)
