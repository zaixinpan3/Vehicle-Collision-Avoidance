"""Add diagnostic timing to an isolated solvePredictiveControl copy.

The production source is never overwritten. Exact text matches deliberately
reject a different source layout rather than silently producing partial timing.
"""

import argparse
from pathlib import Path


def instrument(source):
    def replace(old, new):
        nonlocal source
        if source.count(old) != 1:
            raise ValueError(f'Expected one instrumentation anchor: {old[:100]}')
        source = source.replace(old, new)

    replace('    baseline=localNominalEvaluation(anchor,model);',
            '    baseline=localNominalEvaluation(anchor,model);\n'
            '    search.initialEvaluationSeconds=baseline.profileSeconds;')
    replace('            candidate=localNominalEvaluation(inputs,model);',
            '            candidate=localNominalEvaluation(inputs,model);\n'
            '            step.evaluationSeconds=candidate.profileSeconds;\n'
            '            step.nominalClfSlack=candidate.clfSlack;\n'
            '            step.previousSafety=baseline.safety;\n'
            '            step.previousHard=baseline.hard;\n'
            '            step.previousClfSlack=baseline.clfSlack;')
    replace('            search.converged=solution.safety<=cfg.solver.feasibilityTolerance ...',
            '            step.returnedClfSlack=solution.clfSlack;\n'
            '            step.clfLinearizationGap=solution.clfSlack-step.clfSlack;\n'
            '            search.sequentialIterations{end}=step;\n'
            '            search.converged=solution.safety<=cfg.solver.feasibilityTolerance ...')
    replace('function [inputs,info]=localSequentialStep(anchor,model,radius)\n',
            'function [inputs,info]=localSequentialStep(anchor,model,radius)\n'
            '    assemblyTimer=tic;\n')
    replace('    [first,~,flag]=linprog(objective,a,b,equal,rhs,lower,upper,options);info.calls=1;',
            '    info.assemblySeconds=toc(assemblyTimer);\n'
            '    info.variables=nv;info.equalities=size(equal,1);\n'
            '    info.primaryInequalities=size(a,1);\n'
            '    info.hessianNonzeros=nnz(hessian);info.equalityNonzeros=nnz(equal);\n'
            '    info.primaryInequalityNonzeros=nnz(a);\n'
            '    lpTimer=tic;\n'
            '    [first,~,flag,lpOutput]=linprog(objective,a,b,equal,rhs,lower,upper,options);info.calls=1;\n'
            '    info.lpSeconds=toc(lpTimer);info.lpIterations=lpOutput.iterations;')
    replace('    [second,~,flag]=quadprog(hessian,linear,a,b,equal,rhs,lower,upper,[],options);info.calls=2;',
            '    qpTimer=tic;\n'
            '    [second,~,flag,qpOutput]=quadprog(hessian,linear,a,b,equal,rhs,lower,upper,[],options);info.calls=2;\n'
            '    info.qpSeconds=toc(qpTimer);info.qpIterations=qpOutput.iterations;')
    replace('function evaluation=localNominalEvaluation(inputs,model)\n',
            'function evaluation=localNominalEvaluation(inputs,model)\n'
            '    evaluationTimer=tic;\n')
    replace("        'cost',localCost(states,inputs,model)+cfg.clf.relaxationWeight*clf^2);",
            "        'cost',localCost(states,inputs,model)+cfg.clf.relaxationWeight*clf^2);\n"
            '    evaluation.profileSeconds=toc(evaluationTimer);')
    return source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    if args.source.resolve() == args.destination.resolve():
        raise ValueError('Timing must be added to an isolated copy.')
    result = instrument(args.source.read_text())
    args.destination.parent.mkdir(parents=True, exist_ok=True)
    args.destination.write_text(result)


if __name__ == '__main__':
    main()
