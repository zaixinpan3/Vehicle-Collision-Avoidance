function program = updateAvoidanceStageBounds(qp)
%updateAvoidanceStageBounds Update hard RHS values without touching equalities.
% The retained row reduction is valid for the entire carried-margin interval.
    program = qp.stageProgram;
    program.b(program.rowMap.inequality) = program.inequalityOffset ...
        +qp.inequalityBound(program.inequalityIndices);
end
