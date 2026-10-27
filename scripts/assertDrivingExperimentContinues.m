function assertDrivingExperimentContinues(state,terminalControllerActive)
%assertDrivingExperimentContinues Fail a driving trial on backup activation or rest.
% State is the physical [X;Y;psi;vx;vy;r], never a predicted/estimated state.
% The declared plant resets all three motion states to exactly zero at rest.
% Mere terminal-set membership and ordinary optimized braking are allowed.
    arguments
        state (6,1) double {mustBeReal,mustBeFinite}
        terminalControllerActive (1,1) logical
    end
    if terminalControllerActive
        error('runNonlinearPredictiveSafetyValidation:terminalControllerActivated', ...
            'The terminal controller was activated; end the driving trial before executing its command.');
    end
    if all(state(4:6)==0)
        error('runNonlinearPredictiveSafetyValidation:unexpectedStop', ...
            'The physical ego reached rest; stop the driving trial without restarting it.');
    end
end
