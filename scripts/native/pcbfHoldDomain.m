function rows=pcbfHoldDomain(dense,input,cfg)
    rows=solvePredictiveControl("outputFeedback",struct('operation',"holdDomain", ...
        'dense',dense,'input',input,'cfg',cfg));
end
