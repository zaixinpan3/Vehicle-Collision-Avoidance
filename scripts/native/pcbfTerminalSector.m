function [value,normal,gradient,exitSteps]=pcbfTerminalSector(ego,box,speeds,course,halfWidth,radius,cfg)
%pcbfTerminalSector Evaluate the moving-target terminal separation condition.
    sector=terminalSafeSet.velocitySector(speeds(1),speeds(2),course,halfWidth);
    [value,normal,gradient,exitSteps]=terminalSafeSet.sectorSeparation(ego,box,sector,radius,cfg);
end
