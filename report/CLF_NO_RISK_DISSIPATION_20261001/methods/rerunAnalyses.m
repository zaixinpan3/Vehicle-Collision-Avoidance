% Logged rerun of every table used in the report (all deterministic, offline).
% Each script runs in its own function workspace; the driver's own names are
% chosen so that no analysis script reuses them.
rerunMethods='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001/methods';
rerunLogs='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001/logs';
if ~isfolder(rerunLogs),mkdir(rerunLogs);end
for rerunScript=["regimes","planDecay","localDecay","recoveryTiming","brakingDrift","greedySummary","greedyNearCheck","crossTerms","speedCycle","cancellation","curvedDeparture"]
    localRun(rerunMethods,rerunLogs,rerunScript);
end

function localRun(rerunMethods,rerunLogs,rerunScript)
    rerunLogFile=fullfile(rerunLogs,rerunScript+".log");if isfile(rerunLogFile),delete(rerunLogFile);end
    diary(rerunLogFile);fprintf('=== %s (%s)\n',rerunScript,string(datetime('now','TimeZone','America/Chicago')));
    rerunClock=tic;run(fullfile(rerunMethods,rerunScript+".m"));
    fprintf('=== %s done in %.1f s\n',rerunScript,toc(rerunClock));diary off;
end
