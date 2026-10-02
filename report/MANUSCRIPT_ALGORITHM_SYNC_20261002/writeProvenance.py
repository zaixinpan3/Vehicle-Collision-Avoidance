#!/usr/bin/env python3
"""Write provenance.json for the October 2, 2026 manuscript synchronization.

Usage: python3 writeProvenance.py <repository root>

Run it after auditManuscript.py has written audit.json and after the
manuscript and its two new figure assets were built. It reads Git objects of
the reviewed commit and the files named below; it runs no simulation.
"""
import hashlib
import json
import platform
import subprocess
import sys
from pathlib import Path

REVIEWED = '5692e1551241b568bcfb300093814373c674a412'
CAMPAIGN = 'a29ccb6ff2856fe3df33efff4b1cb8619e523658'
SOURCES = [
    'controller/collisionAvoidanceController.m', 'controller/solvePredictiveControl.m',
    'controller/predictiveSafetyGeometry.m', 'controller/terminalContinuation.m',
    'controller/nonlinearBicycleModel.m', 'controller/modifiedFialaTire.m',
    'controller/laneGeometry.m', 'controller/readControllerInputs.m',
    'config/collisionAvoidanceControllerConfig.m', 'scripts/collisionThreatScenario.m',
    'scripts/runNonlinearPredictiveSafetyValidation.m',
    'estimator/synthesizeTargetTrackerCertificate.m', 'estimator/synthesizeNrmmObserverGains.m',
    'estimator/nrmmTargetLipschitzCertificate.m', 'config/nrmmTrackingConfig.m',
    'config/mncavVehicleConfig.m']
RECORDS = [
    'report/SHARMA_TIMED_FLOW_20261002.tex', 'report/SHARMA_TIMED_FLOW_20261002/comparison.json',
    'report/SHARMA_TIMED_FLOW_20261002/provenance.json',
    'report/CONTROLLER_FAILURE_CAUSES_20261002.tex',
    'report/CONTROLLER_FAILURE_CAUSES_20261002/findings.json',
    'report/TARGET_NOISE_SHAPE_GAINS_20261001.tex',
    'report/TARGET_NOISE_SHAPE_GAINS_20261001/design_table.csv',
    'report/TARGET_NOISE_SHAPE_GAINS_20261001/benchmark_summary.csv',
    'report/TARGET_NOISE_SHAPE_GAINS_20261001/audit/summary.csv',
    'report/TARGET_NOISE_SHAPE_GAINS_20261001/audit/scenario-tuned.csv',
    'report/TARGET_NOISE_SHAPE_GAINS_20261001/audit/trials.csv',
    'report/TARGET_NOISE_SHAPE_GAINS_20261001/audit/paired_intervals.csv',
    'report/TARGET_NOISE_SHAPE_GAINS_20261001/audit/oracle_summary.csv',
    'report/SHARMA_ESTIMATOR_AUDIT_20260930.tex']
OUTPUTS = ['paper/figures/closed_loop_results_v01.pdf', 'paper/figures/system_architecture_v11.pdf',
           'paper/manuscript.pdf']


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    repo = Path(sys.argv[1]).resolve()
    bundle = repo / 'report/MANUSCRIPT_ALGORITHM_SYNC_20261002'

    def git(*args, text=True):
        return subprocess.run(['git', '-C', str(repo), *args], capture_output=True, text=text,
                              check=True).stdout

    def blob(path):
        return sha(git('show', f'{REVIEWED}:{path}', text=False))

    audit = json.loads((bundle / 'audit.json').read_text())
    flow = json.loads((repo / 'report/SHARMA_TIMED_FLOW_20261002/provenance.json').read_text())
    status = git('status', '--short').splitlines()
    pages = subprocess.run(['pdfinfo', str(repo / 'paper/manuscript.pdf')], capture_output=True,
                           text=True, check=True).stdout.split('Pages:')[1].split()[0]
    provenance = {
        'task': 'Synchronize paper/manuscript.tex with the committed estimator and controller',
        'date': '2026-10-02',
        'reviewedCommit': REVIEWED,
        'controllerCampaignCommit': CAMPAIGN,
        'precedingManuscriptCommits': ['e64d5dd9bf6d476470f7c3395d61fa0821fd60d8',
                                       'c152323e18a24995b518fb8c7839c9886324ec74'],
        'controllerSourceHashesMatchCampaignRecord': all(
            blob(path) == digest for path, digest in flow['sourceHashes'].items()
            if path.startswith('controller/')),
        'sourceSha256AtReviewedCommit': {path: blob(path) for path in SOURCES},
        'recordSha256': {path: sha((repo / path).read_bytes()) for path in RECORDS},
        'concurrentUncommittedPathsNotDescribedAsImplemented': sorted(
            line[3:] for line in status
            if line.startswith(' M') and not line[3:].startswith('paper/')),
        'tools': {
            'matlab': json.loads((bundle / 'estimator-design.json').read_text())['matlabVersion'],
            'python': platform.python_version(),
            'pdftex': subprocess.run(['pdflatex', '--version'], capture_output=True,
                                     text=True).stdout.splitlines()[0]},
        'commands': [
            'git archive HEAD controller config scripts | tar -x -C <export>',
            "matlab -singleCompThread -batch \"addpath('report/MANUSCRIPT_ALGORITHM_SYNC_20261002'); "
            "evaluateEstimatorDesign(pwd,'<bundle>/estimator-design.json'); "
            "evaluateControllerConstants('<export>','<bundle>/controller-constants.json')\"",
            'python3 report/MANUSCRIPT_ALGORITHM_SYNC_20261002/deriveClosedLoopStatistics.py . '
            '<bundle>/closed-loop-statistics.json',
            'python3 report/MANUSCRIPT_ALGORITHM_SYNC_20261002/checkDualLemma.py 40000',
            'python3 paper/figures/closed_loop_results_v01.py .',
            'pdflatex system_architecture_v11.tex (in paper/figures)',
            'latexmk -pdf -interaction=nonstopmode -halt-on-error manuscript.tex (in paper)',
            'python3 report/MANUSCRIPT_ALGORITHM_SYNC_20261002/auditManuscript.py . '
            '<bundle>/audit.json 5692e1551241b568bcfb300093814373c674a412',
            'python3 report/MANUSCRIPT_ALGORITHM_SYNC_20261002/writeProvenance.py .'],
        'manuscriptSourceSha256': {
            path: sha((repo / path).read_bytes())
            for path in ('paper/manuscript.tex', 'paper/references.bib')},
        'localOutputSha256': {path: sha((repo / path).read_bytes()) for path in OUTPUTS},
        'manuscriptPages': int(pages),
        'audit': {'checks': audit['checks'], 'failed': audit['failed']},
        'notRun': ['closed-loop simulation', 'estimator trials', 'MATLAB unit-test suite'],
        'untrackedOutputs': 'paper/figures/ and paper/manuscript.pdf are local derived outputs '
                            'and are not committed',
    }
    (bundle / 'provenance.json').write_text(json.dumps(provenance, indent=1) + '\n')
    print(json.dumps({key: provenance[key] for key in (
        'reviewedCommit', 'controllerSourceHashesMatchCampaignRecord', 'manuscriptPages',
        'audit')}))


if __name__ == '__main__':
    main()
