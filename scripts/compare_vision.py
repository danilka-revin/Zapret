#!/usr/bin/env python3
"""Compare two saved fixed-profile vision evaluations, never select on test.

No inference or model/threshold selection: use evaluate_vision.py first.
Data fingerprint, split, frame names/truth classes and inference settings
must match. Weights and trafficMode may differ: those are the treatments.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from robot_control.vision.classes import CLASS_NAMES  # noqa: E402


def _rates(counts: dict) -> dict:
    tp, fp, fn = (int(counts[k]) for k in ('tp', 'fp', 'fn'))
    return {'tp': tp, 'fp': fp, 'fn': fn,
            'precision': tp / (tp + fp) if tp + fp else 0.,
            'recall': tp / (tp + fn) if tp + fn else 0.,
            'f1': 2 * tp / (2 * tp + fp + fn) if 2 * tp + fp + fn else 0.}


def summarize(report: dict) -> dict:
    classes = {name: _rates(report['counts'][name]) for name in CLASS_NAMES}
    micro = _rates({key: sum(c[key] for c in classes.values()) for key in ('tp', 'fp', 'fn')})
    confusion = report['signalConfusion']
    signals = {}
    for state in ('red', 'yellow', 'green'):
        support = sum(value for key, value in confusion.items() if key.startswith(state + '->'))
        emitted = sum(value for key, value in confusion.items() if key.endswith('->' + state))
        correct = confusion.get(f'{state}->{state}', 0)
        signals[state] = {'correct': correct, 'support': support, 'predictions': emitted,
                          'recallIncludingMisses': correct / support if support else 0.,
                          'precision': correct / emitted if emitted else 0.}
    macro_f1 = sum(c['f1'] for c in classes.values()) / len(classes)
    balanced_recall = sum(c['recallIncludingMisses'] for c in signals.values()) / 3
    return {'micro': micro, 'classes': classes, 'macroF1': macro_f1, 'signals': signals,
            'signalBalancedRecall': balanced_recall, 'falseGreen': report['falseGreen'],
            'validationObjective': .7 * macro_f1 + .3 * balanced_recall,
            'latencyMedianMs': report.get('latencyMedianMs'),
            'latencyP95Ms': report.get('latencyP95Ms')}


def compare(before: dict, after: dict) -> dict:
    for key in ('datasetFingerprint', 'split', 'images', 'confidence', 'imgsz',
                'device', 'preferRed', 'iouMatch', 'backend'):
        if key not in before or key not in after or before[key] != after[key]:
            raise ValueError(f'Non-comparable evaluations: {key} must match')
    for report in (before, after):
        names = [row['image'] for row in report['frames']]
        if len(names) != report['images'] or len(names) != len(set(names)):
            raise ValueError('Duplicate or missing frames in an evaluation')
    frames = [{row['image']: row['groundTruthClasses'] for row in r['frames']} for r in (before, after)]
    if frames[0] != frames[1]:
        raise ValueError('Frame names or ground truth differ')
    summaries = [summarize(report) for report in (before, after)]
    for state in ('red', 'yellow', 'green'):
        if summaries[0]['signals'][state]['support'] != summaries[1]['signals'][state]['support']:
            raise ValueError('Signal ground-truth support differs')
    summary = {
        'split': before['split'], 'images': before['images'],
        'datasetFingerprint': before['datasetFingerprint'],
        'profile': {k: before[k] for k in ('confidence', 'imgsz', 'device', 'preferRed', 'iouMatch', 'backend')},
        'before': {'weights': before['weights'], 'sha256': before['sha256'],
                   'trafficMode': before.get('trafficMode', 'legacy'), **summaries[0]},
        'after': {'weights': after['weights'], 'sha256': after['sha256'],
                  'trafficMode': after.get('trafficMode', 'legacy'), **summaries[1]},
        'delta': {key: summaries[1][key] - summaries[0][key]
                  for key in ('macroF1', 'signalBalancedRecall', 'falseGreen', 'validationObjective')},
        'warning': 'Same saved images/agent-assisted labels, NOT independent human ground truth. '
                   'Signal recall includes missed bodies. Confidence is not calibrated correctness. '
                   'No model selection on test is performed here. Zero observed false-green is not safety proof.',
    }
    summary['delta']['micro'] = {k: summaries[1]['micro'][k] - summaries[0]['micro'][k]
                                  for k in summaries[0]['micro']}
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', required=True, type=Path)
    parser.add_argument('--after', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = compare(json.loads(args.before.read_text()), json.loads(args.after.read_text()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
