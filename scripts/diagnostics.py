#!/usr/bin/env python3
"""Read retained Neural Racing evidence. Standard library only; never executes events."""
import argparse
import collections
import datetime as dt
import json
import os
from pathlib import Path
import sys


def timestamp(text):
    try:
        if text.isdigit():
            return int(text)
        value = dt.datetime.fromisoformat(text.replace('Z', '+00:00'))
        if value.tzinfo is None:
            raise ValueError('include a timezone, e.g. Z')
        return int(value.timestamp() * 1000)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dir', type=Path, default=Path(os.environ.get(
        'NEURAL_RACING_DIAGNOSTICS_DIR', str(Path.home() / '.local/state/neural-racing/diagnostics'))))
    parser.add_argument('--since', type=timestamp, help='ISO 8601 with timezone or Unix milliseconds')
    parser.add_argument('--until', type=timestamp)
    parser.add_argument('--run')
    parser.add_argument('--operation', type=int, help='operation or direct parent/session/worker/command reference')
    parser.add_argument('--event', action='append', help='repeat for multiple event names')
    parser.add_argument('--summary', action='store_true', help='summarize complete retained runs intersecting time range')
    parser.add_argument('--limit', type=int, default=200, help='maximum event output, 1–10000 (file order)')
    args = parser.parse_args(argv)
    if not 1 <= args.limit <= 10000:
        parser.error('--limit must be between 1 and 10000')
    if args.since is not None and args.until is not None and args.since > args.until:
        parser.error('--since must not exceed --until')
    if args.summary and (args.operation is not None or args.event):
        parser.error('--summary describes retained runs; use event/operation filters without --summary')
    runs = {}
    selected = []
    matches = 0
    malformed = 0
    inaccessible = 0
    if not args.dir.is_dir():
        print('Evidence unavailable: diagnostic directory does not exist or is inaccessible.', file=sys.stderr)
        return 2
    for path in sorted(args.dir.glob('nr-*.jsonl')):
        if path.is_symlink():
            inaccessible += 1
            continue
        try:
            with path.open('rb') as stream:
                line_no = 0
                while True:
                    raw = stream.readline(16385)
                    if not raw:
                        break
                    line_no += 1
                    if len(raw) > 16384:
                        malformed += 1
                        while raw and not raw.endswith(b'\n'):
                            raw = stream.readline(16385)
                        continue
                    try:
                        row = json.loads(raw)
                        if (not isinstance(row, dict) or row.get('schema') != 1
                                or not isinstance(row.get('run'), str)
                                or not isinstance(row.get('event'), str)
                                or not isinstance(row.get('ts_ms'), int)):
                            raise ValueError('unsupported event schema')
                        if row['event'] != 'segment_start' and (
                                not isinstance(row.get('seq'), int) or not isinstance(row.get('data'), dict)):
                            raise ValueError('invalid event fields')
                        run = row['run']
                        if args.run and run != args.run:
                            continue
                        info = runs.setdefault(run, dict(run=run, events=collections.Counter(), sequences=[],
                            segments=[], unwinding=False, dropped_total=0, io_errors_total=0, first_ms=row['ts_ms'], last_ms=row['ts_ms'], metadata=None))
                        info['first_ms'] = min(info['first_ms'], row['ts_ms'])
                        info['last_ms'] = max(info['last_ms'], row['ts_ms'])
                        info['events'][row['event']] += 1
                        if row['event'] == 'process_end':
                            info['unwinding'] = bool(row.get('data', {}).get('unwinding'))
                        if row['event'] == 'segment_start':
                            info['segments'].append(row.get('segment', 0))
                            info['metadata'] = row
                        else:
                            if not isinstance(row.get('seq'), int) or not isinstance(row.get('data'), dict):
                                raise ValueError('invalid event fields')
                            info['sequences'].append(row['seq'])
                            for key in ('dropped_total', 'io_errors_total'):
                                info[key] = max(info[key], int(row.get(key, 0)))
                        if args.since is not None and row['ts_ms'] < args.since:
                            continue
                        if args.until is not None and row['ts_ms'] > args.until:
                            continue
                        if args.event and row['event'] not in args.event:
                            continue
                        data = row.get('data', {})
                        if args.operation is not None and args.operation not in [row.get('operation')] + [
                                data.get(k) for k in ('parent', 'session', 'worker', 'command', 'old_session')]:
                            continue
                        matches += 1
                        if len(selected) < args.limit:
                            selected.append(dict(row, _source=f'{path.name}:{line_no}'))
                    except (ValueError, TypeError, KeyError, UnicodeError):
                        malformed += 1
        except OSError:
            inaccessible += 1
    if args.summary:
        emitted = 0
        for info in runs.values():
            if args.since is not None and info['last_ms'] < args.since:
                continue
            if args.until is not None and info['first_ms'] > args.until:
                continue
            seq = sorted(set(info.pop('sequences')))
            info['sequence_gaps'] = sum(b - a - 1 for a, b in zip(seq, seq[1:]))
            info['first_sequence'] = seq[0] if seq else None
            info['last_sequence'] = seq[-1] if seq else None
            info['process_end_observed'] = bool(info['events']['process_end'])
            info['clean_end_observed'] = info['process_end_observed'] and not info['unwinding']
            info['warnings'] = []
            if not info['process_end_observed']:
                info['warnings'].append('No process_end: still running, abrupt exit, lost events, or retention; cause unknown.')
            if info['first_sequence'] not in (None, 1) or (info['segments'] and min(info['segments']) > 0):
                info['warnings'].append('Earlier evidence is absent (rotation, expiration, deletion or loss).')
            if info['sequence_gaps'] or info['dropped_total'] or info['io_errors_total']:
                info['warnings'].append('Evidence is incomplete; inspect gaps and loss counters.')
            if info['events'].get('panic'):
                info['warnings'].append('Panic observed: inspect location and operation; payload intentionally omitted.')
            info['warnings'].append('Attempt/update details are sampled. No exact action replay or unsaved policy recovery.')
            print(json.dumps(info, sort_keys=True))
            emitted += 1
        matches = emitted
    else:
        for row in selected:
            print(json.dumps(row, sort_keys=True))
    print(json.dumps(dict(matching_records=matches, emitted=min(matches, args.limit) if not args.summary else matches,
        malformed_or_partial_lines=malformed, inaccessible_files=inaccessible,
        warning='No match is not proof of no failure. Retention is bounded; events and shutdown tails may be missing.')), file=sys.stderr)
    if not args.summary and matches > args.limit:
        print('Output limited; narrow the time range or raise --limit. File order is not causal order; use run/seq and command IDs.', file=sys.stderr)
    return 0 if matches and not malformed and not inaccessible else 2


if __name__ == '__main__':
    sys.exit(main())
