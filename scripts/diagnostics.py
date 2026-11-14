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


REFERENCE_FIELDS = ('parent', 'session', 'worker', 'command', 'old_session')


def records(stream):
    """Bound line reads and require the writer's newline commit delimiter."""
    line_no = 0
    while raw := stream.readline(16385):
        line_no += 1
        if len(raw) > 16384:
            while raw and not raw.endswith(b'\n'):
                raw = stream.readline(16385)
            yield line_no, None
            continue
        try:
            if not raw.endswith(b'\n'):
                raise ValueError('partial record')
            row = json.loads(raw)
            if (not isinstance(row, dict) or type(row.get('schema')) is not int
                    or row['schema'] != 1 or not isinstance(row.get('run'), str)
                    or not isinstance(row.get('event'), str)
                    or type(row.get('ts_ms')) is not int):
                raise ValueError('unsupported record')
            if row['event'] == 'segment_start':
                if type(row.get('segment')) is not int or row['segment'] < 0:
                    raise ValueError('invalid segment')
            elif (type(row.get('seq')) is not int or row['seq'] <= 0
                    or not isinstance(row.get('data'), dict)
                    or any(type(row.get(key, 0)) is not int or row.get(key, 0) < 0
                           for key in ('dropped_total', 'io_errors_total'))
                    or (row['event'] == 'process_end'
                        and type(row['data'].get('unwinding')) is not bool)):
                raise ValueError('invalid event')
            yield line_no, row
        except (ValueError, UnicodeError, RecursionError):
            yield line_no, None


def summarize(info):
    sequences = info.pop('sequences')
    seq = sorted(set(sequences))
    info['sequence_gaps'] = sum(b - a - 1 for a, b in zip(seq, seq[1:]))
    info['duplicate_sequences'] = len(sequences) - len(seq)
    info['first_sequence'] = seq[0] if seq else None
    info['last_sequence'] = seq[-1] if seq else None
    info['segments'] = sorted(set(info['segments']))
    info['missing_segments_between_retained'] = sum(
        b - a - 1 for a, b in zip(info['segments'], info['segments'][1:]))
    info['process_end_observed'] = bool(info['events']['process_end'])
    info['clean_end_observed'] = info['process_end_observed'] and not info['unwinding']
    warnings = info['warnings'] = []
    if not info['process_end_observed']:
        warnings.append('No process_end: still running, abrupt exit, lost events, or retention; cause unknown.')
    if info['first_sequence'] not in (None, 1) or (info['segments'] and info['segments'][0] > 0):
        warnings.append('Earlier evidence is absent (rotation, expiration, deletion or loss).')
    if (not info['metadata'] or info['sequence_gaps'] or info['duplicate_sequences']
            or info['missing_segments_between_retained'] or info['dropped_total'] or info['io_errors_total']):
        warnings.append('Evidence is incomplete or inconsistent; inspect gaps and loss counters.')
    if info['events'].get('panic'):
        warnings.append('Panic observed: inspect location and operation; payload intentionally omitted.')
    warnings.append('Attempt/update details are sampled. No exact action replay or unsaved policy recovery.')
    return info


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
    parser.add_argument('--offset', type=int, default=0, help='skip matches (page a stable evidence copy)')
    args = parser.parse_args(argv)
    if args.offset < 0:
        parser.error('--offset must be nonnegative')
    if not 1 <= args.limit <= 10000:
        parser.error('--limit must be between 1 and 10000')
    if args.since is not None and args.until is not None and args.since > args.until:
        parser.error('--since must not exceed --until')
    if args.summary and (args.operation is not None or args.event):
        parser.error('--summary describes retained runs; use event/operation filters without --summary')
    runs = {}
    matches = emitted = malformed = opened = 0
    first_ms = last_ms = None
    unavailable = []
    directory_error = None

    def output(row):
        nonlocal matches, emitted
        matches += 1
        if args.offset < matches <= args.offset + args.limit:
            print(json.dumps(row, sort_keys=True))
            emitted += 1

    try:
        # Explicit enumeration: Path.glob may suppress directory access errors.
        paths = sorted(p for p in args.dir.iterdir()
                       if p.name.startswith('nr-') and p.name.endswith('.jsonl'))
    except OSError as exc:
        paths = []
        directory_error = type(exc).__name__

    for path in paths:
        if args.run and not path.name.startswith(f'nr-{args.run}-'):
            continue
        try:
            if path.is_symlink() or (path.exists() and not path.is_file()):
                unavailable.append(dict(file=path.name, reason='not_regular_file'))
                continue
            with path.open('rb') as stream:
                opened += 1
                for line_no, row in records(stream):
                    if row is None:
                        malformed += 1
                        continue
                    run = row['run']
                    if args.run and run != args.run:
                        continue
                    ts = row['ts_ms']
                    first_ms = ts if first_ms is None else min(ts, first_ms)
                    last_ms = ts if last_ms is None else max(ts, last_ms)
                    if args.summary:
                        info = runs.setdefault(run, dict(run=run, events=collections.Counter(),
                            sequences=[], segments=[], unwinding=False, dropped_total=0,
                            io_errors_total=0, first_ms=ts, last_ms=ts, metadata=None))
                        info['first_ms'] = min(info['first_ms'], ts)
                        info['last_ms'] = max(info['last_ms'], ts)
                        info['events'][row['event']] += 1
                        if row['event'] == 'process_end':
                            info['unwinding'] |= row['data']['unwinding']
                        if row['event'] == 'segment_start':
                            info['segments'].append(row['segment'])
                            info['metadata'] = row
                        else:
                            info['sequences'].append(row['seq'])
                            for key in ('dropped_total', 'io_errors_total'):
                                info[key] = max(info[key], row.get(key, 0))
                        continue
                    if args.since is not None and ts < args.since:
                        continue
                    if args.until is not None and ts > args.until:
                        continue
                    if args.event and row['event'] not in args.event:
                        continue
                    if args.operation is not None:
                        data = row.get('data', {})
                        refs = [row.get('operation')] + [data.get(key) for key in REFERENCE_FIELDS]
                        if not any(type(ref) is int and ref == args.operation for ref in refs):
                            continue
                    output(dict(row, _source=f'{path.name}:{line_no}'))
        except OSError as exc:
            unavailable.append(dict(file=path.name, reason=type(exc).__name__))
    for info in runs.values():
        # Include runs with a retained time span intersecting the requested window.
        if args.since is not None and info['last_ms'] < args.since:
            continue
        if args.until is not None and info['first_ms'] > args.until:
            continue
        output(summarize(info))

    scan_incomplete = bool(directory_error or unavailable or malformed)
    next_offset = args.offset + emitted if matches > args.offset + emitted else None
    status = ('unavailable' if directory_error or (unavailable and not opened)
              else 'partial' if scan_incomplete else 'matched' if matches else 'no_recorded_match')
    print(json.dumps(dict(query_schema=1, status=status, directory=str(args.dir.absolute()),
        since_ms=args.since, until_ms=args.until, run=args.run, operation=args.operation,
        event=args.event, summary=args.summary, matching_records=matches, emitted=emitted,
        first_retained_ms=first_ms, last_retained_ms=last_ms, opened_files=opened,
        directory_error=directory_error, inaccessible_files=len(unavailable),
        unavailable_sources=unavailable, malformed_or_partial_lines=malformed,
        offset=args.offset, limit=args.limit, next_offset=next_offset,
        warning='This query searches local retained files only. A missing event may be sampled, dropped, rotated or never recorded. File order is not causal order.'),
        sort_keys=True), file=sys.stderr)
    return 0 if emitted and not scan_incomplete else 2


if __name__ == '__main__':
    sys.exit(main())
