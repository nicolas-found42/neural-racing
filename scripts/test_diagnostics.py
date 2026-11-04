import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('diagnostics', Path(__file__).with_name('diagnostics.py'))
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


class RetrievalTests(unittest.TestCase):
    def query(self, root, *args):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = diag.main(['--dir', str(root), *args])
        return code, [json.loads(line) for line in out.getvalue().splitlines()], err.getvalue()

    def test_partial_tail_time_filter_operation_and_loss_summary(self):
        with tempfile.TemporaryDirectory() as root:
            records = [dict(schema=1, run='r', event='segment_start', segment=2, ts_ms=1000),
                       dict(schema=1, run='r', event='command_received', seq=5, ts_ms=1100,
                            operation=9, data=dict(worker=2), dropped_total=3),
                       dict(schema=1, run='r', event='live_error', seq=7, ts_ms=1200,
                            operation=2, data=dict(reason='synthetic'), dropped_total=3)]
            path = Path(root) / 'nr-r-000002.jsonl'
            path.write_text(''.join(json.dumps(row) + '\n' for row in records) + '{partial')
            code, rows, err = self.query(root, '--summary')
            self.assertEqual(code, 2)
            self.assertEqual(rows[0]['sequence_gaps'], 1)
            self.assertFalse(rows[0]['clean_end_observed'])
            self.assertEqual(rows[0]['dropped_total'], 3)
            self.assertIn('"malformed_or_partial_lines": 1', err)
            code, rows, _ = self.query(root, '--since', '1150', '--until', '1250', '--operation', '2')
            self.assertEqual([r['event'] for r in rows], ['live_error'])
            code, rows, _ = self.query(root, '--since', '2000')
            self.assertEqual(code, 2)
            self.assertEqual(rows, [])

    def test_corrupt_end_marker_cannot_claim_clean_shutdown(self):
        with tempfile.TemporaryDirectory() as root:
            rows = [dict(schema=1, run='r', event='segment_start', segment=0, ts_ms=1000),
                    dict(schema=1, run='r', event='process_end', seq=1, ts_ms=1100, data=[])]
            (Path(root) / 'nr-r-000000.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
            code, result, err = self.query(root, '--summary')
            self.assertEqual(code, 2)
            self.assertFalse(result[0]['clean_end_observed'])
            self.assertIn('"malformed_or_partial_lines": 1', err)

    def test_no_directory_is_unknown_and_timezone_is_required(self):
        code, rows, err = self.query('/nonexistent/neural-racing-evidence')
        self.assertEqual(code, 2)
        self.assertIn('unavailable', err)
        self.assertEqual(diag.timestamp('1970-01-01T00:00:01Z'), 1000)
        with self.assertRaises(Exception):
            diag.timestamp('2026-11-04T12:00:00')


if __name__ == '__main__':
    unittest.main()
