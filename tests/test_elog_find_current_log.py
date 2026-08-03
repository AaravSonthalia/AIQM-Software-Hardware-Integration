#!/usr/bin/env python3
"""Tests for ``drivers.elog.find_current_log`` session-file selection.

Regression suite for the Jul 30 2026 O-MBE outage: the function hardcoded
the ``_000000`` time suffix, so every day EvapControl was started mid-day
it returned None and the entire elog direct-read path went dark while the
data sat on disk. See the docstring on ``find_current_log`` for the full
account.

These tests are filesystem-only — no EvapControl, no chamber, no .elo
parsing. They build empty files with the right *names* in a tmpdir, which
is all the selection logic looks at.
"""
import datetime
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from drivers.elog import find_current_log


def _today() -> str:
    return datetime.date.today().isoformat()


class FindCurrentLogTests(unittest.TestCase):
    def setUp(self):
        self._tmp = TemporaryDirectory()
        self.dir = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)

    def _touch(self, name: str) -> Path:
        p = self.dir / name
        p.write_bytes(b"")
        return p

    # --- the original behaviour, preserved -----------------------------

    def test_finds_midnight_rotation_file(self):
        """The uninterrupted-day case that always worked must keep working."""
        expected = self._touch(f"log_{_today()}_000000.elo")
        self.assertEqual(find_current_log(self.dir), expected)

    # --- the bug this suite exists for ---------------------------------

    def test_finds_mid_day_start_file(self):
        """log_<today>_141157.elo — the Jul 30 2026 failure. Was None."""
        expected = self._touch(f"log_{_today()}_141157.elo")
        self.assertEqual(find_current_log(self.dir), expected)

    def test_returns_newest_when_evapcontrol_restarted(self):
        """61 days in Bulbasaur's log dir have multiple session files.

        Returning the first match would hand back a stale completed log
        whose values still look plausible — a silent wrong answer, worse
        than the visible failure it replaces.
        """
        self._touch(f"log_{_today()}_154324.elo")
        self._touch(f"log_{_today()}_155226.elo")
        newest = self._touch(f"log_{_today()}_163753.elo")
        self.assertEqual(find_current_log(self.dir), newest)

    def test_newest_beats_midnight_file_after_restart(self):
        """A restart after a midnight rotation: _000000 is the stale one."""
        self._touch(f"log_{_today()}_000000.elo")
        newest = self._touch(f"log_{_today()}_141157.elo")
        self.assertEqual(find_current_log(self.dir), newest)

    # --- things that must NOT be selected -------------------------------

    def test_ignores_zipped_sessions(self):
        """A .elo.zip for today has already been rotated away from.

        _open_elo can read zips, but that is for historical logs; the file
        being actively written is never the compressed one.
        """
        self._touch(f"log_{_today()}_141157.elo.zip")
        self.assertIsNone(find_current_log(self.dir))

    def test_prefers_uncompressed_over_newer_zip(self):
        live = self._touch(f"log_{_today()}_141157.elo")
        self._touch(f"log_{_today()}_235959.elo.zip")
        self.assertEqual(find_current_log(self.dir), live)

    def test_ignores_other_days(self):
        yesterday = (
            datetime.date.today() - datetime.timedelta(days=1)
        ).isoformat()
        self._touch(f"log_{yesterday}_000000.elo")
        self._touch(f"log_{yesterday}_141157.elo")
        self.assertIsNone(find_current_log(self.dir))

    def test_ignores_error_logs(self):
        """The same directory holds error_<date>_<time>.txt files."""
        self._touch(f"error_{_today()}_141157.txt")
        self.assertIsNone(find_current_log(self.dir))

    # --- malformed names ------------------------------------------------

    def test_ignores_non_numeric_time_suffix(self):
        self._touch(f"log_{_today()}_backup.elo")
        self.assertIsNone(find_current_log(self.dir))

    def test_ignores_wrong_width_time_suffix(self):
        self._touch(f"log_{_today()}_1411.elo")
        self._touch(f"log_{_today()}_1411570.elo")
        self.assertIsNone(find_current_log(self.dir))

    def test_ignores_empty_time_suffix(self):
        self._touch(f"log_{_today()}_.elo")
        self.assertIsNone(find_current_log(self.dir))

    def test_malformed_name_cannot_outrank_valid_session(self):
        """The reason the width check exists rather than being cosmetic.

        ASCII letters sort above digits, so a bare glob + sorted()[-1]
        would rank 'backup' over '141157' and serve a non-session file
        as live chamber data.
        """
        live = self._touch(f"log_{_today()}_141157.elo")
        self._touch(f"log_{_today()}_backup.elo")
        self.assertEqual(find_current_log(self.dir), live)

    def test_ignores_directory_with_matching_name(self):
        """A directory named like a session file is not a log."""
        (self.dir / f"log_{_today()}_141157.elo").mkdir()
        found = find_current_log(self.dir)
        self.assertTrue(found is None or found.is_file())

    # --- degenerate inputs ----------------------------------------------

    def test_empty_dir_returns_none(self):
        self.assertIsNone(find_current_log(self.dir))

    def test_missing_dir_returns_none(self):
        """A wrong install path must return None, not raise.

        ElogReader.connect() converts None into a RuntimeError carrying the
        directory it tried; an exception escaping from here instead would
        lose that message.
        """
        self.assertIsNone(find_current_log(self.dir / "nope"))

    def test_accepts_str_path(self):
        expected = self._touch(f"log_{_today()}_141157.elo")
        self.assertEqual(find_current_log(str(self.dir)), expected)


if __name__ == "__main__":
    unittest.main()
