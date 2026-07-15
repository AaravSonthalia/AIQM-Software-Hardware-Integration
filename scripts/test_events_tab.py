"""Unit tests for gui/events_tab.py — construction + attach_session
lifecycle (D2 Day 8 sprint kickoff).

Locks the core surface of the Events tab: master table shape, detail
pane placeholder, label form presence, and the attach_session lifecycle
(clears table, reloads cache, handles None). More coverage — row
insertion, label-form activation slots, unreviewed-badge — lands in
Day 9's follow-on test classes; this file is the construction +
lifecycle foundation those will extend.

Run:
    QT_QPA_PLATFORM=offscreen python scripts/test_events_tab.py
"""
from __future__ import annotations

import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# QApplication precedes any QWidget. Same pattern as
# test_scrubber_tab.py and test_live_equalizer_tab.py.
from PyQt6.QtWidgets import QApplication  # noqa: E402
_app = QApplication.instance() or QApplication(sys.argv)  # noqa: F841

from gui.events_tab import (  # noqa: E402
    COLUMN_HEADERS,
    EventsTab,
)
from gui.growth_logger import GrowthLogger  # noqa: E402


class EventsTabConstructionTests(unittest.TestCase):
    """Five tests locking the widget's structural shape at construction."""

    def setUp(self):
        self.tab = EventsTab()

    def tearDown(self):
        self.tab.deleteLater()

    def test_builds_cleanly_without_session(self):
        # Construction must not require a session. GrowthMonitor
        # instantiates the tab before any session is armed, so a
        # bare EventsTab() has to be a valid state.
        self.assertIsNone(self.tab._session_dir)
        self.assertIsNone(self.tab._growth_logger)
        self.assertEqual(self.tab._last_seen_event_idx, 0)
        self.assertEqual(self.tab._labels_cache, {})
        self.assertIsNone(self.tab._currently_displayed_event_idx)

    def test_master_table_column_count(self):
        # 5 columns per COLUMN_HEADERS. If a new column is added,
        # this test flags the schema change so table-index constants
        # (COL_EVENT_IDX etc.) can be reviewed alongside.
        self.assertEqual(len(COLUMN_HEADERS), 5)
        self.assertEqual(
            self.tab.events_table.columnCount(), 5,
        )
        # Header labels round-trip
        header = self.tab.events_table.horizontalHeader()
        for i, expected in enumerate(COLUMN_HEADERS):
            item = self.tab.events_table.horizontalHeaderItem(i)
            if item is not None:
                self.assertEqual(item.text(), expected)

    def test_detail_placeholder_visible_initially(self):
        # No event selected → placeholder shown, content hidden.
        # Uses isHidden() rather than isVisible() because offscreen
        # Qt reports isVisible=False for any child of a non-shown
        # parent — isHidden() reflects explicit hide() calls, which
        # is the design contract we actually care about.
        self.assertFalse(self.tab._detail_placeholder.isHidden())
        self.assertTrue(self.tab._detail_content.isHidden())
        self.assertIn(
            "Select an event", self.tab._detail_placeholder.text(),
        )

    def test_label_form_widgets_present(self):
        # The three labeling combos + notes field are load-bearing
        # attribute names — attach_session's _populate_label_form,
        # the _on_*_activated slots, and Day 3's dropdowns all
        # reach into these by name.
        for attr in (
            "_primary_recon_combo",
            "_change_from_combo",
            "_change_to_combo",
            "_notes_input",
        ):
            self.assertTrue(
                hasattr(self.tab, attr),
                msg=f"missing labelform widget: {attr}",
            )

    def test_unreviewed_count_signal_registered(self):
        # unreviewed_count_changed is consumed by GrowthMonitor's
        # events-tab-badge painter. Losing the signal name would
        # silently break the badge — regression guard here.
        self.assertTrue(hasattr(self.tab, "unreviewed_count_changed"))
        # Signal can be connected + emitted without exceptions
        received: list[int] = []
        self.tab.unreviewed_count_changed.connect(received.append)
        self.tab.unreviewed_count_changed.emit(7)
        self.assertEqual(received, [7])


class EventsTabAttachSessionTests(unittest.TestCase):
    """Six tests locking the attach_session lifecycle."""

    def setUp(self):
        self.tab = EventsTab()
        self.tmp = tempfile.TemporaryDirectory()
        self.logger = GrowthLogger(base_dir=self.tmp.name)
        self.logger.start_session("D2_TEST")

    def tearDown(self):
        self.tab.deleteLater()
        try:
            self.logger.end_session()
        except Exception:
            pass
        self.tmp.cleanup()

    def _write_labels_csv(self, rows: list[dict]):
        """Helper: write events_labels.csv with the given rows before
        attach so we can verify cache repopulation."""
        csv_path = self.logger.session_dir / "events_labels.csv"
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(
                f, fieldnames=GrowthLogger.EVENT_LABEL_FIELDS,
            )
            w.writeheader()
            for row in rows:
                w.writerow(row)

    def test_attach_populates_session_dir_and_logger(self):
        # After attach, both refs are set + point to the correct
        # session. Downstream slots (labeling writes) rely on both
        # being non-None.
        self.tab.attach_session(self.logger)
        self.assertIs(self.tab._growth_logger, self.logger)
        self.assertEqual(
            self.tab._session_dir, self.logger.session_dir,
        )

    def test_attach_clears_table_from_prior_session(self):
        # Simulate a prior session having populated the table (rows
        # from a previous session leaking into a new one would be
        # a UI-truth failure). attach_session must clear.
        self.tab.events_table.setRowCount(3)
        self.assertEqual(self.tab.events_table.rowCount(), 3)
        self.tab.attach_session(self.logger)
        self.assertEqual(self.tab.events_table.rowCount(), 0)

    def test_attach_reloads_label_cache_from_csv(self):
        # Pre-write a label to the logger's session_dir. attach
        # must re-read that CSV into _labels_cache so the labeling
        # form can prefill existing labels when the grower selects
        # a previously-labeled event.
        self._write_labels_csv([
            {
                "event_idx": "5",
                "primary_reconstruction": "1x1",
                "change_from": "",
                "change_to": "",
                "notes": "prior session note",
                "label_timestamp_iso": "2026-07-22T10:00:00",
                "recon_1x1": "",
                "recon_tw": "",
                "recon_c6x2": "",
                "recon_rt13": "",
                "recon_HTR": "",
            },
        ])
        self.tab.attach_session(self.logger)
        self.assertIn(5, self.tab._labels_cache)
        self.assertEqual(
            self.tab._labels_cache[5]["primary_reconstruction"],
            "1x1",
        )
        self.assertEqual(
            self.tab._labels_cache[5]["notes"],
            "prior session note",
        )

    def test_attach_with_no_labels_csv_yields_empty_cache(self):
        # Fresh session, no CSV → cache is an empty dict, not
        # None (downstream .get(idx) calls assume dict).
        self.tab.attach_session(self.logger)
        self.assertEqual(self.tab._labels_cache, {})
        self.assertIsInstance(self.tab._labels_cache, dict)

    def test_attach_none_clears_state(self):
        # attach(None) is the "session ended" cleanup path from
        # GrowthApp._on_stop. Both refs must be nulled + cache
        # must clear.
        self.tab.attach_session(self.logger)
        self.assertIsNotNone(self.tab._growth_logger)

        self.tab.attach_session(None)
        self.assertIsNone(self.tab._growth_logger)
        self.assertIsNone(self.tab._session_dir)
        self.assertEqual(self.tab._labels_cache, {})

    def test_attach_resets_currently_displayed_event_idx(self):
        # If the grower had an event selected when the previous
        # session ended, attach_session must reset the cursor so
        # form-write slots don't write against a stale event_idx
        # under the new session's CSV.
        self.tab.attach_session(self.logger)
        self.tab._currently_displayed_event_idx = 42
        self.tab.attach_session(self.logger)  # re-attach same logger
        self.assertIsNone(
            self.tab._currently_displayed_event_idx,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
