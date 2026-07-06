"""Canonical 5-class RHEED reconstruction labels.

Single source of truth for reconstruction label strings across the
Growth Monitor GUI. Prevents string drift between the classifier
bridge, the main-tab sliders, the events tab, and any future component
that needs to talk about the same five classes.

Canonical spellings (per AJ's convention, see feedback_recon_naming
memory 2026-05-07):

    - ASCII 'x' (not Unicode '×')
    - lowercase 'rt13' (not 'RT13')
    - full 'Twinned (2x1)' (not the abbreviated 'Tw(2x1)')
    - no parens around '1x1'
    - 'rt13xrt13' spells √13 × √13 as ASCII

New code should import ``RECON_LABELS`` from here. The bridge re-exports
under its old ``GUI_LABELS`` name as an alias so existing importers
(scripts/classifier_demo.py) keep working.

The Equalizer scripts (scripts/equalizer_ui.py,
scripts/equalizer_svd_prototype.py) currently use their own
non-canonical labels ('Tw(2x1)', 'RT13'). They live in a separate label
domain and will migrate independently — do not fold them in here until
the Equalizer side is updated in the same commit.
"""
from __future__ import annotations


# Canonical five-class label order. Any UI that lists all five classes
# should preserve this order for consistency across tabs.
RECON_LABELS: list[str] = ["1x1", "Twinned (2x1)", "c(6x2)", "rt13xrt13", "HTR"]


# Classifier2's internal label strings (as embedded in best_model.pth)
# mapped to the canonical GUI labels above. Update the left-hand side
# only if Justin ships a checkpoint with different string keys.
CLASSIFIER2_TO_GUI: dict[str, str] = {
    "(1 x 1)":         "1x1",
    "Twinned(2 x 1)":  "Twinned (2x1)",
    "c(6 x 2)":        "c(6x2)",
    "(√13 x √13)":     "rt13xrt13",
    "HTR":             "HTR",
}
