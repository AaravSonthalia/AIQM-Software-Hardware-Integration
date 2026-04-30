#!/usr/bin/env python3
"""Generate a polished PDF of the RHEED ML Literature Comparison Table."""

from reportlab.lib.pagesizes import letter, landscape
from reportlab.lib.units import inch, mm
from reportlab.lib.colors import HexColor, white, black
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate, Table, TableStyle, Paragraph, Spacer,
    KeepTogether, HRFlowable
)
from reportlab.lib.enums import TA_LEFT, TA_CENTER
from pathlib import Path

# --- Color palette ---
DARK_BG = HexColor("#1a1a2e")
HEADER_BG = HexColor("#16213e")
ROW_EVEN = HexColor("#f8f9fa")
ROW_ODD = HexColor("#ffffff")
ACCENT = HexColor("#0f3460")
ACCENT_LIGHT = HexColor("#e8edf3")
HIGHLIGHT = HexColor("#1a7a4c")
HIGHLIGHT_BG = HexColor("#e6f4ed")
BORDER = HexColor("#c8cdd3")
MUTED = HexColor("#6b7280")
TITLE_COLOR = HexColor("#0f3460")

OUTPUT = Path(__file__).parent / "literature_comparison.pdf"


def build_styles():
    title = ParagraphStyle(
        "Title", fontName="Helvetica-Bold", fontSize=18,
        textColor=TITLE_COLOR, spaceAfter=4, alignment=TA_LEFT,
    )
    subtitle = ParagraphStyle(
        "Subtitle", fontName="Helvetica", fontSize=10,
        textColor=MUTED, spaceAfter=14, alignment=TA_LEFT,
    )
    section = ParagraphStyle(
        "Section", fontName="Helvetica-Bold", fontSize=13,
        textColor=TITLE_COLOR, spaceBefore=18, spaceAfter=8,
    )
    cell = ParagraphStyle(
        "Cell", fontName="Helvetica", fontSize=7.5,
        leading=9.5, textColor=HexColor("#1f2937"),
    )
    cell_bold = ParagraphStyle(
        "CellBold", fontName="Helvetica-Bold", fontSize=7.5,
        leading=9.5, textColor=HexColor("#111827"),
    )
    cell_header = ParagraphStyle(
        "CellHeader", fontName="Helvetica-Bold", fontSize=8,
        leading=10, textColor=white, alignment=TA_CENTER,
    )
    cell_highlight = ParagraphStyle(
        "CellHighlight", fontName="Helvetica-Bold", fontSize=7.5,
        leading=9.5, textColor=HIGHLIGHT,
    )
    body = ParagraphStyle(
        "Body", fontName="Helvetica", fontSize=9,
        leading=12, textColor=HexColor("#374151"), spaceAfter=6,
    )
    return dict(
        title=title, subtitle=subtitle, section=section,
        cell=cell, cell_bold=cell_bold, cell_header=cell_header,
        cell_highlight=cell_highlight, body=body,
    )


def make_main_table(s):
    """Main literature comparison table."""
    headers = ["Work", "Problem", "Phase /\nRecon?", "Method", "Real-time?",
               "Auto-\ncapture?", "Usefulness"]

    rows_data = [
        [
            "<b>RHAAPSODY</b><br/>(PNNL, 2023)",
            "Unsupervised phase discovery in RHEED video",
            "Phase (unsupervised)",
            "VGG16 embeddings &rarr; custom similarity-matrix changepoint detection &rarr; Louvain clustering",
            "Post-hoc only",
            "Offline changepoints",
            "<b>Partial</b> &mdash; concept useful; TF-locked, custom changepoint method. Reimplement with our SimCLR encoder.",
        ],
        [
            "<b>rhana</b><br/>(LHT, 2022)",
            "Quantitative RHEED analysis toolkit",
            "Neither &mdash; signal processing",
            "Rolling ball background subtraction, FFT PeriodicityTracker, U-Net segmentation",
            "Yes (streaming)",
            "No",
            "<b>High</b> &mdash; PeriodicityTracker for oscillations; rolling ball for noise; PyTorch compatible.",
        ],
        [
            "<b>Classifier2</b><br/>(Ours, 2025-26)",
            "Classify STO surface reconstructions",
            "Reconstruction (5-class supervised)",
            "Bradley-Terry reward model on SimCLR ResNet18; pairwise human preferences (RLHF-style)",
            "Real-time inference (~80 ms CPU)",
            "Tier 3 trigger (JS-divergence)",
            "<b>Core</b> &mdash; 100% ideal-split accuracy. Integrated via ClassifierBridge.",
        ],
        [
            "<b>Rahim NMF</b><br/>(Yang Lab thesis)",
            "Unsupervised RHEED pattern decomposition",
            "Phase (unsupervised)",
            "NMF (3 components) + DBSCAN / K-means on STO RHEED video",
            "Offline",
            "No",
            "<b>Low</b> &mdash; baseline unsupervised comparison only.",
        ],
        [
            "<b>PyRHEED</b><br/>(Yu, 2020)",
            "Quantitative streak analysis",
            "Neither &mdash; peak fitting",
            "Gaussian / Voigt peak fitting on RHEED streak profiles",
            "No (interactive)",
            "No",
            "<b>Future</b> &mdash; streak width metrics for quality assessment.",
        ],
        [
            "<b>FRHEED</b><br/>(Young, 2023)",
            "Live RHEED monitoring GUI",
            "Neither &mdash; visualization",
            "Threading + ROI selection + FFT oscillation tracking",
            "Yes (live GUI)",
            "No (manual)",
            "<b>Low</b> &mdash; architecture reference; our GUI already has these features.",
        ],
        [
            "<b>RHEED_2D_ML</b><br/>(Chang, 2021)",
            "Unsupervised pattern clustering",
            "Phase (unsupervised)",
            "PCA + K-means on 2D RHEED video frames",
            "Offline",
            "No",
            "<b>Low</b> &mdash; PCA vs NMF baseline.",
        ],
        [
            "<b>RHEED Viewer</b><br/>(Jacques, Yang Lab)",
            "Live RHEED + pyrometer monitoring",
            "Neither &mdash; data acquisition",
            "ROI intensity tracking + pywinauto pyrometer scraping",
            "Yes (live)",
            "No (manual)",
            "<b>Predecessor</b> &mdash; pyrometer scrape pattern inspired our serial driver.",
        ],
    ]

    header_row = [Paragraph(h.replace("\n", "<br/>"), s["cell_header"]) for h in headers]

    data = [header_row]
    for row in rows_data:
        styled_row = [
            Paragraph(row[0], s["cell_bold"]),
            Paragraph(row[1], s["cell"]),
            Paragraph(row[2], s["cell"]),
            Paragraph(row[3], s["cell"]),
            Paragraph(row[4], s["cell"]),
            Paragraph(row[5], s["cell"]),
            Paragraph(row[6], s["cell"]),
        ]
        data.append(styled_row)

    col_widths = [1.05*inch, 1.15*inch, 0.85*inch, 2.0*inch, 0.8*inch, 0.7*inch, 2.2*inch]

    tbl = Table(data, colWidths=col_widths, repeatRows=1)
    style = TableStyle([
        # Header
        ("BACKGROUND", (0, 0), (-1, 0), ACCENT),
        ("TEXTCOLOR", (0, 0), (-1, 0), white),
        ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
        ("FONTSIZE", (0, 0), (-1, 0), 8),
        ("BOTTOMPADDING", (0, 0), (-1, 0), 8),
        ("TOPPADDING", (0, 0), (-1, 0), 8),
        ("ALIGN", (0, 0), (-1, 0), "CENTER"),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        # Body
        ("FONTNAME", (0, 1), (-1, -1), "Helvetica"),
        ("FONTSIZE", (0, 1), (-1, -1), 7.5),
        ("TOPPADDING", (0, 1), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 1), (-1, -1), 5),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        # Alternating rows
        *[("BACKGROUND", (0, i), (-1, i), ROW_EVEN if i % 2 == 0 else ROW_ODD)
          for i in range(1, len(data))],
        # Highlight row 3 (Classifier2 = ours)
        ("BACKGROUND", (0, 3), (-1, 3), HIGHLIGHT_BG),
        # Grid
        ("GRID", (0, 0), (-1, -1), 0.5, BORDER),
        ("LINEBELOW", (0, 0), (-1, 0), 1.5, ACCENT),
    ])
    tbl.setStyle(style)
    return tbl


def make_tools_table(s):
    """Additional tools discovered."""
    headers = ["Tool", "Domain", "Relevance"]
    rows_data = [
        ["changepoint-online / NPFocus", "Online changepoint detection",
         "Streaming changepoint detection for Tier 2 &mdash; designed for live data"],
        ["STUMPY (FLOSS)", "Time series anomaly detection",
         "Alternative to PELT for detecting RHEED pattern transitions in real-time"],
        ["BOCD", "Probabilistic changepoint detection",
         "Most principled approach &mdash; posterior probability of changepoint at each timestep"],
        ["PC-Gym", "RL for process control",
         "Relevant to Anneal RL &mdash; environments for tuning RL on thermal processes"],
        ["HELAO-async / Bluesky", "Self-driving lab frameworks",
         "Architecture reference for autonomous experiment orchestration"],
        ["Alibi Detect", "Distribution drift detection",
         "Detect when RHEED patterns drift outside training distribution"],
    ]

    header_row = [Paragraph(h, s["cell_header"]) for h in headers]
    data = [header_row]
    for row in rows_data:
        data.append([
            Paragraph(f"<b>{row[0]}</b>", s["cell_bold"]),
            Paragraph(row[1], s["cell"]),
            Paragraph(row[2], s["cell"]),
        ])

    col_widths = [2.0*inch, 1.8*inch, 4.95*inch]
    tbl = Table(data, colWidths=col_widths, repeatRows=1)
    tbl.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), ACCENT),
        ("TEXTCOLOR", (0, 0), (-1, 0), white),
        ("BOTTOMPADDING", (0, 0), (-1, 0), 7),
        ("TOPPADDING", (0, 0), (-1, 0), 7),
        ("ALIGN", (0, 0), (-1, 0), "CENTER"),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("TOPPADDING", (0, 1), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 1), (-1, -1), 4),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        *[("BACKGROUND", (0, i), (-1, i), ROW_EVEN if i % 2 == 0 else ROW_ODD)
          for i in range(1, len(data))],
        ("GRID", (0, 0), (-1, -1), 0.5, BORDER),
        ("LINEBELOW", (0, 0), (-1, 0), 1.5, ACCENT),
    ]))
    return tbl


def make_tier_table(s):
    """Three-tier auto-capture architecture."""
    headers = ["Tier", "Method", "Detector Class", "Status", "Novelty"]
    rows_data = [
        ["1", "Global mean intensity change", "IntensityChangeDetector", "DONE", "Standard (baseline)"],
        ["2", "SimCLR embedding cosine distance +\nonline changepoint detection",
         "EmbeddingChangeDetector\n(planned)", "TODO",
         "Adapts RHAAPSODY concept;\nuse BOCD or STUMPY"],
        ["3", "Classifier2 output distribution\nchange (JS-divergence)",
         "ClassificationChangeDetector", "DONE",
         "Novel \u2014 semantic auto-capture\ntriggered by reconstruction transitions"],
    ]

    header_row = [Paragraph(h, s["cell_header"]) for h in headers]
    data = [header_row]
    for row in rows_data:
        status_style = s["cell_highlight"] if row[3] == "DONE" else s["cell_bold"]
        data.append([
            Paragraph(f"<b>{row[0]}</b>", s["cell_bold"]),
            Paragraph(row[1].replace("\n", "<br/>"), s["cell"]),
            Paragraph(row[2].replace("\n", "<br/>"), s["cell"]),
            Paragraph(row[3], status_style),
            Paragraph(row[4].replace("\n", "<br/>"), s["cell"]),
        ])

    col_widths = [0.5*inch, 2.2*inch, 1.8*inch, 0.7*inch, 3.55*inch]
    tbl = Table(data, colWidths=col_widths, repeatRows=1)
    tbl.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), ACCENT),
        ("TEXTCOLOR", (0, 0), (-1, 0), white),
        ("BOTTOMPADDING", (0, 0), (-1, 0), 7),
        ("TOPPADDING", (0, 0), (-1, 0), 7),
        ("ALIGN", (0, 0), (0, -1), "CENTER"),
        ("ALIGN", (0, 0), (-1, 0), "CENTER"),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("TOPPADDING", (0, 1), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 1), (-1, -1), 4),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        # Highlight Tier 3 row
        ("BACKGROUND", (0, 3), (-1, 3), HIGHLIGHT_BG),
        *[("BACKGROUND", (0, i), (-1, i), ROW_EVEN if i % 2 == 0 else ROW_ODD)
          for i in range(1, 3)],
        ("GRID", (0, 0), (-1, -1), 0.5, BORDER),
        ("LINEBELOW", (0, 0), (-1, 0), 1.5, ACCENT),
    ]))
    return tbl


def make_recon_table(s):
    """STO surface reconstructions physics context."""
    headers = ["Reconstruction", "Conditions", "Significance"]
    rows_data = [
        ["(1x1) &mdash; Unreconstructed", "Low T, O-rich", "Starting / reference surface"],
        ["Twinned(2x1)", "~500-600 \u00b0C, reducing", "Ti-rich surface, pre-growth prep"],
        ["c(6x2)", "~600-700 \u00b0C, UHV", "Clean, well-ordered; good growth starting point"],
        ["(\u221a13 x \u221a13) R33.7\u00b0", "~700-900 \u00b0C, reducing",
         "Ti-rich, high-quality surface"],
        ["HTR", ">900 \u00b0C", "Extreme conditions; less studied"],
    ]

    header_row = [Paragraph(h, s["cell_header"]) for h in headers]
    data = [header_row]
    for row in rows_data:
        data.append([
            Paragraph(f"<b>{row[0]}</b>", s["cell_bold"]),
            Paragraph(row[1], s["cell"]),
            Paragraph(row[2], s["cell"]),
        ])

    col_widths = [2.8*inch, 1.8*inch, 4.15*inch]
    tbl = Table(data, colWidths=col_widths, repeatRows=1)
    tbl.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), ACCENT),
        ("TEXTCOLOR", (0, 0), (-1, 0), white),
        ("BOTTOMPADDING", (0, 0), (-1, 0), 7),
        ("TOPPADDING", (0, 0), (-1, 0), 7),
        ("ALIGN", (0, 0), (-1, 0), "CENTER"),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("TOPPADDING", (0, 1), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 1), (-1, -1), 4),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        *[("BACKGROUND", (0, i), (-1, i), ROW_EVEN if i % 2 == 0 else ROW_ODD)
          for i in range(1, len(data))],
        ("GRID", (0, 0), (-1, -1), 0.5, BORDER),
        ("LINEBELOW", (0, 0), (-1, 0), 1.5, ACCENT),
    ]))
    return tbl


def build_pdf():
    s = build_styles()

    doc = SimpleDocTemplate(
        str(OUTPUT),
        pagesize=landscape(letter),
        leftMargin=0.5*inch, rightMargin=0.5*inch,
        topMargin=0.5*inch, bottomMargin=0.5*inch,
    )

    story = []

    # Title block
    story.append(Paragraph("RHEED ML Literature Comparison", s["title"]))
    story.append(Paragraph(
        "AI-Driven Optimized MBE Growth &mdash; Yang Lab | Prepared for meeting, Mar 6 2026",
        s["subtitle"],
    ))
    story.append(HRFlowable(width="100%", thickness=1, color=ACCENT_LIGHT, spaceAfter=10))

    # Key insight callout
    story.append(Paragraph(
        "<b>Key insight:</b> No prior work has implemented real-time ML-triggered "
        "auto-capture for RHEED. Our Tier 2/3 auto-capture system (embedding changepoints "
        "+ classification change detection) would be novel.",
        s["body"],
    ))
    story.append(Spacer(1, 6))

    # Main table
    story.append(Paragraph("Literature Comparison", s["section"]))
    story.append(make_main_table(s))

    # Additional tools
    story.append(Paragraph("Additional Tools Discovered", s["section"]))
    story.append(make_tools_table(s))

    # Three-tier architecture
    story.append(Paragraph("Our Three-Tier Auto-Capture Architecture", s["section"]))
    story.append(make_tier_table(s))
    story.append(Spacer(1, 4))
    story.append(Paragraph(
        "<b>Tier 3</b> is the most semantically meaningful: captures exactly the events "
        "we care about (reconstruction transitions). <b>Tier 2</b> is the most robust to "
        "novel patterns &mdash; detects changes the classifier wasn't trained for.",
        s["body"],
    ))

    # STO reconstructions
    story.append(Paragraph("STO Surface Reconstructions (Physics Context)", s["section"]))
    story.append(make_recon_table(s))
    story.append(Spacer(1, 4))
    story.append(Paragraph(
        "These 5 classes are the output of our Classifier2 model and correspond to the "
        "surface science of SrTiO<sub>3</sub> (001) under varying temperature and oxygen "
        "partial pressure during MBE growth.",
        s["body"],
    ))

    doc.build(story)
    print(f"PDF written to {OUTPUT}")


if __name__ == "__main__":
    build_pdf()
