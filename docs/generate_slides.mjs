import pptxgen from "pptxgenjs";

const pres = new pptxgen();
pres.layout = "LAYOUT_16x9";
pres.author = "AJ — Yang Lab / Meng Group";
pres.title = "AIQM Growth Monitor — Option B Architecture";

// ── Color palette: Ocean Gradient + Lab instrument feel ──
const C = {
  navy:      "0C1B33",
  deepBlue:  "1A365D",
  midBlue:   "2B6CB0",
  teal:      "0D9488",
  seafoam:   "14B8A6",
  mint:      "5EEAD4",
  white:     "FFFFFF",
  offWhite:  "F7FAFC",
  lightGray: "E2E8F0",
  midGray:   "94A3B8",
  darkGray:  "334155",
  text:      "1E293B",
  textMuted: "64748B",
  amber:     "D97706",
  amberLight:"FEF3C7",
  green:     "059669",
  greenLight:"D1FAE5",
  red:       "DC2626",
  redLight:  "FEE2E2",
};

const FONT_TITLE = "Georgia";
const FONT_BODY = "Calibri";

// Helper: fresh shadow each call (pptxgenjs mutates objects)
const cardShadow = () => ({ type: "outer", blur: 6, offset: 2, angle: 135, color: "000000", opacity: 0.10 });

// ═══════════════════════════════════════════════════════════
// SLIDE 1 — Title
// ═══════════════════════════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.navy };

  // Decorative top bar
  s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 10, h: 0.06, fill: { color: C.teal } });

  // Subtitle label
  s.addText("AIQM GROWTH MONITOR", {
    x: 0.8, y: 1.2, w: 8.4, h: 0.4,
    fontFace: FONT_BODY, fontSize: 13, color: C.teal,
    charSpacing: 4, bold: true, margin: 0,
  });

  // Title
  s.addText("Option B: Local Deployment\non Bulbasaur", {
    x: 0.8, y: 1.7, w: 8.4, h: 1.6,
    fontFace: FONT_TITLE, fontSize: 40, color: C.white,
    bold: true, margin: 0,
  });

  // Tagline
  s.addText("Closed-loop autonomous MBE growth via RHEED classification,\nRL temperature policy, and PID-controlled OWON power supply", {
    x: 0.8, y: 3.5, w: 7, h: 0.9,
    fontFace: FONT_BODY, fontSize: 15, color: C.midGray,
    margin: 0,
  });

  // Footer info
  s.addText("Yang Lab  /  Meng Group Collaboration", {
    x: 0.8, y: 4.8, w: 4, h: 0.4,
    fontFace: FONT_BODY, fontSize: 11, color: C.textMuted, margin: 0,
  });
  s.addText("Meeting — March 6, 2026", {
    x: 6.5, y: 4.8, w: 3, h: 0.4,
    fontFace: FONT_BODY, fontSize: 11, color: C.textMuted, align: "right", margin: 0,
  });
}

// ═══════════════════════════════════════════════════════════
// SLIDE 2 — Two Operating Modes
// ═══════════════════════════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.offWhite };

  s.addText("Two Operating Modes for MBE Growth", {
    x: 0.6, y: 0.3, w: 8.8, h: 0.7,
    fontFace: FONT_TITLE, fontSize: 28, color: C.text, bold: true, margin: 0,
  });

  s.addText("The system supports switching between human-driven and autonomous operation.", {
    x: 0.6, y: 1.0, w: 8, h: 0.4,
    fontFace: FONT_BODY, fontSize: 13, color: C.textMuted, margin: 0,
  });

  // ── Normal Mode card ──
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.5, y: 1.6, w: 4.3, h: 3.4,
    fill: { color: C.white }, shadow: cardShadow(),
  });
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.5, y: 1.6, w: 4.3, h: 0.06, fill: { color: C.midBlue },
  });

  s.addText("NORMAL MODE", {
    x: 0.8, y: 1.8, w: 3.5, h: 0.35,
    fontFace: FONT_BODY, fontSize: 11, color: C.midBlue,
    charSpacing: 3, bold: true, margin: 0,
  });
  s.addText("Human-Driven Growth", {
    x: 0.8, y: 2.15, w: 3.5, h: 0.4,
    fontFace: FONT_TITLE, fontSize: 18, color: C.text, bold: true, margin: 0,
  });

  s.addText([
    { text: "Standard lab PSU (not programmable)", options: { bullet: true, breakLine: true } },
    { text: "RHEED on kSA screen, pyrometer on TemperaSure", options: { bullet: true, breakLine: true } },
    { text: "Human watches patterns, adjusts temperature", options: { bullet: true, breakLine: true } },
    { text: "Our system is not required", options: { bullet: true, breakLine: true, bold: true } },
  ], {
    x: 0.8, y: 2.65, w: 3.8, h: 2.0,
    fontFace: FONT_BODY, fontSize: 12, color: C.darkGray, margin: 0,
    paraSpaceAfter: 4,
  });

  // ── AI-Scientist Mode card ──
  s.addShape(pres.shapes.RECTANGLE, {
    x: 5.2, y: 1.6, w: 4.3, h: 3.4,
    fill: { color: C.white }, shadow: cardShadow(),
  });
  s.addShape(pres.shapes.RECTANGLE, {
    x: 5.2, y: 1.6, w: 4.3, h: 0.06, fill: { color: C.teal },
  });

  s.addText("AI-SCIENTIST MODE", {
    x: 5.5, y: 1.8, w: 3.5, h: 0.35,
    fontFace: FONT_BODY, fontSize: 11, color: C.teal,
    charSpacing: 3, bold: true, margin: 0,
  });
  s.addText("Closed-Loop Autonomous", {
    x: 5.5, y: 2.15, w: 3.5, h: 0.4,
    fontFace: FONT_TITLE, fontSize: 18, color: C.text, bold: true, margin: 0,
  });

  s.addText([
    { text: "OWON PSU swapped in (programmable, serial)", options: { bullet: true, breakLine: true } },
    { text: "RHEED frames streamed to Classifier2 ML", options: { bullet: true, breakLine: true } },
    { text: "RL policy decides temperature adjustments", options: { bullet: true, breakLine: true } },
    { text: "PID loop drives OWON, human can override", options: { bullet: true, breakLine: true, bold: true } },
  ], {
    x: 5.5, y: 2.65, w: 3.8, h: 2.0,
    fontFace: FONT_BODY, fontSize: 12, color: C.darkGray, margin: 0,
    paraSpaceAfter: 4,
  });

  // Badge on AI card
  s.addShape(pres.shapes.RECTANGLE, {
    x: 8.2, y: 1.8, w: 1.1, h: 0.3,
    fill: { color: C.greenLight },
  });
  s.addText("OUR SYSTEM", {
    x: 8.2, y: 1.8, w: 1.1, h: 0.3,
    fontFace: FONT_BODY, fontSize: 8, color: C.green,
    bold: true, align: "center", valign: "middle", margin: 0,
  });
}

// ═══════════════════════════════════════════════════════════
// SLIDE 3 — Why Option B Is the Only Viable Path
// ═══════════════════════════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.offWhite };

  s.addText("Why Run Everything on the Lab PC?", {
    x: 0.6, y: 0.3, w: 8.8, h: 0.7,
    fontFace: FONT_TITLE, fontSize: 28, color: C.text, bold: true, margin: 0,
  });

  s.addText("AI-Scientist mode requires direct access to three hardware interfaces on the same machine.", {
    x: 0.6, y: 1.0, w: 8, h: 0.4,
    fontFace: FONT_BODY, fontSize: 13, color: C.textMuted, margin: 0,
  });

  // Three hardware constraint cards
  const cards = [
    { label: "OWON PSU", detail: "USB Serial", reason: "PID voltage control\nto substrate heater", color: C.teal },
    { label: "Pyrometer", detail: "COM4 / Modbus RTU", reason: "Temperature feedback\nfor RL + PID loop", color: C.midBlue },
    { label: "RHEED Camera", detail: "kSA Screen Grab", reason: "Frame capture with\nexact false-color LUT", color: C.amber },
  ];

  cards.forEach((c, i) => {
    const x = 0.5 + i * 3.15;
    s.addShape(pres.shapes.RECTANGLE, {
      x, y: 1.6, w: 2.85, h: 2.0,
      fill: { color: C.white }, shadow: cardShadow(),
    });
    s.addShape(pres.shapes.RECTANGLE, {
      x, y: 1.6, w: 2.85, h: 0.06, fill: { color: c.color },
    });
    s.addText(c.label, {
      x: x + 0.2, y: 1.8, w: 2.45, h: 0.35,
      fontFace: FONT_TITLE, fontSize: 16, color: C.text, bold: true, margin: 0,
    });
    s.addText(c.detail, {
      x: x + 0.2, y: 2.1, w: 2.45, h: 0.3,
      fontFace: FONT_BODY, fontSize: 11, color: c.color, bold: true, margin: 0,
    });
    s.addText(c.reason, {
      x: x + 0.2, y: 2.5, w: 2.45, h: 0.8,
      fontFace: FONT_BODY, fontSize: 11, color: C.textMuted, margin: 0,
    });
  });

  // Conclusion box
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.5, y: 3.9, w: 9, h: 0.9,
    fill: { color: C.greenLight },
  });
  s.addText([
    { text: "Conclusion: ", options: { bold: true, color: C.green } },
    { text: "All three interfaces are physically connected to Bulbasaur. No remote architecture can control the OWON PSU or read COM4 directly. The GUI ", options: { color: C.text } },
    { text: "must run on the lab PC.", options: { bold: true, color: C.text } },
  ], {
    x: 0.8, y: 3.95, w: 8.4, h: 0.8,
    fontFace: FONT_BODY, fontSize: 13, margin: 0, valign: "middle",
  });

  // Rejected alternatives footnote
  s.addText("Rejected: TeamViewer screen grab (lossy, high latency), Frame Server (still needs Python + can't control PSU), Direct GigE (conflicts with kSA)", {
    x: 0.6, y: 5.0, w: 8.8, h: 0.4,
    fontFace: FONT_BODY, fontSize: 10, color: C.midGray, italic: true, margin: 0,
  });
}

// ═══════════════════════════════════════════════════════════
// SLIDE 4 — Hardware Topology
// ═══════════════════════════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.offWhite };

  s.addText("Hardware Topology — Bulbasaur", {
    x: 0.6, y: 0.3, w: 8.8, h: 0.7,
    fontFace: FONT_TITLE, fontSize: 28, color: C.text, bold: true, margin: 0,
  });

  // Lab PC box
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.4, y: 1.2, w: 6.0, h: 3.8,
    fill: { color: C.white }, shadow: cardShadow(),
  });
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.4, y: 1.2, w: 6.0, h: 0.06, fill: { color: C.navy },
  });

  s.addText("BULBASAUR  —  MBE Lab PC (Windows)", {
    x: 0.6, y: 1.35, w: 5.6, h: 0.35,
    fontFace: FONT_BODY, fontSize: 12, color: C.navy, bold: true, charSpacing: 1, margin: 0,
  });

  const hwItems = [
    { label: "kSA 400", value: "RHEED display — window: \"AVT Manta_G Live Video\"", color: C.amber },
    { label: "TemperaSure 5.7", value: "Pyrometer GUI — BASF TemperaSure 5.7.0.4", color: C.midBlue },
    { label: "COM4 (PL2303GS)", value: "RS-422 to BASF Exactus pyrometer via IFD-5", color: C.midBlue },
    { label: "GigE", value: "Allied Vision Manta G-033B (656x492, mono, 12-bit)", color: C.teal },
    { label: "OWON PSU", value: "USB serial — swapped in for AI-Scientist mode", color: C.green },
    { label: "Python + GUI", value: "To be installed — Growth Monitor + Classifier2 + PID", color: C.red },
  ];

  hwItems.forEach((item, i) => {
    const y = 1.85 + i * 0.48;
    s.addShape(pres.shapes.RECTANGLE, {
      x: 0.7, y, w: 0.12, h: 0.3, fill: { color: item.color },
    });
    s.addText(item.label, {
      x: 1.0, y, w: 1.7, h: 0.3,
      fontFace: FONT_BODY, fontSize: 10, color: C.text, bold: true, margin: 0, valign: "middle",
    });
    s.addText(item.value, {
      x: 2.7, y, w: 3.5, h: 0.3,
      fontFace: FONT_BODY, fontSize: 10, color: C.textMuted, margin: 0, valign: "middle",
    });
  });

  // Dev Mac box
  s.addShape(pres.shapes.RECTANGLE, {
    x: 6.8, y: 1.2, w: 2.8, h: 2.2,
    fill: { color: C.white }, shadow: cardShadow(),
  });
  s.addShape(pres.shapes.RECTANGLE, {
    x: 6.8, y: 1.2, w: 2.8, h: 0.06, fill: { color: C.midBlue },
  });

  s.addText("DEV MAC", {
    x: 7.0, y: 1.35, w: 2.4, h: 0.35,
    fontFace: FONT_BODY, fontSize: 12, color: C.midBlue, bold: true, charSpacing: 1, margin: 0,
  });

  s.addText([
    { text: "Train models (MPS GPU)", options: { bullet: true, breakLine: true } },
    { text: "Develop GUI + drivers", options: { bullet: true, breakLine: true } },
    { text: "Copy artifacts to Bulbasaur", options: { bullet: true } },
  ], {
    x: 7.0, y: 1.8, w: 2.4, h: 1.2,
    fontFace: FONT_BODY, fontSize: 10, color: C.darkGray, margin: 0,
    paraSpaceAfter: 3,
  });

  // Arrow from Mac to Bulbasaur
  s.addShape(pres.shapes.RECTANGLE, {
    x: 6.8, y: 3.7, w: 2.8, h: 0.9,
    fill: { color: C.amberLight },
  });
  s.addText("Deploy: model + code\nvia USB drive or network", {
    x: 7.0, y: 3.75, w: 2.4, h: 0.8,
    fontFace: FONT_BODY, fontSize: 10, color: C.amber, bold: true, margin: 0, align: "center", valign: "middle",
  });

  // Network note
  s.addText("IP: 205.208.25.122 (campus) — Remote: TeamViewer only (monitoring, not control)", {
    x: 0.6, y: 5.1, w: 8.8, h: 0.3,
    fontFace: FONT_BODY, fontSize: 10, color: C.midGray, italic: true, margin: 0,
  });
}

// ═══════════════════════════════════════════════════════════
// SLIDE 5 — Current System Status
// ═══════════════════════════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.offWhite };

  s.addText("What We've Built So Far", {
    x: 0.6, y: 0.3, w: 8.8, h: 0.7,
    fontFace: FONT_TITLE, fontSize: 28, color: C.text, bold: true, margin: 0,
  });

  // Left column — Software
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.5, y: 1.15, w: 4.3, h: 4.0,
    fill: { color: C.white }, shadow: cardShadow(),
  });
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.5, y: 1.15, w: 4.3, h: 0.06, fill: { color: C.teal },
  });

  s.addText("SOFTWARE (DONE)", {
    x: 0.7, y: 1.35, w: 3.9, h: 0.3,
    fontFace: FONT_BODY, fontSize: 11, color: C.teal, bold: true, charSpacing: 2, margin: 0,
  });

  const softwareItems = [
    "Driver ABCs (camera, pyrometer) + 3 implementations each",
    "GUI tabs: RHEED, Pyrometer, PSU, Thermocouple, Dashboard",
    "Growth Monitor dashboard with human classification sliders",
    "Tier 1 auto-capture (intensity change detection)",
    "Tier 3 auto-capture (classification JS-divergence)",
    "ClassifierBridge — loads Classifier2, expose classify(frame)",
    "Session logging (sensor CSV, commit CSV, frame PNGs)",
    "GrowthApp orchestrator (ARM/DISARM/START/STOP)",
  ];

  s.addText(
    softwareItems.map((t, i) => ({
      text: t,
      options: { bullet: true, breakLine: i < softwareItems.length - 1 },
    })),
    {
      x: 0.7, y: 1.75, w: 3.9, h: 3.2,
      fontFace: FONT_BODY, fontSize: 10, color: C.darkGray, margin: 0,
      paraSpaceAfter: 3,
    }
  );

  // Right column — ML
  s.addShape(pres.shapes.RECTANGLE, {
    x: 5.2, y: 1.15, w: 4.3, h: 2.3,
    fill: { color: C.white }, shadow: cardShadow(),
  });
  s.addShape(pres.shapes.RECTANGLE, {
    x: 5.2, y: 1.15, w: 4.3, h: 0.06, fill: { color: C.midBlue },
  });

  s.addText("CLASSIFIER2 (TRAINED)", {
    x: 5.4, y: 1.35, w: 3.9, h: 0.3,
    fontFace: FONT_BODY, fontSize: 11, color: C.midBlue, bold: true, charSpacing: 2, margin: 0,
  });

  s.addText([
    { text: "100% ideal-split accuracy (30/30)", options: { bullet: true, breakLine: true, bold: true } },
    { text: "88.3% pairwise holdout (159 ideal images)", options: { bullet: true, breakLine: true } },
    { text: "5 classes: (1x1), Tw(2x1), c(6x2), rt13, HTR", options: { bullet: true, breakLine: true } },
    { text: "CPU inference: ~80 ms/frame (ResNet18, 11.3M params)", options: { bullet: true } },
  ], {
    x: 5.4, y: 1.75, w: 3.9, h: 1.5,
    fontFace: FONT_BODY, fontSize: 10, color: C.darkGray, margin: 0,
    paraSpaceAfter: 3,
  });

  // Right column — Blockers
  s.addShape(pres.shapes.RECTANGLE, {
    x: 5.2, y: 3.65, w: 4.3, h: 1.5,
    fill: { color: C.white }, shadow: cardShadow(),
  });
  s.addShape(pres.shapes.RECTANGLE, {
    x: 5.2, y: 3.65, w: 4.3, h: 0.06, fill: { color: C.red },
  });

  s.addText("BLOCKERS", {
    x: 5.4, y: 3.85, w: 3.9, h: 0.3,
    fontFace: FONT_BODY, fontSize: 11, color: C.red, bold: true, charSpacing: 2, margin: 0,
  });

  s.addText([
    { text: "Python not installed on Bulbasaur", options: { bullet: true, breakLine: true } },
    { text: "RHEED gun down — no live camera testing", options: { bullet: true, breakLine: true } },
    { text: "Pyrometer was off during last visit", options: { bullet: true } },
  ], {
    x: 5.4, y: 4.2, w: 3.9, h: 0.9,
    fontFace: FONT_BODY, fontSize: 10, color: C.darkGray, margin: 0,
    paraSpaceAfter: 3,
  });
}

// ═══════════════════════════════════════════════════════════
// SLIDE 6 — Full Closed-Loop Data Flow
// ═══════════════════════════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.navy };
  s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 10, h: 0.06, fill: { color: C.teal } });

  s.addText("AI-Scientist Mode: Full Closed-Loop", {
    x: 0.6, y: 0.2, w: 8.8, h: 0.6,
    fontFace: FONT_TITLE, fontSize: 26, color: C.white, bold: true, margin: 0,
  });

  s.addText("~120 ms per cycle  |  1 FPS target  |  All local on Bulbasaur", {
    x: 0.6, y: 0.75, w: 8, h: 0.3,
    fontFace: FONT_BODY, fontSize: 11, color: C.midGray, margin: 0,
  });

  // Flow boxes — horizontal pipeline
  const flowSteps = [
    { label: "RHEED\nScreen Grab", time: "~10 ms", color: C.amber, x: 0.3 },
    { label: "Classifier2\nInference", time: "~80 ms", color: C.midBlue, x: 2.2 },
    { label: "RL\nPolicy", time: "~1 ms", color: C.teal, x: 4.1 },
    { label: "PID\nController", time: "~1 ms", color: C.seafoam, x: 6.0 },
    { label: "OWON\nPSU Write", time: "~10 ms", color: C.green, x: 7.9 },
  ];

  flowSteps.forEach((step, i) => {
    s.addShape(pres.shapes.RECTANGLE, {
      x: step.x, y: 1.35, w: 1.6, h: 1.1,
      fill: { color: step.color, transparency: 85 },
      line: { color: step.color, width: 1.5 },
    });
    s.addText(step.label, {
      x: step.x, y: 1.4, w: 1.6, h: 0.7,
      fontFace: FONT_BODY, fontSize: 11, color: C.white, bold: true,
      align: "center", valign: "middle", margin: 0,
    });
    s.addText(step.time, {
      x: step.x, y: 2.1, w: 1.6, h: 0.3,
      fontFace: FONT_BODY, fontSize: 9, color: C.midGray,
      align: "center", margin: 0,
    });

    // Arrow between boxes
    if (i < flowSteps.length - 1) {
      const arrowX = step.x + 1.6;
      s.addShape(pres.shapes.LINE, {
        x: arrowX, y: 1.9, w: 0.6, h: 0,
        line: { color: C.teal, width: 2 },
      });
    }
  });

  // Feedback loops — pyrometer + heater
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.3, y: 2.8, w: 9.4, h: 0.06, fill: { color: C.deepBlue },
  });

  // Bottom row: data sources + outputs
  s.addText("FEEDBACK LOOPS", {
    x: 0.3, y: 3.0, w: 3, h: 0.3,
    fontFace: FONT_BODY, fontSize: 10, color: C.teal, bold: true, charSpacing: 2, margin: 0,
  });

  const feedbackItems = [
    { label: "Pyrometer (COM4)", desc: "Modbus RTU, ~20 ms read", note: "Temperature feedback to RL + PID", color: C.midBlue },
    { label: "OWON PSU (USB)", desc: "Serial write, ~10 ms", note: "Voltage to substrate heater", color: C.green },
    { label: "Auto-Capture", desc: "Tier 1/2/3 detectors", note: "Log frames + CSVs to session folder", color: C.amber },
    { label: "Growth Monitor GUI", desc: "Human override at any time", note: "AI confidence bar, classification, BAD indicator", color: C.seafoam },
  ];

  feedbackItems.forEach((item, i) => {
    const x = 0.3 + i * 2.4;
    s.addShape(pres.shapes.RECTANGLE, {
      x, y: 3.4, w: 2.2, h: 1.8,
      fill: { color: item.color, transparency: 90 },
      line: { color: item.color, width: 1 },
    });
    s.addText(item.label, {
      x: x + 0.1, y: 3.5, w: 2.0, h: 0.3,
      fontFace: FONT_BODY, fontSize: 10, color: C.white, bold: true, margin: 0,
    });
    s.addText(item.desc, {
      x: x + 0.1, y: 3.8, w: 2.0, h: 0.25,
      fontFace: FONT_BODY, fontSize: 9, color: C.midGray, margin: 0,
    });
    s.addText(item.note, {
      x: x + 0.1, y: 4.15, w: 2.0, h: 0.7,
      fontFace: FONT_BODY, fontSize: 9, color: C.lightGray, margin: 0,
    });
  });
}

// ═══════════════════════════════════════════════════════════
// SLIDE 7 — ML Pipeline Detail
// ═══════════════════════════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.offWhite };

  s.addText("ML Classification Pipeline", {
    x: 0.6, y: 0.3, w: 8.8, h: 0.7,
    fontFace: FONT_TITLE, fontSize: 28, color: C.text, bold: true, margin: 0,
  });

  // Pipeline flow
  const steps = [
    { label: "kSA Frame\n(656x492 RGB)", sub: "False-color LUT", x: 0.4, color: C.amber },
    { label: "Resize\n224x224", sub: "Normalize", x: 2.4, color: C.textMuted },
    { label: "SimCLR\nResNet18", sub: "Encoder (512-dim)", x: 4.4, color: C.midBlue },
    { label: "Reward\nHead", sub: "512 > 256 > 5", x: 6.4, color: C.teal },
    { label: "Win-Rate\nClassify", sub: "vs 159 ideal refs", x: 8.3, color: C.green },
  ];

  steps.forEach((step, i) => {
    s.addShape(pres.shapes.RECTANGLE, {
      x: step.x, y: 1.2, w: 1.7, h: 1.0,
      fill: { color: C.white }, shadow: cardShadow(),
    });
    s.addShape(pres.shapes.RECTANGLE, {
      x: step.x, y: 1.2, w: 1.7, h: 0.05, fill: { color: step.color },
    });
    s.addText(step.label, {
      x: step.x + 0.05, y: 1.3, w: 1.6, h: 0.6,
      fontFace: FONT_BODY, fontSize: 10, color: C.text, bold: true,
      align: "center", valign: "middle", margin: 0,
    });
    s.addText(step.sub, {
      x: step.x + 0.05, y: 1.9, w: 1.6, h: 0.25,
      fontFace: FONT_BODY, fontSize: 8, color: C.textMuted,
      align: "center", margin: 0,
    });
    if (i < steps.length - 1) {
      s.addShape(pres.shapes.LINE, {
        x: step.x + 1.7, y: 1.7, w: 0.7, h: 0,
        line: { color: C.lightGray, width: 1.5 },
      });
    }
  });

  // Output section
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.4, y: 2.6, w: 9.2, h: 0.05, fill: { color: C.lightGray },
  });

  s.addText("OUTPUT", {
    x: 0.6, y: 2.8, w: 2, h: 0.3,
    fontFace: FONT_BODY, fontSize: 10, color: C.teal, bold: true, charSpacing: 2, margin: 0,
  });

  // Output cards
  const outputs = [
    { label: "predicted_class", value: "Argmax of 5 win-rates\ne.g. \"c(6x2)\"" },
    { label: "classification_scores", value: "5 probabilities\n(1x1, Tw, c6x2, rt13, HTR)" },
    { label: "is_bad + bad_confidence", value: "Compared against 18\nbad reference images" },
    { label: "quality", value: "0-1 scalar,\noverall surface quality" },
  ];

  outputs.forEach((o, i) => {
    const x = 0.4 + i * 2.35;
    s.addShape(pres.shapes.RECTANGLE, {
      x, y: 3.2, w: 2.15, h: 1.1,
      fill: { color: C.white }, shadow: cardShadow(),
    });
    s.addText(o.label, {
      x: x + 0.1, y: 3.25, w: 1.95, h: 0.35,
      fontFace: FONT_BODY, fontSize: 10, color: C.midBlue, bold: true, margin: 0,
    });
    s.addText(o.value, {
      x: x + 0.1, y: 3.6, w: 1.95, h: 0.6,
      fontFace: FONT_BODY, fontSize: 9, color: C.textMuted, margin: 0,
    });
  });

  // Training data note
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.4, y: 4.6, w: 9.2, h: 0.7,
    fill: { color: C.amberLight },
  });
  s.addText([
    { text: "Training data: ", options: { bold: true, color: C.amber } },
    { text: "Bradley-Terry reward model trained on 678 pairwise human comparisons + 159 ideal images across 5 reconstruction types. Model was trained on kSA false-color screenshots — screen grab on Bulbasaur produces ", options: { color: C.text } },
    { text: "exact match to training distribution.", options: { bold: true, color: C.text } },
  ], {
    x: 0.6, y: 4.65, w: 8.8, h: 0.6,
    fontFace: FONT_BODY, fontSize: 10, margin: 0, valign: "middle",
  });
}

// ═══════════════════════════════════════════════════════════
// SLIDE 8 — Performance Budget
// ═══════════════════════════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.offWhite };

  s.addText("Performance Budget — 1 FPS Closed Loop", {
    x: 0.6, y: 0.3, w: 8.8, h: 0.7,
    fontFace: FONT_TITLE, fontSize: 28, color: C.text, bold: true, margin: 0,
  });

  s.addText("Total loop: ~120 ms per cycle on CPU. Target: 1 frame per second (plenty of headroom).", {
    x: 0.6, y: 1.0, w: 8, h: 0.4,
    fontFace: FONT_BODY, fontSize: 13, color: C.textMuted, margin: 0,
  });

  // Horizontal stacked bar
  const barY = 1.7;
  const barH = 0.8;
  const totalW = 8.0;
  const barX = 1.0;
  const segments = [
    { label: "Screen\nGrab", ms: 10, color: C.amber },
    { label: "Modbus\nRead", ms: 20, color: C.midBlue },
    { label: "Classifier2\nInference", ms: 80, color: C.teal },
    { label: "RL\nPolicy", ms: 1, color: C.seafoam },
    { label: "PID +\nSerial", ms: 11, color: C.green },
  ];
  const totalMs = segments.reduce((a, s) => a + s.ms, 0);

  let cx = barX;
  segments.forEach((seg) => {
    const w = Math.max((seg.ms / totalMs) * totalW, 0.35);
    s.addShape(pres.shapes.RECTANGLE, {
      x: cx, y: barY, w, h: barH,
      fill: { color: seg.color },
    });
    s.addText(seg.label, {
      x: cx, y: barY + 0.05, w, h: 0.45,
      fontFace: FONT_BODY, fontSize: 8, color: C.white, bold: true,
      align: "center", valign: "middle", margin: 0,
    });
    s.addText(`${seg.ms} ms`, {
      x: cx, y: barY + 0.48, w, h: 0.25,
      fontFace: FONT_BODY, fontSize: 9, color: C.white,
      align: "center", margin: 0,
    });
    cx += w;
  });

  // Remaining headroom
  const remainW = totalW - cx + barX;
  if (remainW > 0.2) {
    s.addShape(pres.shapes.RECTANGLE, {
      x: cx, y: barY, w: remainW, h: barH,
      fill: { color: C.lightGray },
    });
    s.addText("headroom\n878 ms", {
      x: cx, y: barY, w: remainW, h: barH,
      fontFace: FONT_BODY, fontSize: 9, color: C.textMuted,
      align: "center", valign: "middle", margin: 0,
    });
  }

  // Install footprint
  s.addText("Install Footprint on Bulbasaur", {
    x: 0.6, y: 2.9, w: 8, h: 0.4,
    fontFace: FONT_TITLE, fontSize: 18, color: C.text, bold: true, margin: 0,
  });

  const installItems = [
    { label: "Python 3.12", size: "~30 MB", note: "User-level install (winget / embeddable zip)" },
    { label: "PyTorch CPU-only", size: "~150 MB", note: "No CUDA/MPS needed" },
    { label: "PyQt6 + deps", size: "~100 MB", note: "GUI framework" },
    { label: "Model artifacts", size: "~50 MB", note: "best_model.pth + ideal reference images" },
  ];

  installItems.forEach((item, i) => {
    const y = 3.4 + i * 0.45;
    s.addShape(pres.shapes.RECTANGLE, {
      x: 0.6, y, w: 0.08, h: 0.3, fill: { color: C.teal },
    });
    s.addText(item.label, {
      x: 0.85, y, w: 1.8, h: 0.3,
      fontFace: FONT_BODY, fontSize: 11, color: C.text, bold: true, margin: 0, valign: "middle",
    });
    s.addText(item.size, {
      x: 2.7, y, w: 1.0, h: 0.3,
      fontFace: FONT_BODY, fontSize: 11, color: C.teal, bold: true, margin: 0, valign: "middle",
    });
    s.addText(item.note, {
      x: 3.7, y, w: 4, h: 0.3,
      fontFace: FONT_BODY, fontSize: 11, color: C.textMuted, margin: 0, valign: "middle",
    });
  });

  s.addText("Total: ~350 MB", {
    x: 0.6, y: 5.1, w: 3, h: 0.3,
    fontFace: FONT_BODY, fontSize: 13, color: C.teal, bold: true, margin: 0,
  });
}

// ═══════════════════════════════════════════════════════════
// SLIDE 9 — Deployment Plan & Next Steps
// ═══════════════════════════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.offWhite };

  s.addText("Deployment Plan — Next Steps", {
    x: 0.6, y: 0.3, w: 8.8, h: 0.7,
    fontFace: FONT_TITLE, fontSize: 28, color: C.text, bold: true, margin: 0,
  });

  const steps = [
    { num: "01", text: "Get permission to install Python on Bulbasaur", note: "User-level install via winget, MS Store, or embeddable zip (no admin needed)", priority: "blocker" },
    { num: "02", text: "Clone AIQM repo + copy model artifacts", note: "Via git or USB drive. Include best_model.pth and 159 ideal reference images", priority: "high" },
    { num: "03", text: "pip install dependencies", note: "PyQt6, torch (CPU-only), pymodbus, mss, pillow, numpy — all pure pip", priority: "high" },
    { num: "04", text: "Connect OWON PSU via USB", note: "Verify serial port detection and PID loop on Bulbasaur", priority: "high" },
    { num: "05", text: "Test pyrometer serial (COM4)", note: "Modbus RTU read when pyrometer is powered on — driver already written", priority: "medium" },
    { num: "06", text: "Test kSA screen grab", note: "Verify ScreenGrabCamera captures RHEED window natively at full resolution", priority: "medium" },
    { num: "07", text: "Full integration test", note: "Live RHEED + pyrometer + Classifier2 + OWON PID + auto-capture logging", priority: "final" },
  ];

  steps.forEach((step, i) => {
    const y = 1.1 + i * 0.6;
    const numColor = step.priority === "blocker" ? C.red : step.priority === "final" ? C.green : C.teal;

    s.addShape(pres.shapes.RECTANGLE, {
      x: 0.5, y, w: 0.55, h: 0.45,
      fill: { color: numColor, transparency: 90 },
      line: { color: numColor, width: 1 },
    });
    s.addText(step.num, {
      x: 0.5, y, w: 0.55, h: 0.45,
      fontFace: FONT_BODY, fontSize: 12, color: numColor, bold: true,
      align: "center", valign: "middle", margin: 0,
    });

    s.addText(step.text, {
      x: 1.2, y, w: 4.0, h: 0.45,
      fontFace: FONT_BODY, fontSize: 12, color: C.text, bold: true,
      margin: 0, valign: "middle",
    });

    s.addText(step.note, {
      x: 5.3, y, w: 4.2, h: 0.45,
      fontFace: FONT_BODY, fontSize: 10, color: C.textMuted,
      margin: 0, valign: "middle",
    });
  });

  // Key blocker callout
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.5, y: 5.0, w: 9, h: 0.5,
    fill: { color: C.redLight },
  });
  s.addText([
    { text: "Key blocker: ", options: { bold: true, color: C.red } },
    { text: "Step 01 — everything else is ready to go once Python is on Bulbasaur.", options: { color: C.text } },
  ], {
    x: 0.7, y: 5.0, w: 8.6, h: 0.5,
    fontFace: FONT_BODY, fontSize: 12, margin: 0, valign: "middle",
  });
}

// ═══════════════════════════════════════════════════════════
// SLIDE 10 — Closing
// ═══════════════════════════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.navy };
  s.addShape(pres.shapes.RECTANGLE, { x: 0, y: 0, w: 10, h: 0.06, fill: { color: C.teal } });

  s.addText("Summary", {
    x: 0.8, y: 0.8, w: 8.4, h: 0.6,
    fontFace: FONT_TITLE, fontSize: 32, color: C.white, bold: true, margin: 0,
  });

  s.addText([
    { text: "1.  ", options: { bold: true, color: C.teal } },
    { text: "Two modes: Normal (human) and AI-Scientist (closed-loop autonomous)", options: { color: C.lightGray, breakLine: true } },
    { text: "", options: { breakLine: true, fontSize: 6 } },
    { text: "2.  ", options: { bold: true, color: C.teal } },
    { text: "Option B is the only viable architecture — all hardware is on Bulbasaur", options: { color: C.lightGray, breakLine: true } },
    { text: "", options: { breakLine: true, fontSize: 6 } },
    { text: "3.  ", options: { bold: true, color: C.teal } },
    { text: "Software is built and Classifier2 is trained (100% ideal-split accuracy)", options: { color: C.lightGray, breakLine: true } },
    { text: "", options: { breakLine: true, fontSize: 6 } },
    { text: "4.  ", options: { bold: true, color: C.teal } },
    { text: "Full loop budget: ~120 ms/cycle, well within 1 FPS target", options: { color: C.lightGray, breakLine: true } },
    { text: "", options: { breakLine: true, fontSize: 6 } },
    { text: "5.  ", options: { bold: true, color: C.teal } },
    { text: "One blocker: permission to install Python on Bulbasaur (~350 MB)", options: { color: C.lightGray } },
  ], {
    x: 0.8, y: 1.7, w: 8.4, h: 3.2,
    fontFace: FONT_BODY, fontSize: 16, margin: 0,
    paraSpaceAfter: 2,
  });

  s.addText("Yang Lab  /  Meng Group  |  March 2026", {
    x: 0.8, y: 4.9, w: 4, h: 0.4,
    fontFace: FONT_BODY, fontSize: 11, color: C.midGray, margin: 0,
  });
}

// ── Write file ──
const outPath = "/Users/aj/AIQM-Software-Hardware-Integration/docs/growth_monitor_option_b.pptx";
await pres.writeFile({ fileName: outPath });
console.log(`PPTX written to ${outPath}`);
