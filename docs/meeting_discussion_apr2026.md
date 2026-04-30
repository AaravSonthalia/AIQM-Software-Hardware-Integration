# AIQM Group Discussion — Talking Points (April 2026)

Prepared following Prof. Xu office hours (Apr 2, 2026), Justin Meng sync (Apr 3, 2026), and review of the NSF FMSG proposal, kSA 400 / BASF Exactus manuals, and SDL literature (Tom et al., Chem. Rev. 2024).

**Guiding principle (Xu + Justin, independently):** Science is the main focus. AI is the tool, not the goal. Every modeling decision should be grounded in physical understanding.

---

## 1. Context: Where We Stand

- **Growth Monitor v2** is functional on Bulbasaur — RHEED screengrab, pyrometer screengrab, timestamped log entries with RHEED frame capture, growth log export. Reconstruction estimate sliders now included for grower input.
- **Classifier2** achieves 100% ideal-split accuracy (4 classes), trained on BMP screengrabs. ClassifierBridge ready for v3 integration.
- **Data collected so far:** 170/1,124 trajectory images labeled. ~0 complete structured growth sessions with all parameters recorded.
- **Xu's core message:** Data collection is the primary mission. Temper expectations for the 2-year funding window. Her lab had Argonne's decades of data. We need to start building ours now.

---

## 2. The Parameter Selection Question

**Xu's tradeoff:** Too few parameters = false/non-optimal patterns. Too many = model gets confused, needs more data.

**Core parameters (agreed by both Xu and Justin):** Time, Temperature, Ramp rate (dT/dt), RHEED images. These are the minimum viable feature set for any model.

**Proposed resolution: Separate capture from modeling.**

| Decision | Approach |
|---|---|
| **What to capture/log** | Broadly — every growth session should record as much as possible. Storage is cheap; missing data is irreplaceable. |
| **What the AI models** | Narrowly at first — start with temperature trajectory (T vs t), expand based on feature importance after ~50 growths. |

### The 7 Proposal Parameters and How to Get Them

From the NSF FMSG proposal (Figure 3), the FeSe/STO growth process has 7 key parameters:

| Parameter | Current Status | Path to Automated Capture |
|---|---|---|
| T_pre, T_post (annealing temps) | YES — pyrometer | Already in Growth Monitor |
| D_pre, D_post (durations) | YES — derived | Computed from session timestamps |
| T_S (substrate temp during growth) | YES — pyrometer | Already in Growth Monitor |
| F_Fe, F_Se (fluxes) | NO — locked in MISTRAL | Manual entry fields in GUI (near-term); BFM via MISTRAL/pywinauto (future) |
| dT/dt (ramp rate) | YES — derived | Computed from pyrometer time series |

**Recommended additions to Growth Monitor:**
1. **Time vs Temperature live plot** — Xu specifically requested. Gives ramp rate at a glance.
2. **Manual-entry fields** for F_Fe, F_Se, chamber pressure — even approximate values build the database.
3. **Growth phase markers** — grower tags session segments as "pre-anneal", "growth", "post-anneal" to structure the data.

---

## 3. Sensor Suggestions

Beyond what we currently have, additional sensors would strengthen the dataset.

### Near-Term (investigate at next lab visit)

| Sensor / Source | What It Provides | Integration Path | Effort |
|---|---|---|---|
| **Eurotherm controller** (Modbus TCP) | Heater setpoint, actual temp, output power (%) | Python `pymodbus` on Bulbasaur; existing driver at `research-lab/src/control/temp_pid.py` | Medium — need to find IP on 10.0.42.x subnet |
| **kSA Auto Export** | All RHEED acquisition data as ASCII `.txt` | Enable in kSA Advanced Acquisition Options; Growth Monitor watches output directory | Low — software config only |
| **TemperaSure Macro Mode** | Pyrometer CSV export at configurable interval to `C:\BASF\Macro Data` | File-watch from Growth Monitor; keeps TemperaSure running (no Modbus conflict) | Low |
| **kSA Analog Input Board** (Data Translation DT9802) | Up to 16 channels of external analog signals (pressure gauges, thermocouples, etc.) | Hardware purchase (~$500-1000); kSA auto-records alongside RHEED data in `.kdt` | Medium — hardware + config |

### Future / To Discuss with Expert Growers

| Sensor | What It Would Tell Us | Why It Matters |
|---|---|---|
| **Chamber pressure gauge with digital output** | Base pressure, growth pressure, pressure during annealing | Xu highlighted pressure as a potentially important hidden variable |
| **Multi-zone thermocouple array** | Thermal gradient / heating uniformity across substrate | "Same conditions, different results" — spatial temperature variation could explain irreproducibility |
| **Beam flux monitor (BFM) digital readout** | Real-time flux measurements during growth | Direct measurement of F_Fe, F_Se — eliminates need for manual entry |

**Key question for the group:** Which of these sensors would give us the most information per dollar/effort? The Eurotherm read and kSA Auto Export are essentially free — just need lab time to configure.

---

## 4. Physics-Based Reward Function

Xu raised the concern that the AI advisor needs to stay true to the physics. The SDL literature (GPax, Sanchez et al. 2024) supports this with a concrete approach:

### Physics-Informed Gaussian Process (GPax)

Instead of treating the optimization as a pure black-box, encode known MBE physics into the surrogate model:

- **GP mean function:** Encode the known STO phase diagram — which reconstructions appear at which temperatures (e.g., c(6x2) at ~600-700C, RT13 at ~900-1000C, HTR above ~1050C). This gives the model a physics backbone.
- **Thermodynamic constraints:** Reconstruction transitions follow Arrhenius-like kinetics. The GP kernel can encode this temperature dependence, so the model doesn't waste experiments exploring physically impossible regions.
- **Safety shield:** Hard constraints on temperature bounds (no excursions above 1200C, no ramp rates > 50C/min) encoded as constraints in the acquisition function, not soft penalties in the reward.

### Structured Reward for the RL Agent (v4)

Based on the proposal's framing and Polybot precedent:

```
R(s,a) = alpha * delta_Q           # improvement in RHEED quality metric
       + beta * P_target           # probability of target reconstruction (from Classifier2)
       - lambda_safety * violation  # hard constraint: T_max, ramp rate limits
       + gamma * information_gain   # exploration bonus (encourage diverse parameter sampling)
```

Where `delta_Q` could be the change in classifier confidence toward the target reconstruction, and `violation` is binary (0 or 1) — not a soft penalty.

**This matters for the meeting because:** It directly addresses Xu's concern about understanding the physics. A physics-informed model doesn't just find patterns — it finds patterns that are physically meaningful. SHAP analysis post-campaign can then tell us which parameters actually drove the outcome.

---

## 5. VLM/LLM as RHEED Advisor (from Justin sync)

Justin proposed exploring vision-language models (VLMs) or LLMs as an alternative or complement to Classifier2 for RHEED interpretation. The idea: instead of (or in addition to) a CNN classifier, use a VLM that can look at a RHEED image and provide natural-language analysis — identifying reconstruction type, noting features like satellite peaks or Kikuchi lines, and suggesting what to do next.

### Why This Is Worth Exploring

- **Richer output than classification:** Classifier2 outputs 5 probabilities. A VLM could say "this looks like a transitioning c(6x2) with faint satellite peaks emerging, suggesting the surface is approaching RT13 — consider holding temperature for 5 more minutes." That's closer to what an expert grower thinks.
- **Reward shaping:** VLM analysis could inform the reward function beyond simple classification confidence. Features like Kikuchi line sharpness or satellite peak intensity encode surface quality information that the CNN doesn't explicitly capture.
- **Lower labeling burden:** With in-context learning (few-shot examples), a VLM may generalize from a small set of ideal reconstruction examples without needing thousands of labeled pairs.

### Concrete Next Steps (Justin's plan)

1. **Curate in-context sample sets** — assemble 3-5 ideal examples of each reconstruction type (1x1, Twinned, c(6x2), RT13, HTR) with expert annotations describing what makes each image characteristic.
2. **Tune prompts with domain experts** — have expert growers describe what they look for: satellite peaks, Kikuchi lines, spot sharpness, streak intensity, background texture. Encode this into the VLM prompt.
3. **Benchmark VLM vs Classifier2** — run both on the same test set. Where does the VLM agree/disagree? What does it catch that the CNN misses (and vice versa)?
4. **Read more literature** — survey VLM applications in materials characterization (e.g., image analysis in microscopy, spectroscopy interpretation).

### How This Fits the Architecture

- **v3 Supervised AI Mode:** VLM provides natural-language advice alongside Classifier2 probabilities. Grower sees both.
- **v4 Reward shaping:** VLM analysis feeds into the RL reward function as a qualitative signal — "is the surface improving?" — that complements the quantitative classifier score.

**Key question:** Is the VLM approach a replacement for Classifier2, a complement, or a v3.5 intermediate step? This is worth discussing as a group.

---

## 6. Framing for the Group — SDL Autonomy Level

Using the SDL autonomy framework from Tom et al. (Chem. Rev. 2024, Figure 1 — from Xu's course):

| Version | Hardware Autonomy | Software Autonomy | SDL Level |
|---|---|---|---|
| **v2 (now)** | Category 1 (single task: screengrab + log) | Category 0 (human-driven) | Automated experiment |
| **v3 (next)** | Category 1-2 (screengrab + direct Modbus + classification) | Category 1 (single iteration: classifier advises) | Level 2 |
| **v4 (future)** | Category 2 (automated workflow: classify + RL + PID) | Category 2 (closed-loop: RL policy updated from growth results) | Level 3 |

**The 2-year goal should be Level 2 (v3) with a strong database foundation for Level 3 (v4).** This is realistic given Xu's timeline advice.

---

## 7. Suggested Next Steps (For Group Discussion)

**Data & Infrastructure:**
1. **Get Growth Monitor into routine use during actual growths.** Every unrecorded growth is a missed data point.
2. **Execute the lab visit checklist** — Eurotherm IP discovery, kSA Auto Export, TemperaSure Macro Mode.
3. **Add Time vs Temperature live plot** and manual-entry fields for flux/pressure.
4. **Define the growth session schema** — standardize what a "complete" session record looks like, matching the proposal's 7 parameters.

**ML & Modeling:**
5. **Continue RHEED labeling** — 170/1124 (15%) is not enough. Assign this to undergrads.
6. **VLM exploration** — curate in-context sample sets, tune prompts with expert growers, benchmark against Classifier2 (Justin's lead).
7. **Read more literature** — SDL applications in MBE/thin-film, VLMs for materials characterization, physics-informed BO (GPax). Everyone should be reading, not just coding.

**Team & Planning:**
8. **Decide on sensor priorities** — which new sensors/integrations to pursue first.
9. **Assign workstreams** — lab hardware, ML/data pipeline, software development. Who owns what?

---

## 8. Open Questions for the Team

- What is the Eurotherm controller model on OMBE? Can we get its IP?
- Does the ion gauge have a digital or analog output we could tap?
- How many growths per week are happening right now? What's realistic for data collection rate?
- Who on the team is best positioned for: (a) lab hardware investigation, (b) RHEED labeling, (c) software development?
- Should we set a concrete target (e.g., 50 fully-recorded growth sessions by end of quarter)?
- **VLM approach:** What RHEED features do expert growers look for beyond reconstruction type? (Satellite peaks, Kikuchi lines, spot sharpness, streak-to-spot ratio, background intensity?) We need their vocabulary to tune prompts.
- **VLM vs Classifier2:** Is the VLM a replacement, a complement, or a fallback? What's the right role for each?
- **Literature gaps:** What papers should we all be reading? Assign 2-3 papers per person for the next meeting.
