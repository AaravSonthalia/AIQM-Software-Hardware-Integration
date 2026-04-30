# RHEED ML Literature Comparison Table

**Prepared for lab meeting — Mar 6, 2026**

## Overview

This table compares existing RHEED analysis tools and ML approaches in the
literature against our Growth Monitor system. **Key insight:** no prior work
has implemented real-time ML-triggered auto-capture for RHEED. Our Tier 2/3
auto-capture system (embedding changepoints + classification change detection)
would be novel.

---

## Comparison Table (for Justin)

| Work | Problem Addressed | Phase or Reconstruction? | Method | Real-time? | Auto-capture? | Usefulness to Us |
|---|---|---|---|---|---|---|
| **RHAAPSODY** (PNNL, 2023) | Unsupervised phase discovery in RHEED video | Phase (unsupervised clustering) | VGG16 embeddings → custom similarity-matrix changepoint detection → Louvain graph clustering | Post-hoc only | Offline changepoints (not live triggers) | **Partial** — concept of embedding changepoints is useful, but their code is TensorFlow-locked and uses custom changepoint method (not `ruptures.Pelt` as we assumed). We'd reimplement with our SimCLR encoder in PyTorch. |
| **rhana** (LHT, 2022) | Quantitative RHEED analysis toolkit | Neither — signal processing | Rolling ball background subtraction, FFT periodicity tracking (`PeriodicityTracker`), U-Net streak/spot segmentation | Yes (streaming) | No | **High** — PeriodicityTracker is exactly what our oscillation monitoring needs; rolling ball preprocessing improves noise robustness; U-Net model is PyTorch-compatible. Cleanest codebase of the group. |
| **Classifier2** (ours, 2025-26) | Classify STO surface reconstructions | Reconstruction (5-class supervised) | Bradley-Terry reward model on SimCLR ResNet18 encoder; trained via pairwise human preferences (RLHF-style) | Offline training, real-time inference (~80ms CPU) | Tier 3 trigger (JS-divergence on classification change) | **Core** — already integrated via `ClassifierBridge`. 100% ideal-split accuracy (30/30), 88.3% pairwise holdout. |
| **Rahim NMF** (Yang Lab thesis) | Unsupervised RHEED pattern decomposition | Phase (unsupervised) | NMF (3 components) + DBSCAN/K-means clustering on STO RHEED video frames | Offline | No | **Low** — same STO dataset as ours, useful as unsupervised baseline comparison only. |
| **PyRHEED** (Yu, 2020) | Quantitative RHEED streak analysis | Neither — peak fitting | Gaussian/Voigt peak fitting on RHEED streak intensity profiles | No (interactive GUI) | No | **Future** — streak width/spacing metrics could augment quality assessment beyond classification. |
| **FRHEED** (Young, 2023) | Live RHEED monitoring GUI | Neither — visualization | Threading + ROI selection + FFT oscillation tracking, pip-installable | Yes (live GUI) | No (manual snapshot) | **Low** — architecture reference only; our GUI already has all these features. |
| **RHEED_2D_ML** (Chang, 2021) | Unsupervised RHEED pattern clustering | Phase (unsupervised) | PCA + K-means on 2D RHEED video frames | Offline | No | **Low** — PCA vs NMF comparison for dimensionality reduction baseline. |
| **RHEED Viewer** (Jacques, Yang Lab) | Live RHEED + pyrometer monitoring | Neither — data acquisition | ROI intensity tracking + pywinauto pyrometer screen scraping | Yes (live) | No (manual) | **Direct predecessor** — pyrometer scrape pattern inspired our serial driver approach. |

---

## Additional Tools Discovered (broader search)

| Tool | Domain | Relevance |
|---|---|---|
| **changepoint-online / NPFocus** | Online changepoint detection | Could replace RHAAPSODY's custom method for Tier 2 — designed for streaming data |
| **STUMPY (FLOSS)** | Streaming time series anomaly detection | Alternative to PELT for detecting RHEED pattern transitions in real-time |
| **BOCD** (Bayesian Online Changepoint Detection) | Probabilistic changepoint detection | Most principled approach — gives posterior probability of changepoint at each timestep |
| **PC-Gym** | RL for process control | Relevant to Anneal RL — provides environments for tuning RL policies on chemical/thermal processes |
| **HELAO-async / Bluesky** | Self-driving lab frameworks | Architecture reference for autonomous experiment orchestration |
| **Alibi Detect** | Distribution drift detection | Could detect when RHEED patterns drift outside training distribution (model confidence monitoring) |

---

## Our Three-Tier Auto-Capture Architecture (Novel)

| Tier | Method | Detector Class | Status | Novelty |
|---|---|---|---|---|
| **1** | Global mean intensity change | `IntensityChangeDetector` | DONE | Standard (baseline) |
| **2** | SimCLR embedding cosine distance + online changepoint detection | `EmbeddingChangeDetector` (planned) | TODO | Adapts RHAAPSODY concept for real-time; use BOCD or STUMPY instead of their custom method |
| **3** | Classifier2 output distribution change (JS-divergence) | `ClassificationChangeDetector` | DONE | **Novel** — semantic auto-capture triggered by reconstruction transitions |

**Tier 3** is the most semantically meaningful: it captures exactly the events
we care about (surface reconstruction transitions during MBE growth).

**Tier 2** is the most robust to novel patterns — it doesn't require a trained
classifier and can detect changes the classifier wasn't trained for.

**No prior work has combined ML classification with real-time auto-capture for RHEED.**

---

## STO Surface Reconstructions (Physics Context)

| Reconstruction | Notation | Conditions | Significance |
|---|---|---|---|
| (1x1) | Unreconstructed | Low T, O-rich | Starting/reference surface |
| Twinned(2x1) | Double periodicity, twinned domains | ~500-600 C, reducing | Ti-rich surface, pre-growth prep |
| c(6x2) | Centered rectangular | ~600-700 C, UHV | Clean, well-ordered; good growth starting point |
| (sqrt13 x sqrt13) R33.7 deg | Large unit cell, rotated | ~700-900 C, reducing | Ti-rich, high-quality surface |
| HTR (High-T Reconstruction) | Not fully characterized | >900 C | Extreme conditions; less studied |

These 5 classes are the output of our Classifier2 model and correspond to the
surface science of SrTiO3 (001) under varying temperature and oxygen partial
pressure during MBE growth.

---

## Key References

1. RHAAPSODY: https://github.com/pnnl/RHAAPSODY
2. rhana: https://github.com/AuroraLHT/rhana
3. PyRHEED: https://github.com/yux1991/PyRHEED
4. FRHEED: https://github.com/ecyoung3/FRHEED
5. RHEED_2D_ML: https://github.com/youngjunchang/RHEED_2D_ML
6. STUMPY: https://github.com/TDAmeritrade/stumpy
7. changepoint-online: https://github.com/Alan-Turing-Institute/changepoint-online
8. PC-Gym: https://github.com/MaximilianB2/pc-gym
