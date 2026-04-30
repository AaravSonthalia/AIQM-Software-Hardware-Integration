# Email draft — to Justin Meng on unlabeled RHEED data

**Status:** Draft — review/edit before sending. Open since Apr 17 internal AI-MBE meeting.

---

**Subject:** Question on unlabeled RHEED data — what's worth doing with the long tail?

Hi Justin,

A question came up at our Apr 17 internal meeting that I'd like your read on.

Where we are now: we have 1,124 RHEED trajectory frames, of which **~170 (15%) carry
human reconstruction labels**. Classifier2 trained on the ideal-split subset
(159 images, 4 classes) achieves 100% accuracy on the held-out ideal split, but
the long tail of unlabeled frames is currently sitting on disk doing nothing.

That long tail is about to grow significantly. We just shipped an
**intelligent auto-capture system** in the Growth Monitor GUI (pixel-diff
change detector, default-OFF, see `docs/change_detector_algorithm.md` if
you're curious) that flags ~3 events per typical session. We've also added
heartbeat anchor capture every 10 minutes. Even at default settings,
we're looking at ~30–50 *useful* unlabeled frames per growth, accumulating
across every session as the lab adopts the GUI.

The question: **what's the highest-leverage thing we can do with all this
unlabeled data, given that we're not going to hand-label most of it?**

I see a few candidate directions, and I'm not sure which is the most
promising — would value your read:

1. **Self-supervised pre-training of a better encoder.** Our current encoder
   is SimCLR-ResNet18 trained on a small dataset. Pre-training on the long
   tail (MoCo, BYOL, DINOv2) might give us a better backbone for Classifier2.
   But the marginal gain over our current 100% ideal-split accuracy is
   unclear, and the pairwise holdout is already 88%.

2. **Out-of-distribution / drift detection.** Use the unlabeled frames to
   characterize "normal" RHEED at each reconstruction, then flag live
   frames that fall outside that distribution as low-confidence. Would
   give the GUI a model-uncertainty signal to display alongside the
   classification (e.g., "Classifier says RT13 but this looks unusual —
   recommend manual confirmation"). Tools like Alibi Detect implement
   the standard methods here.

3. **Unsupervised clustering to surface unknown reconstructions.** Run
   the embedding through HDBSCAN / k-means and see if any clusters fall
   outside the 4 known classes. Could find reconstructions we didn't
   know to label — particularly useful for FeSe where our taxonomy is
   not yet established. Rahim's NMF paper did something similar offline;
   we'd want it as a continuous monitoring pass.

4. **Active learning loop.** Pick the most informative unlabeled frames
   (high model uncertainty, or far from any labeled frame in embedding
   space) and surface them in the GUI for the grower to label as part
   of their normal workflow. Turns the long tail into a labeling
   pipeline rather than dead weight.

My instinct says (2) and (4) have the cleanest path to value because they
plug into the existing GUI without requiring a separate research project —
but you have a much better sense of the tradeoffs than I do. Anything
obvious I'm missing? Anything you'd cross off the list?

Happy to pull frames into a shared dataset directory if you want to
prototype against the long tail directly.

Thanks,
AJ

---

## Notes for AJ before sending

- **Length check:** ~350 words — about right for a substantive ask, not
  overwhelming. Cut paragraphs 1–2 if you want to tighten further.
- **One thing I'd consider adding:** a specific question about whether
  Justin's group would want to co-author on whatever comes out of this.
  The Apr 14 meeting framing was "publishable artifacts that target
  data-scarcity / dataset-building narratives" — that's exactly the
  story this work tells.
- **Reference materials to have ready** if Justin asks follow-ups:
  - `docs/change_detector_algorithm.md` — the new auto-capture writeup
  - `docs/literature_comparison.md` — already covers Alibi Detect for #2
  - `Classifier2/artifacts/` — the current model + training set
  - 388-frame Rahim trajectory at `~/Downloads/rahim_2022_02_04/` —
    a concrete trajectory he could prototype against
