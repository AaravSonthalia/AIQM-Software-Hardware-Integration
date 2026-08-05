# Weak-primary lambda=0.1 shadow package

This directory is the self-contained runtime package for the GUI's optional
four-class shadow display. It contains the registered DINOv2 encoder, all 36
lambda=0.1 cross-validation heads (4 folds x 3 pair runs x 3 seeds), and the
minimal Python model definitions required to load them.

The source files and weights were copied from
`D:/AI4MBE/RHEEDClassify/Classifier2` on 2026-08-05. The adjacent research
checkout was at Git commit `89c5c943c91b814d1d60f42aaacaea14bb044abf`;
`backbones.py`, `weak_primary_model.py`, and `davidson_pairwise.py` were local
untracked source files in that checkout, so this GUI commit is the authoritative
snapshot for the packaged runtime.

The `.pth` files are ordinary Git blobs so workstations whose networks cannot
reach GitHub's separate LFS/S3 endpoint can fetch the complete branch. The GUI
validates the encoder SHA-256 metadata and every checkpoint's
`weak_shadow_only` contract before inference. These conditional probabilities
are diagnostic only: they have no 1x1/none class, are not surface fractions,
and are never actionable.
