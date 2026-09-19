# Harness notes — PS3 controller (widen 15 mm + left-handed mirror)

`tests/task/harness/harness.py` is the grading harness for this task.  This note
records what was changed in it, why, and the score envelopes it produces for the
reference and for every shipped example.  (Remove this file if the submission
must stay limited to harness + task metadata.)

## Grading contract kept

- **Geometry only**: mass properties, products of inertia, body centroids, body
  bounding boxes, face areas.  Never feature names, body names, feature-tree
  structure, or coincidence with the reference file — a candidate may remodel
  the part any way it likes.
- **Weights unchanged and continuous** (do not sum to anything else):
  `widened by 15 mm` 1.0, `clusters at mirrored positions` 1.0,
  `left-handed layout achieved` 1.0, `no new control interference` 0.5,
  `no unrequested changes` 0.25, `glyphs preserved` 0.25 → `max_score = 4.0`
  (matches `task.toml`).
- **Rebuild health is a gate**: a candidate with new feature-tree errors vs the
  frozen seed census scores 0 on every component instead of being measured
  against garbage.

## What changed, and why

1. **Body matching uses the fingerprint to break near-ties** (baseline body ↔
   candidate body by expected mirrored+widened position).  The START/SELECT
   bodies sit only 4.7 cm apart, so position alone could pair a label pair that
   never moved with its mirror twin and read it as correctly swapped.
2. **Sides are witnessed per body, and cluster position is the mirror of the
   cluster centroid.**  Every shoulder pair and both button diamonds are
   centred on, or straddle, the plane, so the old role-mean side test was
   vacuously true — mirroring bodies individually and averaging cancels back
   onto the plane, which is how a cluster that never moved looked correctly
   placed.  Re-spacing targets are bands (mirror-only … mirror+spread), because
   the prompt allows either.
3. **The one-sided housing glyph cluster (port lights) is re-found by its
   mirror-invariant area multiset, with a geometric fallback** (tiny faces in
   the seed's Y/Z band near the top-rear edge; Y/Z are mirror-invariant, so a
   mirrored or rebuilt glyph is still located).  It must stay on its original
   side and move only by the widening — and if it cannot be found at all the
   guard **FAILS** rather than passing as unverifiable.  This is the witness
   that catches `adversarial_text_mirrored_incorrectly`.
4. **Chirality and shell-moment guards are continuous, but their pass set is
   unchanged**: any genuine sign flip is full credit (the reference re-cuts the
   D-pad engraving, so |Ixy| magnitudes are not comparable with the seed's), a
   kept sign tapers to zero, and a collapsed witness sits mid-scale instead of
   paying out full credit.  Guards combine by `min`.
5. **`glyphs preserved` scores engraving PRESENCE per matched body** (the
   reference rebuilds the D-pad engraving, 11 → 5 faces per button), as half the
   per-body mean plus half the worst body, so a stripped engraving cannot hide
   behind the housing's 276 port/vent faces; a body that vanished scores 0 for
   it, and nothing matched at all scores 0 for the component.
6. **`no unrequested changes` also checks the global X span** against the
   candidate's own measured widening (only over-growth is charged, so a
   legitimate widen is never punished twice while an X stretch/scale is
   caught) **and fingerprints the bodies that land in no role at all**
   (here the two 2 mm slivers c12/c13).  Background: the X extremes of this
   part belong to the housing (135.5 mm off the plane vs 88 mm for the
   outermost control) and the reference re-shells it, so a raw document-wide
   span comparison produced a live false positive on the reference (21.9 mm of
   span growth against 15.0 mm of pair growth, while the solution's own STL
   sidecar shows the intended 14.96 mm).
7. **No component pays out full credit for an empty measurement** — every
   `if scores else 1.0` fallback is gone.
8. **Diagnostics only**: `capture()` prints per-stage timings to stderr (no COM
   calls, no scoring effect), so a slow run is distinguishable from a hung
   SolidWorks session.


## Score envelopes

Measured live on Windows against a licensed, running SolidWorks session
(`python tests/task/harness/harness.py <model>.SLDPRT`); the harness prints
`{"score", "max_score", "passed", "subscores"}`.  Both branches score
identically: `main` and `developers` ship byte-identical harness/task files
(only `.gitignore` differs) and the `.SLDPRT` models are untracked/shared.

| model | score | passed | components that lose credit |
|---|---|---|---|
| `solution/solution.SLDPRT` | **4.0000 / 4.0** | **true** | — |
| `adversarial_text_mirrored_incorrectly` | 3.0000 / 4.0 | false | `left-handed layout achieved` = 0.0 (one-sided glyph cluster changed sides) |
| `adversarial_widened_by_30mm` | 3.0000 / 4.0 | false | `widened by 15 mm` = 0.0 (pairs grew 30 mm, not 15) |
| `adversarial_widened_15mm_clusters_at_original_spacing` | 1.5667 / 4.0 | false | `widened by 15 mm` = 0.0, `clusters at mirrored positions` = 0.4, `no new control interference` = 0.0, `left-handed layout achieved` = 0.6667 |
| `adversarial_only_one_button_cluster_mirrored` | *not measured live* | — | expected (offline mock, and by construction): `clusters at mirrored positions` and `left-handed layout achieved` lose part of their credit, because one diamond stays where it was while the other moves |
| `adversarial_unrequested_change_elsewhere` | *not measured live* | — | expected: `no unrequested changes` loses the credit its shipped change earns; the change is invisible to the other components by design |
| `adversarial_missing_glyphs` | 4.0000 / 4.0 | true | **none — see the asset caveat below** |
| `adversarial_unwidened_shell_with_correct_clusters` | 0.0000 / 4.0 | false | all six: rebuild gate (34 new feature-tree errors vs a seed census of 0) |
| `adversarial_feature_tree_with_errors` | *not measured live* | — | expected 0.0: rebuild gate (its tree is the one shipped riddled with errors) |

Rows marked *not measured live* are the three models that could not be run in
this session: the SolidWorks session had to be relaunched, and the first
`EditRebuild3()` on a cold session blocked for >12 minutes (the same stall the
stage diagnostics exist to expose), so the batch was stopped rather than
reported as a failure.  Re-running the same one-line command once SolidWorks is
warm fills them in:

```powershell
python tests\task\harness\harness.py examples\<name>\<name>.SLDPRT
```

## Caveats, stated plainly

- **`adversarial_missing_glyphs` cannot be docked by a geometry-only harness.**
  Its shipped `.STL` sidecar is byte-identical (same SHA256) to
  `solution.STL`, and its live capture is indistinguishable from the
  reference's (same body count, same small-face counts, `rebuild ok=True`), so
  the glyphs are not missing *geometrically*.  Detecting it would require
  grading colours or feature-tree state, which the task's contract forbids.
- **`adversarial_unwidened_shell_with_correct_clusters` loses everything**
  because its feature tree does not rebuild (34 hard errors) and the gate is
  deliberately all-or-nothing; measured geometrically it would lose only the
  widen component.
- **An L1/L2 ↔ R1/R2 swap is not geometrically witnessable** on this part: the
  shoulder twins are exact mirrors (13449.00 vs 13448.96 mm³) and each pair is
  symmetric about the plane, so "swapped" and "not swapped" describe the same
  solid.  The harness grades what is observable there (matched-body position
  versus the candidate's own widening, chirality sign flip, pair consistency)
  instead of pretending to see more.
- **Cost**: one model ≈ 210–235 s over COM (health gate ≈ 45 s, small-face
  scans ≈ 70 s, housing faces ≈ 60 s, 21 boolean intersections ≈ 16 s), inside
  the harness's declared `timeout_s = 300`.
