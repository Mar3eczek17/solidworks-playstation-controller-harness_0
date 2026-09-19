#!/usr/bin/env python3
"""
Task 1 (SolidWorks) -- PS3 controller: widen 15 mm + left-handed mirror.

Grades a candidate .SLDPRT against prompt/input.json (frozen measurements of
PS3-Controller_Base.SLDPRT).  Grading is geometry-only: mass properties,
inertia signs, centroids, face-area moments -- never feature or body names
(body order is unstable and candidates may remodel).

The controller's mirror plane is x = plane_x_m (NOT x=0).  A correct edit:
  - widens by 15 mm total (mirror-pair separations grow +15 mm)
  - moves the dpad diamond to the right and the face-button diamond to the
    left of the plane (left-handed layout), each as a RIGID unit
  - keeps chiral one-sided housing features (port lights, text) on their
    original side -- the housing is updated, not mirrored wholesale
  - adds no interference among the controls
  - changes nothing else (Y/Z spans, non-control body shapes)

What this harness leans on, and why (see also task.toml's notes):
  * Baseline bodies are matched to candidate bodies by EXPECTED mirrored +
    widened position, with the body fingerprint breaking near-ties.  That
    fingerprint term matters: the START/SELECT bodies sit only 4.7 cm apart,
    so position alone let a pair that never moved be paired with its mirror
    twin and read as correctly swapped.
  * Symmetry is the enemy of side witnesses.  Every shoulder pair and both
    button diamonds are centred on (or straddle) the plane, so a ROLE centroid
    says nothing; sides are therefore witnessed per BODY, and the cluster
    position is measured by mirroring the CLUSTER centroid (mirroring bodies
    individually and averaging cancels back onto the plane).
  * The one-sided housing glyph cluster (port lights) must be found again on
    the candidate -- by area multiset first, then by the seed's geometric band
    (Y/Z are mirror-invariant, so a mirrored or rebuilt glyph is still
    located).  If it cannot be found at all, the guard FAILS rather than
    scoring as unverifiable: "the shell was updated, not flipped" cannot be
    claimed from an empty measurement.
  * An empty measurement never earns full credit anywhere (no `else 1.0`).
"""

from __future__ import annotations

import itertools
import json
import math
import os
import sys
import time
from collections import Counter
from itertools import permutations
from pathlib import Path

HERE = Path(__file__).resolve().parent
TASK_DIR = HERE.parent


_REPO_ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(_REPO_ROOT / "SolidWorks"))  # shared modules, if any
sys.path.insert(0, str(_REPO_ROOT))                 # openai-eval-set/ (common)

from common import solidworks_measure as M        # noqa: E402
from common.solidworks_measure import z           # noqa: E402
from common import solidworks_capture as SC       # noqa: E402
from common.harness_base import Harness         # noqa: E402
from common.harness_base import (               # noqa: E402
    score_error, score_ratio, score_band, clamp01, _numeric,
)

BASELINE_PATH = TASK_DIR / "prompt" / "input.json"

PASS, FAIL, UNVERIFIABLE = "PASS", "FAIL", "UNVERIFIABLE"

POLICY = {
    "width_delta_m": 0.015,      # +15 mm total
    "half_m": 0.0075,            # 7.5 mm per side
    "handedness": "mirror about plane_x (dpad <-> face_buttons swap sides)",
}

TOL = {
    "width_m": 0.0015,           # pair-separation growth tolerance
    "cluster_pos_m": 0.004,      # cluster centre vs expected mirrored+widened
    "intra_cluster_m": 0.001,    # rigid diamond internal spacing
    "straddle_m": 0.012,         # cluster centred on the plane = no side
    "chiral_floor": 1e-3,        # inertia-product ratio below = unverifiable
    "fp_reshape": 8e-2,          # fingerprint drift = reshaped
    "span_m": 0.006,             # Y/Z span drift allowed (remodel slack)
    "sig_floor_frac": 0.2,       # candidate |moment3| must be >= this frac of
                                 # the baseline's to count as a valid witness
    "intf_growth_m3": 5e-9,      # 5 mm^3 of new control interference allowed
    "glyph_floor": 0.25,         # fraction of a body's seed engraved-face
                                 # count that still counts as "glyph present"
    "xspan_m": 0.004,            # global X growth vs the candidate's OWN
                                 # widening: perfect agreement
    "xspan_zero_m": 0.016,       # ... no credit left
    "onesided_x_m": 0.010,       # port-light cluster X drift still credited as
                                 # "moved only by the widening" (a wholesale
                                 # mirror puts it ~160 mm out)
    "onesided_x_zero_m": 0.040,  # ... guard credit gone
}

MIRROR_ROLES = ["dpad", "face_buttons", "sticks", "triggers", "bumpers",
                "centre"]
# clusters with a determinate expected position under mirror+widen.  centre
# (select/start/ps) is EXCLUDED: it hugs the plane, and "adjust spacing
# accordingly" is satisfiable by mirror-only or mirror+spread -- the
# reference uses mirror-only.  Its side/straddle is still graded in c4.
POSITION_ROLES = ["dpad", "face_buttons", "sticks", "triggers", "bumpers"]
CHIRAL_ROLES = ["sticks", "triggers", "bumpers", "dpad"]
RIGID_ROLES = ["dpad", "face_buttons"]
# engineer-remodel exemptions for the reshape check
RESHAPE_EXEMPT = ["dpad", "face_buttons", "housing", "centre"]

SWDOC_PART = 1

# body-volume windows (m^3) used by role assignment
BUTTON_VOL_MIN = 0.4e-6
BUTTON_VOL_MAX = 5.0e-6
PAIR_VOL_MIN = 5.0e-6
DIAMOND_LINK_M = 0.035          # X-Z proximity that chains a button diamond
HOUSING_FRACTION = 0.5          # of the largest body's volume
SMALL_FACE_AREA = 15e-6         # arrow/glyph engraving faces (dpad witness)

# port-light glyph cluster constants
LIGHT_FACE_MAX_AREA = 8e-6
LIGHT_MIN_OFFPLANE_M = 0.040
LIGHT_AREA_TOL_FRAC = 0.02
LIGHT_CLUSTER_RADIUS_M = 0.020
LIGHT_MIN_MATCHES = 3

# Weight (in metres of equivalent position error) given to a body-fingerprint
# mismatch when matching baseline bodies to candidate bodies.  Positions still
# dominate -- a body on the wrong SIDE of the plane is ~0.16 m out -- but a
# 4.7 cm position ambiguity (the START/SELECT pair's mirrored positions are
# that close) must not let a body be paired with a differently-shaped twin,
# which is what made "labels left where they were" look correct.
MATCH_FP_WEIGHT = 0.15


# --------------------------------------------------------------------------
# role assignment (pure geometry)
# --------------------------------------------------------------------------

def sym_plane(bodies):
    """Mirror plane x from the mode of same-fingerprint pair midpoints."""
    planes = []
    for a, b in itertools.combinations(bodies, 2):
        if abs(a["volume_m3"] - b["volume_m3"]) > 1e-9:
            continue
        if abs(a["area_m2"] - b["area_m2"]) > 1e-7:
            continue
        ca, cb = a["centroid_m"], b["centroid_m"]
        if abs(ca[1] - cb[1]) > 2e-3 or abs(ca[2] - cb[2]) > 2e-3:
            continue
        planes.append((ca[0] + cb[0]) / 2)
    if not planes:
        return None
    buckets = Counter(round(p * 1000) for p in planes)
    best = buckets.most_common(1)[0][0]
    members = [p for p in planes if abs(p * 1000 - best) <= 1.5]
    return sum(members) / len(members)


def _diamond_groups(small):
    def near(a, b):
        ca, cb = a["centroid_m"], b["centroid_m"]
        return math.hypot(ca[0] - cb[0], ca[2] - cb[2]) < DIAMOND_LINK_M

    groups, used = [], set()
    for b in small:
        if b["id"] in used:
            continue
        stack, comp = [b], []
        used.add(b["id"])
        while stack:
            x = stack.pop()
            comp.append(x)
            for o in small:
                if o["id"] not in used and near(x, o):
                    used.add(o["id"])
                    stack.append(o)
        groups.append(comp)
    return groups


def assign_roles(bodies, small_face_count):
    """{role: [ids]} from geometry alone."""
    roles = {}

    bysize = sorted(bodies, key=lambda b: -b["volume_m3"])
    housing = [bysize[0]["id"]]
    for b in bysize[1:]:
        if b["volume_m3"] > HOUSING_FRACTION * bysize[0]["volume_m3"]:
            housing.append(b["id"])
    roles["housing"] = housing

    small = [b for b in bodies
             if BUTTON_VOL_MIN < b["volume_m3"] < BUTTON_VOL_MAX]
    diamonds = [g for g in _diamond_groups(small) if len(g) == 4]
    diamonds.sort(key=lambda g: -sum(small_face_count.get(b["id"], 0)
                                     for b in g) / len(g))
    if len(diamonds) >= 2:
        roles["dpad"] = [b["id"] for b in diamonds[0]]
        roles["face_buttons"] = [b["id"] for b in diamonds[1]]
    elif len(diamonds) == 1:
        roles["dpad"] = [b["id"] for b in diamonds[0]]
        roles["face_buttons"] = []
    else:
        roles["dpad"], roles["face_buttons"] = [], []

    diamond_ids = {b["id"] for g in diamonds for b in g}
    roles["centre"] = [b["id"] for b in small if b["id"] not in diamond_ids]

    taken = set(housing) | diamond_ids | set(roles["centre"])
    mids = [b for b in bodies
            if b["id"] not in taken and b["volume_m3"] >= PAIR_VOL_MIN]
    pairs = []
    for a, b in itertools.combinations(mids, 2):
        if abs(a["volume_m3"] - b["volume_m3"]) > 1e-8:
            continue
        if abs(a["area_m2"] - b["area_m2"]) > 1e-6:
            continue
        if abs(a["centroid_m"][2] - b["centroid_m"][2]) > 3e-3:
            continue
        pairs.append((a, b))

    def pz(p):
        return (p[0]["centroid_m"][2] + p[1]["centroid_m"][2]) / 2

    used = set()
    if pairs:
        st = max(pairs, key=pz)          # sticks sit highest (z)
        roles["sticks"] = [st[0]["id"], st[1]["id"]]
        used = set(roles["sticks"])
        low = [p for p in pairs
               if p[0]["id"] not in used and p[1]["id"] not in used]
        low.sort(key=lambda p: -p[0]["volume_m3"])
        if low:                          # larger low pair = triggers
            roles["triggers"] = [low[0][0]["id"], low[0][1]["id"]]
            used |= set(roles["triggers"])
        if len(low) >= 2:
            roles["bumpers"] = [low[1][0]["id"], low[1][1]["id"]]
    return roles


def _diamond_radius(bodies_by_id, ids):
    """Mean X-Z distance from a 4-button diamond's centroid to its buttons.

    This is a rigid property of a button cluster: it survives the swap, the
    mirror and the widen, and -- unlike the engraved-face count -- it does not
    change when the engineer rebuilds a glyph.
    """
    pts = [bodies_by_id[i]["centroid_m"] for i in ids]
    cx = sum(p[0] for p in pts) / len(pts)
    cz = sum(p[2] for p in pts) / len(pts)
    return sum(math.hypot(p[0] - cx, p[2] - cz) for p in pts) / len(pts)


def anchor_button_roles(roles, bodies, baseline):
    """Confirm or flip the dpad/face_buttons labels against the seed.

    assign_roles() labels whichever 4-button diamond carries more sub-15mm2
    faces as the D-pad.  That is fragile: the reference remodel leaves the
    face-button cluster with MORE small faces than the D-pad, so the two
    labels come out swapped and every position/chirality check downstream then
    compares against the wrong bodies.  The two diamonds have distinct button
    spacings (here 14.8mm vs 19.8mm) that the swap and the widen both leave
    untouched, so the seed's spacings are used as the tie-breaker.
    """
    d_ids = roles.get("dpad") or []
    f_ids = roles.get("face_buttons") or []
    if len(d_ids) != 4 or len(f_ids) != 4 or not baseline:
        return roles
    b_roles = baseline.get("roles") or {}
    b_d = b_roles.get("dpad") or []
    b_f = b_roles.get("face_buttons") or []
    if len(b_d) != 4 or len(b_f) != 4:
        return roles
    cand = {b["id"]: b for b in bodies}
    seed = {b["id"]: b for b in baseline.get("bodies", [])}
    if not all(i in cand for i in d_ids + f_ids):
        return roles
    if not all(i in seed for i in b_d + b_f):
        return roles
    try:
        cd, cf = _diamond_radius(cand, d_ids), _diamond_radius(cand, f_ids)
        sd, sf = _diamond_radius(seed, b_d), _diamond_radius(seed, b_f)
    except Exception:
        return roles
    if abs(sd - sf) < 1e-4:
        return roles
    keep = abs(cd - sd) + abs(cf - sf)
    flip = abs(cd - sf) + abs(cf - sd)
    if flip < keep:
        roles["dpad"], roles["face_buttons"] = f_ids, d_ids
    return roles


# --------------------------------------------------------------------------
# capture helpers
# --------------------------------------------------------------------------

def _small_face_counts(raw_bodies):
    out = {}
    for idx, b in enumerate(raw_bodies):
        n = 0
        for f in (z(b.GetFaces) or []):
            try:
                if float(z(f.GetArea)) < SMALL_FACE_AREA:
                    n += 1
            except Exception:
                continue
        out[f"c{idx:02d}"] = n
    return out


def housing_faces(raw_bodies, housing_ids):
    """One pass over housing faces: (area, box-centre xyz) per face."""
    out = []
    for hid in housing_ids:
        idx = int(hid[1:])
        body = raw_bodies[idx]
        for f in (z(body.GetFaces) or []):
            try:
                a = float(z(f.GetArea))
                box = z(f.GetBox)
                c = [(float(box[k]) + float(box[k + 3])) / 2 for k in range(3)]
            except Exception:
                continue
            out.append((a, c))
    return out


def housing_signature(faces, plane_x):
    """Area-weighted 3rd moment of housing face box-centres about x=plane.
    Sign flips under a whole-part mirror."""
    m3, area_total = 0.0, 0.0
    for a, c in faces:
        m3 += a * (c[0] - plane_x) ** 3
        area_total += a
    return {"moment3_m5": m3, "face_area_m2": area_total,
            "faces": len(faces), "sign": 1 if m3 >= 0 else -1}


def find_light_cluster_seed(faces, plane_x, bbox):
    """Seed-side identification: tiny faces on the top-rear edge, well off
    the mirror plane."""
    y_top = bbox[4]          # y max
    z_rear = bbox[2]         # z min
    cand = [(a, c) for a, c in faces
            if a < LIGHT_FACE_MAX_AREA
            and abs(c[0] - plane_x) > LIGHT_MIN_OFFPLANE_M
            and (y_top - c[1]) < 0.030
            and (c[2] - z_rear) < 0.020]
    if len(cand) < LIGHT_MIN_MATCHES:
        return None
    best = None
    for a0, c0 in cand:
        grp = [(a, c) for a, c in cand
               if math.hypot(c[0] - c0[0], c[2] - c0[2])
               < LIGHT_CLUSTER_RADIUS_M]
        if best is None or len(grp) > len(best):
            best = grp
    if not best or len(best) < LIGHT_MIN_MATCHES:
        return None
    cx = sum(c[0] for _, c in best) / len(best)
    return {
        "areas_m2": sorted(a for a, _ in best),
        "centroid_m": [sum(c[k] for _, c in best) / len(best)
                       for k in range(3)],
        "side": 1 if cx >= plane_x else -1,
        "n_faces": len(best),
    }


def find_light_cluster_candidate(faces, plane_x, seed_cluster, bbox=None):
    """Candidate-side: match the seed's area multiset among housing faces.

    Falls back to the seed's purely geometric rule (tiny faces on the top-rear
    edge, well off the plane) when the areas have drifted: mirroring or
    rebuilding the glyphs changes how they split into faces, and the cluster's
    Y/Z band is mirror-invariant, so the fallback still finds the same
    one-sided feature -- and therefore still witnesses which side it is on.
    Returning None here is reserved for a housing whose one-sided glyph faces
    are gone altogether.
    """
    best = None
    if seed_cluster:
        hits = []
        for want in seed_cluster["areas_m2"]:
            pick, bd = None, None
            for a, c in faces:
                d = abs(a - want) / max(want, 1e-30)
                if d < LIGHT_AREA_TOL_FRAC and (bd is None or d < bd):
                    pick, bd = (a, c), d
            if pick:
                hits.append(pick)
        if len(hits) >= LIGHT_MIN_MATCHES:
            for a0, c0 in hits:
                grp = [(a, c) for a, c in hits
                       if math.hypot(c[0] - c0[0], c[2] - c0[2])
                       < LIGHT_CLUSTER_RADIUS_M]
                if best is None or len(grp) > len(best):
                    best = grp
            if best and len(best) >= LIGHT_MIN_MATCHES:
                best = {"areas_m2": sorted(a for a, _ in best),
                        "centroid_m": [sum(c[k] for _, c in best) / len(best)
                                       for k in range(3)],
                        "n_faces": len(best),
                        "method": "area_multiset"}
    if best is None and bbox is not None:
        geo = find_light_cluster_seed(faces, plane_x, bbox)
        if geo is not None:
            geo["method"] = "geometric_band (areas drifted -- mirrored or " \
                            "rebuilt glyph)"
            best = geo
    if best is None:
        return None
    cx = best["centroid_m"][0]
    best["side"] = 1 if cx >= plane_x else -1
    return best


def control_interference(raw_bodies, bodies, roles):
    """Boolean-intersect interference among control bodies and each control
    vs housing."""
    control_ids = []
    for role in ("dpad", "face_buttons", "sticks", "triggers", "bumpers",
                 "centre"):
        control_ids.extend(roles.get(role, []))
    housing_ids = roles.get("housing", [])

    idx_of = {b["id"]: i for i, b in enumerate(bodies)}
    test_pairs = []
    for a, b in itertools.combinations(control_ids, 2):
        test_pairs.append((a, b))
    for a in control_ids:
        for h in housing_ids:
            test_pairs.append((a, h))

    boxes = {bid: bodies[idx_of[bid]]["bbox_m"]
             for bid in set(control_ids) | set(housing_ids)}
    pairs, total = [], 0.0
    for a, b in test_pairs:
        if not M._boxes_overlap(boxes[a], boxes[b], M.BBOX_PAD):
            continue
        try:
            copy_a = z(raw_bodies[int(a[1:])].Copy)
            copy_b = z(raw_bodies[int(b[1:])].Copy)
            res, code = M._operations2(copy_a, copy_b)
            vol = 0.0
            if res:
                for rb in res:
                    try:
                        vol += float(rb.GetMassProperties(0.0)[3])
                    except Exception:
                        pass
            if vol > M.MIN_INTERFERENCE_VOLUME or code != 0:
                total += vol
                pairs.append({"a": a, "b": b, "volume_m3": vol,
                              "error_code": code})
        except Exception:
            pairs.append({"a": a, "b": b, "volume_m3": 0.0,
                          "error_code": -1})
    return {"tested_pairs": len(test_pairs), "pairs": pairs,
            "total_volume_m3": total}


def _stage(label, started=None):
    """Progress line on stderr, with the elapsed time of the previous stage.

    Diagnostics only: no COM calls, no effect on any score.  A grading run
    opens a 25 MB part, re-reads ~2500 face areas (531 of them on the housing)
    and runs up to 21 boolean intersections, so it takes minutes.  Without
    these lines a slow run and a hung SolidWorks session look exactly alike
    from outside -- which is how the first live smoke test presented itself:
    several minutes in, the log still held nothing but its own header.
    """
    now = time.perf_counter()
    print("  [capture] %-32s %6.1fs"
          % (label, 0.0 if started is None else now - started),
          file=sys.stderr, flush=True)
    return now


def capture(doc, baseline=None, plane_x=None):
    # "health gate" runs EditRebuild3() + a feature-error census (shared code),
    # so it is the stage that stalls first when SolidWorks is left showing a
    # modal dialog -- the label is deliberately the name of the work about to
    # happen, so the last line printed names the stage a caller is stuck in.
    t = _stage("health gate (rebuild + census)")
    rebuild = SC.health_gate(doc, baseline)
    t = _stage("capture_bodies", t)
    raw, bodies, boxes, gmin, gmax = SC.capture_bodies(doc)
    t = _stage("small-face scan (%d bodies)" % len(raw), t)
    smf = _small_face_counts(raw)
    t = _stage("role assignment", t)
    roles = assign_roles(bodies, smf)
    roles = anchor_button_roles(roles, bodies, baseline)
    P = plane_x
    if P is None:
        P = sym_plane(bodies)
    if P is None and baseline:
        P = baseline.get("plane_x_m")

    t = _stage("housing faces (houses %s)"
               % ",".join(roles.get("housing", []) or ["-"]), t)
    hfaces = housing_faces(raw, roles.get("housing", []))
    t = _stage("one-sided glyph cluster", t)
    if P is not None:
        sig = housing_signature(hfaces, P)
        seed_cluster = (baseline or {}).get("light_cluster")
        if seed_cluster:
            lights = find_light_cluster_candidate(hfaces, P, seed_cluster,
                                                  gmin + gmax)
        else:
            lights = find_light_cluster_seed(hfaces, P, gmin + gmax)
    else:
        sig = {"moment3_m5": 0.0, "sign": 0, "faces": 0}
        lights = None

    t = _stage("control interference", t)
    intf = control_interference(raw, bodies, roles)
    _stage("captured (%d bodies, %d faces)"
           % (len(bodies), sum(1 for _ in hfaces)), t)
    return {
        "document": str(z(doc.GetTitle)),
        "rebuild": rebuild,
        "global": {"bbox_m": gmin + gmax,
                   "width_m": gmax[0] - gmin[0]},
        "plane_x_m": P,
        "bodies": bodies,
        "roles": roles,
        "small_face_counts": smf,
        "housing_signature": sig,
        "light_cluster": lights,
        "interference": intf,
    }


def open_or_active(app, path=None):
    if path:
        from common import solidworks_session as sws
        # sws.open_document sweeps the whole session closed before any
        # fresh open -- confirmed live on 30_shampoo_bottle that a
        # multi-component assembly can silently reuse a same-named
        # component document left open from a previous candidate; this
        # task is a standalone part so that specific collision can't
        # happen, but sweeping first is still a strict improvement
        # (keeps the session from accumulating open documents run over
        # run) and keeps every SolidWorks harness on the same helper.
        doc, _opened_here = sws.open_document(app, os.path.abspath(path),
                                              doc_type=SWDOC_PART)
        if doc is not None:
            return doc
    doc = app.ActiveDoc
    if doc is None:
        raise RuntimeError("no active SolidWorks document and no path given")
    return doc


# --------------------------------------------------------------------------
# small helpers
# --------------------------------------------------------------------------

def d3(a, b):
    return math.sqrt(sum((u - v) ** 2 for u, v in zip(a, b)))


def sgn(v):
    return 1.0 if v >= 0 else -1.0


def fingerprint(b):
    I = b["inertia_com"]
    return [b["volume_m3"], b["area_m2"]] + sorted(
        [I["Ixx"], I["Iyy"], I["Izz"]])


def fp_distance(a, b):
    s = 0.0
    fa, fb = fingerprint(a), fingerprint(b)
    for u, v in zip(fa, fb):
        sc = max(abs(u), abs(v), 1e-30)
        s += ((u - v) / sc) ** 2
    return math.sqrt(s / len(fa))


def chirality(b):
    I = b["inertia_com"]
    diag = max(abs(I["Ixx"]), abs(I["Iyy"]), abs(I["Izz"]))
    return 0.0 if diag <= 0 else max(abs(I["Ixy"]), abs(I["Izx"])) / diag


def chirality_witness(b):
    I = b["inertia_com"]
    return "Ixy" if abs(I["Ixy"]) >= abs(I["Izx"]) else "Izx"


def mirror_widen_x(x, plane, half):
    o = x - plane
    return plane - sgn(o) * (abs(o) + half)


def widen_x(x, plane, half):
    """Same side of the plane, pushed `half` further out -- the position a
    non-handed, one-sided housing feature keeps under the widen."""
    o = x - plane
    return plane + sgn(o) * (abs(o) + half)


# --------------------------------------------------------------------------
# grader
# --------------------------------------------------------------------------

class Grader:
    def __init__(self, baseline, measured):
        self.bl = baseline
        self.ms = measured
        self.P = baseline["plane_x_m"]
        self.B = {b["id"]: b for b in baseline["bodies"]}
        self.C = {c["id"]: c for c in measured["bodies"]}
        self.Broles = baseline["roles"]
        self.Croles = measured["roles"]
        self.match = self._match()
        self.actual_half = self._actual_half()

    def _match(self):
        """Per-role matching, baseline -> candidate, by expected
        mirrored+widened position (min-cost permutation)."""
        m = {}
        half = POLICY["half_m"]
        for role, bids in self.Broles.items():
            cids = self.Croles.get(role, [])
            if not cids:
                for bid in bids:
                    m[bid] = {"cand": None, "role": role}
                continue
            exp = {bid: [mirror_widen_x(self.B[bid]["centroid_m"][0],
                                        self.P, half)]
                   + self.B[bid]["centroid_m"][1:]
                   for bid in bids}
            if len(bids) == len(cids) and len(bids) <= 6:
                best, bc = None, float("inf")
                for perm in permutations(cids):
                    cost = sum(d3(exp[b], self.C[c]["centroid_m"])
                               + MATCH_FP_WEIGHT * fp_distance(self.B[b],
                                                               self.C[c])
                               for b, c in zip(bids, perm))
                    if cost < bc:
                        bc, best = cost, perm
                for b, c in zip(bids, best):
                    m[b] = {"cand": c, "role": role,
                            "residual": d3(exp[b], self.C[c]["centroid_m"])}
            else:
                bs = sorted(bids, key=lambda i: -self.B[i]["volume_m3"])
                cs = sorted(cids, key=lambda i: -self.C[i]["volume_m3"])
                for i, b in enumerate(bs):
                    m[b] = {"cand": cs[i] if i < len(cs) else None,
                            "role": role, "via": "volume_rank"}
        return m

    def _actual_half(self):
        """Candidate's actual per-side widening (m) from matched mirror pairs
        (sticks/triggers/bumpers). Falls back to the policy target when no
        pairs survive matching -- keeps c2's position check usable."""
        target = POLICY["width_delta_m"]
        half_target = POLICY["half_m"]
        deltas = []
        for role in ("sticks", "triggers", "bumpers"):
            bids = self.Broles.get(role, [])
            if len(bids) != 2:
                continue
            seed = abs(self.B[bids[0]]["centroid_m"][0]
                       - self.B[bids[1]]["centroid_m"][0])
            cs = [self.match[b]["cand"] for b in bids]
            if None in cs:
                continue
            got = abs(self.C[cs[0]]["centroid_m"][0]
                      - self.C[cs[1]]["centroid_m"][0])
            deltas.append(got - seed)
        if not deltas:
            return half_target
        return sum(deltas) / len(deltas) / 2.0

    def _candidate_role_ids(self):
        """Every candidate body id that landed in some role."""
        ids = set()
        for v in self.Croles.values():
            ids.update(v or [])
        return ids

    def _matched_extent_x(self, candidate):
        """(min_x, max_x) in metres over the GRADED bodies' own boxes.

        `candidate=False` -> the baseline bodies; True -> their matched
        candidate counterparts.  Two reasons to prefer this over the
        document-wide union: a stray/extra body left in the document cannot
        inflate the span, and both sides are then guaranteed to span the same
        set of bodies.  Returns None when the boxes are unavailable.
        """
        lo = hi = None
        for bid, m in self.match.items():
            if candidate:
                if not m.get("cand"):
                    continue
                src = self.C[m["cand"]]
            else:
                src = self.B[bid]
            box = src.get("bbox_m")
            if not box:
                return None
            lo = box[0] if lo is None else min(lo, box[0])
            hi = box[3] if hi is None else max(hi, box[3])
        return None if lo is None else (lo, hi)

    # -- c1 ------------------------------------------------------------
    def c1_width(self):
        target = POLICY["width_delta_m"]
        rows = []
        deltas = []
        for role in ("sticks", "triggers", "bumpers"):
            bids = self.Broles.get(role, [])
            if len(bids) != 2:
                continue
            seed = abs(self.B[bids[0]]["centroid_m"][0]
                       - self.B[bids[1]]["centroid_m"][0])
            cs = [self.match[b]["cand"] for b in bids]
            if None in cs:
                continue
            got = abs(self.C[cs[0]]["centroid_m"][0]
                      - self.C[cs[1]]["centroid_m"][0])
            rows.append({"role": role, "seed_m": seed, "got_m": got,
                         "delta_m": got - seed})
            deltas.append(got - seed)
        # continuous: 1.0 when every pair grew by exactly 15 mm,
        # 0.0 when the growth is ~0 mm or >= 30 mm off.
        if deltas:
            pair_scores = [score_error(abs(d - target),
                                       perfect=0.002, zero=0.012)
                           for d in deltas]
            score = sum(pair_scores) / len(pair_scores)
        else:
            score = 0.0
        ok = score >= 0.999
        return {"status": PASS if ok else FAIL,
                "score": round(score, 4),
                "target_delta_m": target, "pairs": rows,
                "evidence": "widen graded as growth of mirror-pair "
                            "separation (sticks/triggers/bumpers), robust "
                            "to housing remodel; global bbox is NOT used"}

    # -- c2 ------------------------------------------------------------
    def c2_spacing(self):
        fails, det = [], {}
        # The candidate's own widening (not the policy target) defines where
        # a correctly re-spaced cluster should land.  Using the target here
        # would make an unwidened-but-correctly-swapped part lose c2 as well
        # as c1 -- the two components must stay independent.
        half = self.actual_half
        rigid_scores = []
        for role in RIGID_ROLES:
            bids = self.Broles.get(role, [])
            for i in range(len(bids)):
                for j in range(i + 1, len(bids)):
                    a, b = bids[i], bids[j]
                    ca = self.match[a]["cand"]
                    cb = self.match[b]["cand"]
                    if not ca or not cb:
                        continue
                    seed = d3(self.B[a]["centroid_m"], self.B[b]["centroid_m"])
                    got = d3(self.C[ca]["centroid_m"], self.C[cb]["centroid_m"])
                    rigid_scores.append(
                        score_error(abs(got - seed), perfect=5e-4,
                                    zero=2e-3))
                    if abs(got - seed) > TOL["intra_cluster_m"]:
                        fails.append(f"intra {role}: {seed*1000:.1f} -> "
                                     f"{got*1000:.1f} mm (diamond must move "
                                     "as a rigid unit)")
        pos_scores = []
        for role in POSITION_ROLES:
            bids = self.Broles.get(role, [])
            if not bids:
                continue
            cs = [self.match[i]["cand"] for i in bids]
            if None in cs:
                continue
            seed_c = sum(self.B[i]["centroid_m"][0] for i in bids) / len(bids)
            got_c = sum(self.C[c]["centroid_m"][0] for c in cs) / len(cs)
            if abs(seed_c - self.P) < TOL["straddle_m"]:
                # A cluster centred on the plane (every shoulder pair) maps
                # onto the plane however it is mirrored, so its centroid cannot
                # witness a position -- the per-body checks below are what
                # still say something about such a pair.  Crucially the mirror
                # must be applied to the CLUSTER centroid, not averaged over
                # per-body mirrors: the D-pad diamond straddles the plane
                # (buttons at +/-14.76 around x=0), and averaging mirrored
                # bodies cancels back to the plane, which is how a cluster that
                # never moved managed to look correctly placed.
                det[role] = {"note": "cluster centred on the plane -- no "
                                     "centroid witness",
                             "got_centroid_x_mm": round(got_c * 1000, 2)}
                continue
            exp_c = mirror_widen_x(seed_c, self.P, 0.0)
            # "Re-space the clusters to match the wider stance" can be met by
            # mirroring alone or by mirroring and spreading, so the target is a
            # BAND: the exact re-spacing amount is graded by c1 (shell pairs)
            # and by the rigid intra-cluster checks above, while this check
            # owns the much coarser question of whether the cluster moved to
            # the correct side at all (a cluster left in place misses by
            # 2x its offset, ~160 mm).
            reach = self.actual_half + 2e-3
            err = max(0.0, abs(got_c - exp_c) - reach)
            det[role] = {"seed_centroid_x_mm": round(seed_c * 1000, 2),
                         "expected_x_mm": round(exp_c * 1000, 2),
                         "allowed_band_mm": round(reach * 1000, 2),
                         "got_x_mm": round(got_c * 1000, 2),
                         "error_mm": round(err * 1000, 2)}
            pos_scores.append(score_error(err, perfect=3e-3, zero=0.015))
            if err > TOL["cluster_pos_m"]:
                fails.append(f"{role} centre x: expected {exp_c*1000:.1f} "
                             f"(+{reach*1000:.1f} of re-spacing), got "
                             f"{got_c*1000:.1f} mm")
        # Per-BODY placement for the mirror pairs (sticks/triggers/bumpers).
        # The role-mean check above is identically the plane for a symmetric
        # pair, so it says nothing about an individual body: seed L1's expected
        # position is the far side, and an un-swapped pair happens to leave a
        # twin sitting exactly there, so a swap stays invisible (the two bodies
        # are mirror twins -- geometrically indistinguishable).  What a
        # per-body check DOES see is a pair that was not re-spaced in step with
        # the candidate's own widening (one shoulder moved, one not).
        consistency_scores = []
        for role in ("sticks", "triggers", "bumpers"):
            bids = self.Broles.get(role, [])
            if len(bids) != 2:
                continue
            for bid in bids:
                cid = self.match[bid]["cand"]
                if not cid:
                    continue
                want = mirror_widen_x(self.B[bid]["centroid_m"][0], self.P,
                                      half)
                got = self.C[cid]["centroid_m"][0]
                err = abs(got - want)
                consistency_scores.append(score_error(err, perfect=4e-3,
                                                      zero=0.015))
                if err > TOL["cluster_pos_m"]:
                    fails.append(f"{role} body (slot {bid}) x: expected "
                                 f"{want*1000:.1f}, got {got*1000:.1f} mm -- "
                                 "not re-spaced with the rest of the shell")
        # Weighted so the CLUSTER POSITIONS dominate: leaving a diamond where it
        # was must cost most of this component, however rigid its internals are.
        # (`pos_scores` is one entry per named cluster, `consistency_scores`
        # one per shoulder body -- the symmetry of a pair makes those nearly
        # un-witnessable, so they stay a light integrity term.)
        pos_mean = sum(pos_scores) / len(pos_scores) if pos_scores else 0.0
        rigid_mean = (sum(rigid_scores) / len(rigid_scores)
                      if rigid_scores else 0.0)
        cons_mean = (sum(consistency_scores) / len(consistency_scores)
                     if consistency_scores else 1.0)
        score = clamp01(0.60 * pos_mean + 0.25 * rigid_mean + 0.15 * cons_mean)
        return {"status": PASS if score >= 0.999 else FAIL,
                "score": round(score, 4),
                "position_score": round(pos_mean, 4),
                "rigidity_score": round(rigid_mean, 4),
                "pair_consistency_score": round(cons_mean, 4),
                "widening_used_mm": round(half * 2000, 2),
                "failures": fails, "detail": det,
                "evidence": "cluster centres must land at the candidate's "
                            "own mirrored positions (+ its own re-spacing); "
                            "button diamonds must stay internally rigid; a "
                            "pair body must be re-spaced with the rest of the "
                            "shell.  Unverified positions score zero rather "
                            "than full credit"}

    # -- c3 ------------------------------------------------------------
    def c3_interference(self):
        seed_v = self.bl["interference"]["total_volume_m3"]
        got_v = self.ms["interference"]["total_volume_m3"]
        limit = seed_v + TOL["intf_growth_m3"]
        excess = max(0.0, got_v - limit)
        score = score_error(excess, perfect=0.0, zero=1e-7)
        return {"status": PASS if score >= 0.999 else FAIL,
                "score": round(score, 4),
                "seed_total_m3": seed_v, "measured_total_m3": got_v,
                "limit_m3": limit,
                "measured_pairs": len(self.ms["interference"]["pairs"]),
                "evidence": "boolean-intersect volume among control bodies "
                            "and control-vs-housing.  Hardware the candidate "
                            "adds (screws in bosses) is out of scope -- the "
                            "reference seats screws by design"}

    # -- c4 ------------------------------------------------------------
    def c4_handedness(self):
        fails, checks = [], {}
        # Two tiers: PRIMARY evidence that the layout really is left-handed
        # (cluster sides + body inertia signs), and GUARD evidence that the
        # housing was updated rather than mirrored wholesale (port-light
        # cluster side + shell moment sign).  They combine multiplicatively:
        # a part that swapped every control but flipped the housing is not
        # left-handed, however good its clusters look.
        primary, guards = [], []

        sides = {}
        for role in MIRROR_ROLES:
            bids = self.Broles.get(role, [])
            if not bids:
                continue
            cs = [self.match[i]["cand"] for i in bids]
            if None in cs:
                continue
            seed_c = sum(self.B[i]["centroid_m"][0] for i in bids) / len(bids)
            got_c = sum(self.C[c]["centroid_m"][0] for c in cs) / len(cs)
            straddle = abs(seed_c - self.P) < TOL["straddle_m"]
            entry = {"seed_x_mm": round(seed_c * 1000, 1),
                     "got_x_mm": round(got_c * 1000, 1),
                     "on_plane": straddle}
            if not straddle:
                flipped = sgn(seed_c - self.P) * sgn(got_c - self.P) < 0
                entry["ok"] = flipped
                # continuous: a cluster that visibly stayed put scores 0 for
                # every mm it failed to travel.
                travel = abs(got_c - seed_c)
                travel_target = 2 * abs(seed_c - self.P) + self.actual_half
                primary.append(score_error(abs(travel - travel_target),
                                           perfect=0.004, zero=0.030))
                if not flipped:
                    fails.append(f"{role} did not change sides "
                                 f"({seed_c*1000:.0f} -> {got_c*1000:.0f} mm)")
            else:
                # The cluster centroid is on the plane, so it can never witness
                # a side -- but the individual BODIES can, and some of these
                # clusters are exactly the ones the prompt names: the D-pad
                # diamond straddles the plane (buttons at +/-14.76 around x=0)
                # and the START/SELECT pair is symmetric about it.  Graded body
                # by body as a BAND (mirror-only ... mirror+spread), so the
                # documented freedom to re-space or not is not punished while a
                # cluster that simply stayed where it was still loses.
                per_body = []
                for b, c in zip(bids, cs):
                    sx = self.B[b]["centroid_m"][0]
                    gx = self.C[c]["centroid_m"][0]
                    if abs(sx - self.P) < TOL["straddle_m"]:
                        continue          # this body cannot witness a side
                    want = mirror_widen_x(sx, self.P, 0.0)
                    err = max(0.0, abs(gx - want) - self.actual_half)
                    sc = score_error(err, perfect=6e-3, zero=0.020)
                    per_body.append(sc)
                    if sgn(sx - self.P) * sgn(gx - self.P) >= 0:
                        fails.append(f"{role} body (slot {b}) did not change "
                                     f"sides ({sx*1000:.0f} -> "
                                     f"{gx*1000:.0f} mm)")
                entry["per_body_scores"] = [round(s, 3) for s in per_body]
                entry["ok"] = all(s >= 0.999 for s in per_body)
                # nothing appended when no body in the role can witness a side:
                # a check that cannot be made must not pay out full credit.
                primary.extend(per_body)
            sides[role] = entry
        checks["cluster_sides"] = sides

        chir = {}
        for role in CHIRAL_ROLES:
            for bid in self.Broles.get(role, []):
                m = self.match[bid]
                if not m["cand"]:
                    continue
                sb = self.B[bid]
                ch = chirality(sb)
                if ch < TOL["chiral_floor"]:
                    chir[bid] = {"role": role, "status": UNVERIFIABLE,
                                 "chirality": ch,
                                 "note": "near mirror-symmetric; a mirror "
                                         "cannot be read from its inertia"}
                    continue
                key = chirality_witness(sb)
                s = sb["inertia_com"][key]
                c = self.C[m["cand"]]["inertia_com"][key]
                flipped = (s * c) < 0
                # Continuous: the sign IS the witness (a mirror negates the
                # product of inertia), so any genuine flip earns full credit --
                # the reference re-cuts the D-pad engraving and its |Ixy| is not
                # comparable to the seed's.  A body that KEPT its sign tapers to
                # zero as its moment approaches the seed's, and a moment that
                # collapsed to nothing (any |c| < |s|) sits mid-scale instead of
                # paying out full credit for a witness that is no longer there.
                ratio = (c / s) if s else 0.0
                sc = 1.0 if flipped else clamp01(0.5 * (1.0 - clamp01(ratio)))
                chir[bid] = {"role": role,
                             "status": PASS if flipped else FAIL,
                             "witness": key, "seed": s, "measured": c,
                             "ratio": round(ratio, 4), "score": round(sc, 4)}
                primary.append(sc)
                if not flipped:
                    fails.append(f"{role} body (slot {bid}) is NOT mirrored "
                                 f"-- {key} kept its sign (ratio {ratio:.3f})")
        checks["body_chirality"] = chir

        seed_l = self.bl.get("light_cluster")
        cand_l = self.ms.get("light_cluster")
        entry = {}
        if not seed_l:
            entry = {"status": UNVERIFIABLE,
                     "note": "no light cluster recorded in the baseline -- "
                             "no one-sided housing witness exists, so this "
                             "guard earns nothing"}
            guards.append(0.0)
        elif not cand_l:
            # The seed's one-sided glyph faces have no counterpart anywhere on
            # the candidate housing: mirrored, re-scaled or erased, the
            # "the shell was updated, not flipped" claim cannot be witnessed.
            # It must not pay out full credit just because the measurement
            # came back empty.
            entry = {"status": FAIL, "score": 0.0,
                     "note": "the seed's one-sided glyph face areas are gone "
                             "from the whole candidate housing -- the housing "
                             "was mirrored wholesale, re-scaled or its "
                             "one-sided features were erased"}
            guards.append(0.0)
            fails.append("one-sided housing glyph cluster is unaccounted for "
                         "-- the shell cannot be shown to be updated rather "
                         "than mirrored")
        else:
            seed_x = seed_l["centroid_m"][0]
            want = widen_x(seed_x, self.P, self.actual_half)
            got = cand_l["centroid_m"][0]
            err = abs(got - want)
            sc = score_error(err, perfect=TOL["onesided_x_m"],
                             zero=TOL["onesided_x_zero_m"])
            same = seed_l["side"] == cand_l["side"]
            if not same:
                sc = 0.0
            entry = {"status": PASS if (same and sc >= 0.999) else FAIL,
                     "score": round(sc, 4),
                     "seed_side": seed_l["side"],
                     "candidate_side": cand_l["side"],
                     "seed_x_mm": round(seed_x * 1000, 1),
                     "expected_x_mm": round(want * 1000, 1),
                     "candidate_x_mm": round(got * 1000, 1),
                     "matched_faces": cand_l["n_faces"],
                     "located_by": cand_l.get("method", "area_multiset"),
                     "note": "port-light glyphs are wireless-port "
                             "indicators, not a handed control -- they must "
                             "stay on their original side of the plane, moved "
                             "only by the widening"}
            guards.append(sc)
            if not same:
                fails.append("port-light glyph cluster changed sides -- the "
                             "housing was mirrored wholesale, a naive "
                             "geometric flip")
            elif sc < 0.999:
                fails.append("one-sided housing glyphs are not where the "
                             "widening should have put them "
                             "(mirrored or re-scaled shell)")
        checks["port_lights_side"] = entry

        bs = self.bl.get("housing_signature", {})
        cs_sig = self.ms.get("housing_signature", {})
        bm, cm = bs.get("moment3_m5", 0.0), cs_sig.get("moment3_m5", 0.0)
        b_house = self.Broles.get("housing", [])
        c_house = self.Croles.get("housing", [])
        remodeled = True
        if len(b_house) == len(c_house) == 1:
            remodeled = fp_distance(self.B[b_house[0]],
                                    self.C[c_house[0]]) > 0.5
        ratio = (cm / bm) if bm else 0.0
        entry2 = {"baseline_moment3": bm, "measured_moment3": cm,
                  "ratio": round(ratio, 4),
                  "housing_remodeled": remodeled}
        if remodeled:
            entry2["status"] = UNVERIFIABLE
            entry2["note"] = ("housing was re-shelled/remodeled -- interior "
                              "faces dominate the moment, sign is not a "
                              "mirror witness (port_lights_side still is)")
            # Deliberately neither scored nor added to `guards`: the reference
            # DOES remodel the shell, so this witness is simply not available
            # for it, and an unavailable witness must not inflate the mean.
            # The one-sided guard above always is available.
        elif abs(bm) <= 0:
            entry2["status"] = UNVERIFIABLE
            entry2["note"] = ("no asymmetry recorded in the baseline -- no "
                              "witness exists, so this guard earns nothing")
            guards.append(0.0)
        elif abs(cm) < TOL["sig_floor_frac"] * abs(bm) and ratio >= 0:
            # Same-shell fingerprint, but its one-sided asymmetry has collapsed
            # toward zero: the features that make it one-sided were erased.
            # Scored by the same taper below rather than skipped -- a collapsed
            # witness is not a pass.
            entry2["status"] = FAIL
            entry2["score"] = round(clamp01(0.5 + ratio), 4)
            entry2["note"] = ("housing kept its shape but lost its one-sided "
                              "asymmetry -- the mirror witness has been erased")
            guards.append(entry2["score"])
            fails.append("housing one-sided feature moment collapsed "
                         f"({bm:.3e} -> {cm:.3e}): the shell's asymmetry is "
                         "gone")
        else:
            # Sign is the witness (a wholesale mirror negates the asymmetry);
            # anything at or above a fifth of the seed's magnitude with the
            # same sign is a full pass, exactly as before, so a legitimately
            # different-but-valid rebuild of the shell is not punished.  Below
            # that the credit tapers -- a collapsed asymmetry sits mid-scale
            # rather than paying out full credit for a witness that is gone.
            same_side = ratio >= TOL["sig_floor_frac"]
            sc = 1.0 if same_side else (clamp01(0.5 + ratio) if ratio >= 0
                                        else 0.0)
            entry2["status"] = PASS if same_side and sc >= 0.999 else FAIL
            entry2["score"] = round(sc, 4)
            guards.append(sc)
            if ratio < 0:
                fails.append("housing one-sided feature moment changed sign "
                             "-- the shell was mirrored wholesale")
            elif not same_side:
                fails.append(f"housing one-sided feature moment collapsed "
                             f"(ratio {ratio:.3f}) -- the shell's asymmetry "
                             "was erased")
        checks["housing_side_signature"] = entry2

        checks["symbol_glyphs"] = {
            "status": UNVERIFIABLE,
            "note": "PS triangle/circle/cross/square are left-right "
                    "symmetric split-faces; a mirror is geometrically "
                    "undetectable on the symbols themselves.  Handedness is "
                    "graded from cluster sides, body inertia signs and the "
                    "housing side signature instead.",
        }

        primary_score = sum(primary) / len(primary) if primary else 0.0
        # Guards are independent ways of asking the SAME question ("was the
        # housing updated rather than flipped?"), so the weakest one governs:
        # a candidate that satisfies only the one it can see does not get to
        # average its way past the one it fails.
        guard_score = min(guards) if guards else 0.0
        score = primary_score * guard_score
        return {"status": PASS if score >= 0.999 else FAIL,
                "score": round(score, 4),
                "primary_score": round(primary_score, 4),
                "guard_score": round(guard_score, 4),
                "failures": fails, "checks": checks,
                "n_scored_checks": len(primary) + len(guards),
                "evidence": "An empty measurement never earns full credit: "
                            "nothing matched on either side scores 0 here. "
                            "primary = cluster sides (body-by-body for a "
                            "cluster whose centroid sits on the plane) plus "
                            "chiral body inertia signs; guards = the one-sided "
                            "housing glyphs stay put and the shell's asymmetry "
                            "is not negated.  Primary and guards multiply."}

    # -- c5 ------------------------------------------------------------
    def c5_unrequested(self):
        fails, det = [], {}
        span_scores, body_scores = [], []
        sb = self.bl["global"]["bbox_m"]
        mb = self.ms["global"]["bbox_m"]
        for ax, nm in ((1, "Y"), (2, "Z")):
            want = sb[ax + 3] - sb[ax]
            got = mb[ax + 3] - mb[ax]
            det[nm] = {"seed_mm": round(want * 1000, 1),
                       "got_mm": round(got * 1000, 1)}
            span_scores.append(score_error(abs(got - want),
                                           perfect=TOL["span_m"], zero=0.020))
            if abs(got - want) > TOL["span_m"]:
                fails.append(f"global {nm} span changed {want*1000:.1f} -> "
                             f"{got*1000:.1f} mm -- only X growth was "
                             "requested")
        # X: the only X growth the prompt asks for is the widening, but in THIS
        # part the X extremes belong to the housing (135.5 mm off the plane, vs
        # 88 mm for the outermost control) and the housing is the one body the
        # reference legitimately re-shells -- so its tips drift with the remodel
        # on top of the widening.  Measured live: solution's span grew 21.9 mm
        # against a 15.0 mm growth of the mirror pairs (its own STL sidecar
        # shows 14.96 mm), i.e. 6.9 mm of shell-remodel slack, not a stretch.
        # So: measured over the MATCHED bodies (a stray body cannot inflate it)
        # and charged only above the same per-side slack the Y/Z spans get.
        # A global X scale is still caught elsewhere: the pairs (c1) and the
        # rigid intra-cluster spacing (c2) both scale with it, and Y/Z here do.
        seed_ext = self._matched_extent_x(False)
        cand_ext = self._matched_extent_x(True)
        if seed_ext and cand_ext:
            want_x = seed_ext[1] - seed_ext[0]
            got_x = cand_ext[1] - cand_ext[0]
            basis = "matched bodies"
        else:
            want_x = sb[3] - sb[0]
            got_x = mb[3] - mb[0]
            basis = "document union (no per-body boxes available)"
        grow = got_x - want_x
        expect = 2.0 * self.actual_half
        slack = 2.0 * TOL["span_m"]
        det["X"] = {"seed_mm": round(want_x * 1000, 1),
                    "got_mm": round(got_x * 1000, 1),
                    "growth_mm": round(grow * 1000, 2),
                    "expected_growth_mm": round(expect * 1000, 2),
                    "remodel_slack_mm": round(slack * 1000, 2),
                    "basis": basis}
        xerr = max(0.0, grow - expect - slack)
        span_scores.append(score_error(xerr, perfect=TOL["xspan_m"],
                                       zero=TOL["xspan_zero_m"]))
        if xerr > TOL["xspan_m"]:
            fails.append(f"global X span grew {grow*1000:.1f} mm against the "
                         f"candidate's own widening of {expect*1000:.1f} mm "
                         f"plus {slack*1000:.0f} mm of remodel slack -- an "
                         "unrequested X stretch somewhere on the part")
        exempt = set()
        for role in RESHAPE_EXEMPT:
            exempt.update(self.Broles.get(role, []))
        reshaped = {}
        for bid, m in self.match.items():
            if bid in exempt or not m["cand"]:
                continue
            d = fp_distance(self.B[bid], self.C[m["cand"]])
            body_scores.append(score_error(d, perfect=TOL["fp_reshape"],
                                           zero=0.30))
            if d > TOL["fp_reshape"]:
                reshaped[bid] = {"role": m["role"], "distance": d}
                fails.append(f"{m['role']} body reshaped (fingerprint drift "
                             f"{d:.2e})")
        det["reshaped"] = reshaped
        # Bodies that fall into no role at all (in this part the two 2 mm
        # hardware slivers c12/c13 sit inside the shell and match none of the
        # role windows) are graded nowhere else, yet they are exactly the kind
        # of place an unrequested change hides.  Compared by fingerprint only
        # (position free), and only while both sides have a handful of such
        # bodies -- a candidate whose role assignment itself came out unusual
        # is judged by the components above, not here.
        seed_free = [b for b in self.bl.get("bodies", [])
                     if b["id"] not in self.match]
        cand_free = [b for b in self.ms.get("bodies", [])
                     if b["id"] not in self._candidate_role_ids()]
        det["unroled_seed_bodies"] = [b["id"] for b in seed_free]
        det["unroled_candidate_bodies"] = [b["id"] for b in cand_free]
        if len(seed_free) <= 3 and len(cand_free) <= 3:
            for sb in seed_free:
                near = [c for c in cand_free
                        if abs(c["volume_m3"] - sb["volume_m3"])
                        <= 0.2 * sb["volume_m3"]]
                if not near:
                    body_scores.append(0.0)
                    fails.append(f"body (slot {sb['id']}) has no counterpart "
                                 "in the candidate at all")
                    continue
                c = min(near, key=lambda x: abs(x["volume_m3"]
                                                - sb["volume_m3"]))
                d = fp_distance(sb, c)
                body_scores.append(score_error(d, perfect=TOL["fp_reshape"],
                                               zero=0.30))
                det.setdefault("unroled_drift", {})[sb["id"]] = {
                    "role": "unroled", "distance": round(d, 5)}
                if d > TOL["fp_reshape"]:
                    fails.append(f"unroled body (slot {sb['id']}) was reshaped "
                                 f"(fingerprint drift {d:.2e})")
        # Two halves: the GLOBAL spans (strong, whole-part evidence) and the
        # per-body fingerprints (guards).  Averaged separately so that a change
        # big enough to move a global span cannot be diluted by the many
        # bodies that were left alone.
        span_mean = (sum(span_scores) / len(span_scores)
                     if span_scores else 0.0)
        body_mean = (sum(body_scores) / len(body_scores)
                     if body_scores else 0.0)
        score = clamp01(0.5 * span_mean + 0.5 * body_mean)
        return {"status": PASS if score >= 0.999 else FAIL,
                "score": round(score, 4),
                "span_score": round(span_mean, 4),
                "body_score": round(body_mean, 4),
                "failures": fails, "detail": det,
                "caveat": "housing, button diamonds and centre buttons are "
                          "exempt from the reshape check -- the reference "
                          "remodels them.  Sticks/triggers/bumpers and any "
                          "body outside every role must keep their shape.  "
                          "An empty measurement scores 0, never full credit"}

    # -- c6 ------------------------------------------------------------
    def c6_glyphs(self):
        score, det, fails = self._glyph_presence()
        return {"status": PASS if score >= 0.999 else FAIL,
                "score": round(score, 4),
                "failures": fails, "detail": det,
                "evidence": "small engraved glyph faces (d-pad arrows, "
                            "trigger/centre engravings, port-light cluster) "
                            "must survive the remodel; graded per matched "
                            "body on engraving presence (a rebuilt glyph may "
                            "split into a different number of faces), half "
                            "mean and half worst case so a stripped body "
                            "cannot hide among the bodies never at risk"}

    def _glyph_presence(self):
        """Continuous survival of small engraved glyph faces (d-pad arrows,
        trigger/centre engravings, port-light cluster) per matched body.

        Scored as engraving PRESENCE rather than an exact face count: the
        reference rebuilds the engravings when it swaps the clusters, which
        changes how the glyph splits into faces (11 -> 5 on the D-pad) without
        removing it, so full credit starts at a quarter of the seed's count per
        body and tapers to zero as the glyph is erased.

        Two aggregates, because neither alone is enough:
          * mean  -- every engraving body counts the same, so a candidate that
            habitually under-cuts glyphs everywhere is caught;
          * worst -- a body stripped to nothing must not be diluted by the
            housing's ports/vents (276 of the seed's small faces), so the
            component cannot be rescued by the bodies that were never at risk.
        A candidate that loses an entire glyph-bearing body scores 0 for it,
        and a candidate whose glyph-bearing bodies cannot be matched at all
        scores 0 for the component rather than full credit.
        """
        def survival(got, want):
            if want <= 0:
                return 1.0
            return clamp01(got / max(1.0, TOL["glyph_floor"] * want))

        seed_smf = self.bl.get("small_face_counts", {})
        cand_smf = self.ms.get("small_face_counts", {})
        per_body, fails, det = [], [], {}
        total_got, total_want = 0, 0
        for bid, m in self.match.items():
            want = seed_smf.get(bid, 0)
            if want <= 0:
                continue
            cid = m.get("cand")
            got = cand_smf.get(cid, 0) if cid else 0
            score = survival(got, want) if cid else 0.0
            per_body.append(score)
            total_got += got
            total_want += want
            det[bid] = {"role": m["role"], "seed": want, "got": got,
                        "score": round(score, 3),
                        "matched": bool(cid)}
            if score < 1.0:
                fails.append(f"{m['role']} body (slot {bid}) "
                             + ("is missing from the candidate"
                                if not cid else "lost its small engraved "
                                f"faces: {want} -> {got}"))
        seed_l = self.bl.get("light_cluster")
        cand_l = self.ms.get("light_cluster")
        if seed_l:
            want = seed_l.get("n_faces", 0)
            got = cand_l.get("n_faces", 0) if cand_l else 0
            score = survival(got, want)
            per_body.append(score)
            total_got += got
            total_want += want
            det["light_cluster"] = {"seed": want, "got": got,
                                    "score": round(score, 3)}
            if score < 1.0:
                fails.append(f"port-light glyph cluster lost faces: "
                             f"{want} -> {got}")
        if not per_body:
            fails.append("no glyph-bearing body could be matched at all")
            return 0.0, det, fails
        mean = sum(per_body) / len(per_body)
        worst = min(per_body)
        score = clamp01(0.5 * mean + 0.5 * worst)
        det["_aggregate"] = {"engraving_bodies": len(per_body),
                             "mean_score": round(mean, 3),
                             "worst_body": round(worst, 3),
                             "glyph_faces_seed": total_want,
                             "glyph_faces_candidate": total_got,
                             "raw_ratio": round(total_got / total_want, 3)
                             if total_want else None}
        return score, det, fails

    # -- assemble --------------------------------------------------------
    def grade(self):
        report = {"document": self.ms.get("document"),
                  "rebuild": self.ms.get("rebuild", {}),
                  "policy": POLICY, "criteria": {}, "notes": []}

        rb = self.ms.get("rebuild", {})
        if not rb.get("ok", False):
            for k in ("widened by 15 mm", "clusters at mirrored positions",
                      "no new control interference",
                      "left-handed layout achieved", "no unrequested changes",
                      "glyphs preserved"):
                report["criteria"][k] = {
                    "status": FAIL, "score": 0.0,
                    "evidence": "not evaluated -- health gate failed"}
            report["overall"] = FAIL
            report["notes"].append(
                "Health gate failed: the feature tree does not rebuild "
                f"clean vs the seed census (errors={rb.get('errors')}).")
            return report

        report["criteria"]["widened by 15 mm"] = self.c1_width()
        report["criteria"]["clusters at mirrored positions"] = self.c2_spacing()
        report["criteria"]["no new control interference"] = self.c3_interference()
        report["criteria"]["left-handed layout achieved"] = self.c4_handedness()
        report["criteria"]["no unrequested changes"] = self.c5_unrequested()
        report["criteria"]["glyphs preserved"] = self.c6_glyphs()
        st = [v["status"] for v in report["criteria"].values()]
        report["overall"] = FAIL if FAIL in st else PASS
        return report


def summarise(report):
    out = [f"document : {report.get('document')}",
           f"rebuild  : ok={report.get('rebuild', {}).get('ok')} "
           f"errors={report.get('rebuild', {}).get('errors')}", ""]
    for k, v in report["criteria"].items():
        sc = v.get("score")
        tag = f"{v['status']:12} score={sc}" if sc is not None else v["status"]
        out.append(f"  {k:32} {tag}")
        for f in v.get("failures", []):
            out.append(f"      - {f}")
        for name, entry in (v.get("checks") or {}).items():
            if isinstance(entry, dict) and entry.get("status") == UNVERIFIABLE:
                out.append(f"      ! {name}: UNVERIFIABLE")
    out += ["", f"  OVERALL          {report['overall']}"]
    return "\n".join(out)


# --------------------------------------------------------------------------
# entry points
# --------------------------------------------------------------------------

def _app():
    # Forces dynamic/late-bound dispatch (see common/solidworks_session.attach
    # docstring) -- plain win32com.client.GetActiveObject silently upgrades to
    # early-bound dispatch once pywin32 has cached makepy support for the
    # SolidWorks typelib (which it does automatically, persistently, on the
    # very first successful attach on a machine), and open_or_active()'s
    # manual VARIANT(VT_BYREF, ...) OpenDoc6 call then breaks with
    # "TypeError: int() argument must be ... not 'VARIANT'". Confirmed on the
    # grading box 2026-08-15.
    import pythoncom
    try:
        pythoncom.CoInitialize()
    except Exception:
        pass  # already initialized
    from common import solidworks_session as sws
    return sws.attach()


def grade_candidate(path=None, close_after=False):
    baseline = json.loads(BASELINE_PATH.read_text(encoding="utf-8"))
    t0 = time.perf_counter()
    app = _app()
    doc = open_or_active(app, path)
    print("  [harness] document ready in %.1fs (%s)"
          % (time.perf_counter() - t0, path or "active document"),
          file=sys.stderr, flush=True)
    measured = capture(doc, baseline=baseline,
                       plane_x=baseline["plane_x_m"])
    t1 = time.perf_counter()
    print("  [harness] grading", file=sys.stderr, flush=True)
    report = Grader(baseline, measured).grade()
    print(summarise(report), file=sys.stderr)
    print("  [harness] total %.1fs (open+capture %.1fs, grade %.1fs)"
          % (time.perf_counter() - t0, t1 - t0, time.perf_counter() - t1),
          file=sys.stderr, flush=True)
    if close_after and path:
        try:
            app.CloseDoc(measured["document"])
        except Exception:
            pass
    return report


class PS3Harness(Harness):
    CANDIDATE_OPTIONAL = True  # without an arg, grades the live document

    # Continuous component weights; all four values below use exact binary
    # fractions so the sum is exactly 4.0 (task.toml's max_score) with no
    # floating-point residue.  The three headline asks (widen, re-space,
    # left-hand) carry full weight; the "don't break anything" guards carry
    # less, but each still has a dedicated adversarial that must lose it.
    WEIGHTS = {
        "widened by 15 mm": 1.0,
        "clusters at mirrored positions": 1.0,
        "left-handed layout achieved": 1.0,
        "no new control interference": 0.5,
        "no unrequested changes": 0.25,
        "glyphs preserved": 0.25,
    }

    def build_state(self, candidate_path):
        return candidate_path

    def checks(self, candidate_path):
        report = grade_candidate(candidate_path, close_after=bool(candidate_path))
        registry = {}
        for cname, crit in report["criteria"].items():
            score = crit.get("score")
            # continuous score when the criterion provides one; fall back to
            # the boolean verdict for a criterion that predates scoring.
            registry[cname] = (score if score is not None
                               else (1.0 if crit.get("status") == PASS else 0.0))
        return registry


main = PS3Harness.as_main()

if __name__ == "__main__":
    PS3Harness.cli()

