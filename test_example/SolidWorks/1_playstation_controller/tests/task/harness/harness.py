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
"""

from __future__ import annotations

import itertools
import json
import math
import os
import sys
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


def find_light_cluster_candidate(faces, plane_x, seed_cluster):
    """Candidate-side: match the seed's area multiset among housing faces."""
    if not seed_cluster:
        return None
    hits = []
    for want in seed_cluster["areas_m2"]:
        best, bd = None, None
        for a, c in faces:
            d = abs(a - want) / max(want, 1e-30)
            if d < LIGHT_AREA_TOL_FRAC and (bd is None or d < bd):
                best, bd = (a, c), d
        if best:
            hits.append(best)
    if len(hits) < LIGHT_MIN_MATCHES:
        return None
    best = None
    for a0, c0 in hits:
        grp = [(a, c) for a, c in hits
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


def capture(doc, baseline=None, plane_x=None):
    rebuild = SC.health_gate(doc, baseline)
    raw, bodies, boxes, gmin, gmax = SC.capture_bodies(doc)
    smf = _small_face_counts(raw)
    roles = assign_roles(bodies, smf)
    P = plane_x
    if P is None:
        P = sym_plane(bodies)
    if P is None and baseline:
        P = baseline.get("plane_x_m")

    hfaces = housing_faces(raw, roles.get("housing", []))
    if P is not None:
        sig = housing_signature(hfaces, P)
        seed_cluster = (baseline or {}).get("light_cluster")
        if seed_cluster:
            lights = find_light_cluster_candidate(hfaces, P, seed_cluster)
        else:
            lights = find_light_cluster_seed(hfaces, P, gmin + gmax)
    else:
        sig = {"moment3_m5": 0.0, "sign": 0, "faces": 0}
        lights = None

    intf = control_interference(raw, bodies, roles)
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

    # -- c1 ------------------------------------------------------------
    def c1_width(self):
        target = POLICY["width_delta_m"]
        rows = []
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
        deltas = [r["delta_m"] for r in rows]
        ok = bool(deltas) and all(abs(d - target) <= TOL["width_m"]
                                  for d in deltas)
        return {"status": PASS if ok else FAIL,
                "target_delta_m": target, "pairs": rows,
                "evidence": "widen graded as growth of mirror-pair "
                            "separation (sticks/triggers/bumpers), robust "
                            "to housing remodel; global bbox is NOT used"}

    # -- c2 ------------------------------------------------------------
    def c2_spacing(self):
        fails, det = [], {}
        half = POLICY["half_m"]
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
                    if abs(got - seed) > TOL["intra_cluster_m"]:
                        fails.append(f"intra {role}: {seed*1000:.1f} -> "
                                     f"{got*1000:.1f} mm (diamond must move "
                                     "as a rigid unit)")
        for role in POSITION_ROLES:
            bids = self.Broles.get(role, [])
            if not bids:
                continue
            exp_c = sum(mirror_widen_x(self.B[i]["centroid_m"][0], self.P,
                                       half) for i in bids) / len(bids)
            cs = [self.match[i]["cand"] for i in bids]
            if None in cs:
                continue
            got_c = sum(self.C[c]["centroid_m"][0] for c in cs) / len(cs)
            det[role] = {"expected_x_mm": round(exp_c * 1000, 2),
                         "got_x_mm": round(got_c * 1000, 2)}
            if abs(got_c - exp_c) > TOL["cluster_pos_m"]:
                fails.append(f"{role} centre x: expected {exp_c*1000:.1f}, "
                             f"got {got_c*1000:.1f} mm")
        return {"status": PASS if not fails else FAIL,
                "failures": fails, "detail": det,
                "evidence": "cluster centres must land at the mirrored + "
                            "widened positions; button diamonds must stay "
                            "internally rigid"}

    # -- c3 ------------------------------------------------------------
    def c3_interference(self):
        seed_v = self.bl["interference"]["total_volume_m3"]
        got_v = self.ms["interference"]["total_volume_m3"]
        limit = seed_v + TOL["intf_growth_m3"]
        ok = got_v <= limit
        return {"status": PASS if ok else FAIL,
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

        sides = {}
        for role in MIRROR_ROLES:
            bids = self.Broles.get(role, [])
            if not bids:
                continue
            seed_c = sum(self.B[i]["centroid_m"][0] for i in bids) / len(bids)
            cs = [self.match[i]["cand"] for i in bids]
            if None in cs:
                continue
            got_c = sum(self.C[c]["centroid_m"][0] for c in cs) / len(cs)
            straddle = abs(seed_c - self.P) < TOL["straddle_m"]
            flipped = straddle or (sgn(seed_c - self.P)
                                   * sgn(got_c - self.P) < 0)
            sides[role] = {"seed_x_mm": round(seed_c * 1000, 1),
                           "got_x_mm": round(got_c * 1000, 1),
                           "ok": flipped, "on_plane": straddle}
            if not flipped:
                fails.append(f"{role} did not change sides "
                             f"({seed_c*1000:.0f} -> {got_c*1000:.0f} mm)")
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
                chir[bid] = {"role": role,
                             "status": PASS if flipped else FAIL,
                             "witness": key, "seed": s, "measured": c}
                if not flipped:
                    fails.append(f"{role} body (slot {bid}) is NOT mirrored "
                                 f"-- {key} kept its sign")
        checks["body_chirality"] = chir

        seed_l = self.bl.get("light_cluster")
        cand_l = self.ms.get("light_cluster")
        entry = {}
        if not seed_l:
            entry = {"status": UNVERIFIABLE,
                     "note": "no light cluster recorded in the baseline"}
        elif not cand_l:
            entry = {"status": UNVERIFIABLE,
                     "note": "port-light glyph faces not found on the "
                             "candidate (housing re-glyphed?) -- side "
                             "cannot be witnessed"}
        else:
            same = seed_l["side"] == cand_l["side"]
            entry = {"status": PASS if same else FAIL,
                     "seed_side": seed_l["side"],
                     "candidate_side": cand_l["side"],
                     "candidate_x_mm": round(cand_l["centroid_m"][0] * 1000, 1),
                     "matched_faces": cand_l["n_faces"],
                     "note": "port-light glyphs are wireless-port "
                             "indicators, not a handed control -- they must "
                             "stay on their original side of the plane"}
            if not same:
                fails.append("port-light glyph cluster changed sides -- the "
                             "housing was mirrored wholesale, a naive "
                             "geometric flip")
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
        entry2 = {"baseline_moment3": bm, "measured_moment3": cm,
                  "housing_remodeled": remodeled}
        if remodeled:
            entry2["status"] = UNVERIFIABLE
            entry2["note"] = ("housing was re-shelled/remodeled -- interior "
                              "faces dominate the moment, sign is not a "
                              "mirror witness (port_lights_side still is)")
        elif abs(bm) <= 0 or abs(cm) < TOL["sig_floor_frac"] * abs(bm):
            entry2["status"] = UNVERIFIABLE
            entry2["note"] = "housing asymmetry signal too weak"
        else:
            same_side = (bm * cm) > 0
            entry2["status"] = PASS if same_side else FAIL
            if not same_side:
                fails.append("housing one-sided feature moment changed sign "
                             "-- the shell was mirrored wholesale")
        checks["housing_side_signature"] = entry2

        checks["symbol_glyphs"] = {
            "status": UNVERIFIABLE,
            "note": "PS triangle/circle/cross/square are left-right "
                    "symmetric split-faces; a mirror is geometrically "
                    "undetectable on the symbols themselves.  Handedness is "
                    "graded from cluster sides, body inertia signs and the "
                    "housing side signature instead.",
        }

        return {"status": PASS if not fails else FAIL,
                "failures": fails, "checks": checks}

    # -- c5 ------------------------------------------------------------
    def c5_unrequested(self):
        fails, det = [], {}
        sb = self.bl["global"]["bbox_m"]
        mb = self.ms["global"]["bbox_m"]
        for ax, nm in ((1, "Y"), (2, "Z")):
            want = sb[ax + 3] - sb[ax]
            got = mb[ax + 3] - mb[ax]
            det[nm] = {"seed_mm": round(want * 1000, 1),
                       "got_mm": round(got * 1000, 1)}
            if abs(got - want) > TOL["span_m"]:
                fails.append(f"global {nm} span changed {want*1000:.1f} -> "
                             f"{got*1000:.1f} mm -- only X growth was "
                             "requested")
        exempt = set()
        for role in RESHAPE_EXEMPT:
            exempt.update(self.Broles.get(role, []))
        reshaped = {}
        for bid, m in self.match.items():
            if bid in exempt or not m["cand"]:
                continue
            d = fp_distance(self.B[bid], self.C[m["cand"]])
            if d > TOL["fp_reshape"]:
                reshaped[bid] = {"role": m["role"], "distance": d}
                fails.append(f"{m['role']} body reshaped (fingerprint drift "
                             f"{d:.2e})")
        det["reshaped"] = reshaped
        return {"status": PASS if not fails else FAIL,
                "failures": fails, "detail": det,
                "caveat": "housing, button diamonds and centre buttons are "
                          "exempt from the reshape check -- the reference "
                          "remodels them.  Sticks/triggers/bumpers must "
                          "keep their shape"}

    # -- assemble --------------------------------------------------------
    def grade(self):
        report = {"document": self.ms.get("document"),
                  "rebuild": self.ms.get("rebuild", {}),
                  "policy": POLICY, "criteria": {}, "notes": []}

        rb = self.ms.get("rebuild", {})
        if not rb.get("ok", False):
            for k in ("widened by 15 mm", "clusters at mirrored positions",
                      "no new control interference",
                      "left-handed layout achieved", "no unrequested changes"):
                report["criteria"][k] = {
                    "status": FAIL,
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
        st = [v["status"] for v in report["criteria"].values()]
        report["overall"] = FAIL if FAIL in st else PASS
        return report


def summarise(report):
    out = [f"document : {report.get('document')}",
           f"rebuild  : ok={report.get('rebuild', {}).get('ok')} "
           f"errors={report.get('rebuild', {}).get('errors')}", ""]
    for k, v in report["criteria"].items():
        out.append(f"  {k:30} {v['status']}")
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
    from common import solidworks_session as sws
    return sws.attach()


def grade_candidate(path=None, close_after=False):
    baseline = json.loads(BASELINE_PATH.read_text(encoding="utf-8"))
    app = _app()
    doc = open_or_active(app, path)
    measured = capture(doc, baseline=baseline,
                       plane_x=baseline["plane_x_m"])
    report = Grader(baseline, measured).grade()
    print(summarise(report), file=sys.stderr)
    if close_after and path:
        try:
            app.CloseDoc(measured["document"])
        except Exception:
            pass
    return report


class PS3Harness(Harness):
    MUST_PASS = ("no unrequested changes",)
    CANDIDATE_OPTIONAL = True  # without an arg, grades the live document

    def build_state(self, candidate_path):
        return candidate_path

    def checks(self, candidate_path):
        report = grade_candidate(candidate_path, close_after=bool(candidate_path))
        registry = {}
        for cname, crit in report["criteria"].items():
            registry[cname] = (crit.get("status") == PASS)
        return registry


main = PS3Harness.as_main()

if __name__ == "__main__":
    PS3Harness.cli()

