#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Post-processing for FAST-LIO ground-truth recordings (VLN experiments).

Runs on a PC or on the robot; needs only numpy (matplotlib optional, for --plot).

Input: CSV files written by FAST-LIO `stop_path_record` (recording v2):
    # ... metadata lines starting with '#'
    x,y,z,qx,qy,qz,qw,stamp_sec,stamp_nanosec

Commands
--------
compare : metrics of one test trajectory against one reference (human) trajectory
    NE      : endpoint navigation error [m], horizontal distance test-end -> goal
    success : NE <= --success-radius (default 1.0 m, VLN-CE convention)
    PR      : path ratio, length(test)/length(reference), horizontal
    HD      : symmetric Hausdorff distance between the two polylines [m]
summary : run `compare` over many ref,test pairs and report SR / means
interp  : sample a trajectory at given control-command timestamps
          (e.g. the NAPO-VLN decision log) -> one pose per command

Examples
--------
python3 gt_postprocess.py compare --ref human_lab01.csv --test vln_lab01.csv \
        --goal-from-ref --plot lab01.png
python3 gt_postprocess.py summary --success-radius 1.0 \
        --pairs human_lab01.csv,vln_lab01.csv human_lab02.csv,vln_lab02.csv
python3 gt_postprocess.py interp --traj human_lab01.csv --times cmds_lab01.csv \
        --out lab01_at_cmds.csv
"""

import argparse
import csv
import json
import math
import sys

import numpy as np


# ---------------------------------------------------------------- trajectory IO

def load_traj(path):
    """Load a recording CSV -> dict of numpy arrays (plus metadata lines)."""
    meta = {}
    header, rows = None, []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                if ":" in line:
                    k, v = line[1:].split(":", 1)
                    meta[k.strip()] = v.strip()
                continue
            if header is None:
                header = [h.strip() for h in line.split(",")]
                continue
            rows.append([float(v) for v in line.split(",")])
    required = ["x", "y", "z", "qx", "qy", "qz", "qw", "stamp_sec", "stamp_nanosec"]
    missing = [n for n in required if n not in header]
    if missing:
        raise ValueError("%s: missing columns %s (header: %s)" % (path, missing, header))
    if not rows:
        raise ValueError("%s: no pose rows found" % path)
    data = np.asarray(rows, dtype=np.float64)
    col = {name: data[:, i] for i, name in enumerate(header)}
    t = col["stamp_sec"] + col["stamp_nanosec"] * 1e-9
    return {
        "path": path,
        "meta": meta,
        "xyz": np.stack([col["x"], col["y"], col["z"]], axis=1),
        "quat": np.stack([col["qw"], col["qx"], col["qy"], col["qz"]], axis=1),
        "t": t,
    }


def yaw_from_quat(q):
    """yaw (rad) from quaternion(s) [w, x, y, z], shape (..., 4)."""
    q = np.atleast_2d(q)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def path_length_2d(xyz):
    d = np.diff(xyz[:, :2], axis=0)
    return float(np.sum(np.hypot(d[:, 0], d[:, 1])))


def _min_dists(pa, pb):
    """For each point in pa, the distance to its nearest neighbor in pb (2D).
    Chunked so long trajectories (e.g. a full multi-episode session CSV) do not
    materialize an N x M distance matrix in memory."""
    out = np.empty(len(pa))
    if len(pb) == 0:
        raise ValueError("empty trajectory")
    chunk = max(1, int(2e6 // max(len(pb), 1)))  # ~32 MB per block (float64)
    for i in range(0, len(pa), chunk):
        d = pa[i:i + chunk, None, :2] - pb[None, :, :2]
        out[i:i + chunk] = np.sqrt((d * d).sum(-1).min(axis=1))
    return out


def hausdorff_2d(a, b):
    """Symmetric Hausdorff distance between two polylines (horizontal plane)."""
    return float(max(_min_dists(a, b).max(), _min_dists(b, a).max()))


def mean_nn_2d(a, b):
    """Mean nearest-neighbor distance a->b (horizontal), a smooth HD complement."""
    return float(_min_dists(a, b).mean())


# ---------------------------------------------------------------- alignment

def align_origin_2d(test, ref):
    """Rigidly (SE(2)) map `test` so its FIRST pose (x, y, yaw) coincides with
    the FIRST pose of `ref`. Use for trajectories recorded in per-run camera_init
    frames (SLAM mode). With localization mode both are already in the map frame
    and alignment is optional (it only removes the small start-mark offset)."""
    tr = ref["xyz"][0, :2]
    tt = test["xyz"][0, :2]
    dyaw = yaw_from_quat(ref["quat"][0:1])[0] - yaw_from_quat(test["quat"][0:1])[0]
    c, s = math.cos(dyaw), math.sin(dyaw)
    R = np.array([[c, -s], [s, c]])
    out = dict(test)
    xy = test["xyz"][:, :2] - tt
    out["xyz"] = np.column_stack([xy @ R.T + tr, test["xyz"][:, 2]]).astype(np.float64)
    out["quat"] = test["quat"]  # yaw-only delta not folded back into quats
    out["_yaw_offset"] = dyaw
    return out


# ---------------------------------------------------------------- slerp / interp

def slerp(q0, q1, u):
    d = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    if d < 0.0:
        q1 = -q1
        d = -d
    if d > 0.9995:
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(d)
    return (math.sin((1 - u) * theta) * q0 + math.sin(u * theta) * q1) / math.sin(theta)


def interp_traj(traj, times):
    """Sample trajectory at `times` (s). Returns xyz, quat, in_range flags."""
    t, xyz, quat = traj["t"], traj["xyz"], traj["quat"]
    idx = np.clip(np.searchsorted(t, times) - 1, 0, len(t) - 2)
    out_xyz = np.zeros((len(times), 3))
    out_quat = np.zeros((len(times), 4))
    in_range = (times >= t[0]) & (times <= t[-1])
    for k, tk in enumerate(times):
        i = idx[k]
        span = t[i + 1] - t[i]
        u = 0.0 if span <= 0 else (tk - t[i]) / span
        u = float(np.clip(u, 0.0, 1.0))
        out_xyz[k] = xyz[i] * (1 - u) + xyz[i + 1] * u
        out_quat[k] = slerp(quat[i], quat[i + 1], u)
    return out_xyz, out_quat, in_range


# ---------------------------------------------------------------- metrics

def compare_pair(ref, test, success_radius=1.0, goal=None, align=False):
    if align:
        test = align_origin_2d(test, ref)
    if goal is None:
        goal = ref["xyz"][-1, :2]  # goal = end of the human reference trajectory
    goal = np.asarray(goal, dtype=np.float64)[:2]
    ne = float(np.linalg.norm(test["xyz"][-1, :2] - goal))
    res = {
        "ref": ref["path"],
        "test": test["path"],
        "ref_source": ref["meta"].get("control_source", ""),
        "test_source": test["meta"].get("control_source", ""),
        "test_instruction_id": test["meta"].get("instruction_id", ""),
        "goal": [float(goal[0]), float(goal[1])],
        "NE_m": ne,
        "success": bool(ne <= success_radius),
        "PR": path_length_2d(test["xyz"]) / max(path_length_2d(ref["xyz"]), 1e-9),
        "HD_m": hausdorff_2d(test["xyz"], ref["xyz"]),
        "mean_NN_m": mean_nn_2d(test["xyz"], ref["xyz"]),
        "n_ref": len(ref["t"]),
        "n_test": len(test["t"]),
        "duration_ref_s": float(ref["t"][-1] - ref["t"][0]),
        "duration_test_s": float(test["t"][-1] - test["t"][0]),
    }
    return res


def print_compare(res):
    print("ref                  : %s (%s, %d poses)" % (res["ref"], res["ref_source"] or "?", res["n_ref"]))
    print("test                 : %s (%s, %d poses)" % (res["test"], res["test_source"] or "?", res["n_test"]))
    print("NE   [m]             : %.3f   (goal %s)" % (res["NE_m"], ["%.2f" % g for g in res["goal"]]))
    print("success (NE<=%.2f m) : %s" % (res.get("radius", 1.0), "YES" if res["success"] else "NO"))
    print("PR   (len_t/len_r)   : %.2f" % res["PR"])
    print("HD   [m]             : %.3f" % res["HD_m"])
    print("mean NN [m]          : %.3f" % res["mean_NN_m"])


# ---------------------------------------------------------------- commands

def cmd_compare(a):
    ref = load_traj(a.ref)
    test = load_traj(a.test)
    goal = None
    if a.goal is not None:
        goal = a.goal[:2]
    res = compare_pair(ref, test, success_radius=a.success_radius, goal=goal, align=a.align_origin)
    res["radius"] = a.success_radius
    print_compare(res)
    if a.json:
        print(json.dumps(res))
    if a.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available, skipping plot")
            return
        if a.align_origin:
            test = align_origin_2d(test, ref)
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(ref["xyz"][:, 0], ref["xyz"][:, 1], "-o", color="tab:green", ms=2, lw=1.5, label="reference (%s)" % (res["ref_source"] or "ref"))
        ax.plot(test["xyz"][:, 0], test["xyz"][:, 1], "-o", color="tab:red", ms=2, lw=1.5, label="test (%s)" % (res["test_source"] or "test"))
        ax.plot(*ref["xyz"][0, :2], marker="s", color="k", ms=8, ls="", label="start")
        ax.plot(*res["goal"], marker="*", color="tab:blue", ms=14, ls="", label="goal")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title("NE=%.2f m  PR=%.2f  HD=%.2f m" % (res["NE_m"], res["PR"], res["HD_m"]))
        ax.legend()
        fig.savefig(a.plot, dpi=150, bbox_inches="tight")
        print("plot saved to %s" % a.plot)


def cmd_summary(a):
    rows = []
    for pair in a.pairs:
        ref_path, test_path = pair.split(",")
        ref = load_traj(ref_path)
        test = load_traj(test_path)
        rows.append(compare_pair(ref, test, success_radius=a.success_radius, align=a.align_origin))
    w = max(len(r["test"]) for r in rows)
    print("%-*s  %8s  %7s  %6s  %7s" % (w, "test", "NE[m]", "PR", "HD[m]", "success"))
    for r in rows:
        print("%-*s  %8.3f  %7.2f  %6.3f  %7s" % (w, r["test"], r["NE_m"], r["PR"], r["HD_m"], "YES" if r["success"] else "NO"))
    sr = 100.0 * sum(r["success"] for r in rows) / len(rows)
    ok = [r for r in rows if r["success"]]
    mean_ne = float(np.mean([r["NE_m"] for r in ok])) if ok else float("nan")
    mean_pr = float(np.mean([r["PR"] for r in ok])) if ok else float("nan")
    mean_hd = float(np.mean([r["HD_m"] for r in ok])) if ok else float("nan")
    print("-" * (w + 50))
    print("SR = %.1f%% (%d/%d), success-radius %.2f m" % (sr, len(ok), len(rows), a.success_radius))
    print("successful episodes: mean NE %.3f m, mean PR %.2f, mean HD %.3f m" % (mean_ne, mean_pr, mean_hd))
    if a.json:
        print(json.dumps({"SR": sr, "radius": a.success_radius, "rows": rows}))


def cmd_interp(a):
    traj = load_traj(a.traj)
    times, labels = [], []
    with open(a.times, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        header = [h.strip().lstrip("#").strip() for h in header]
        ti = header.index("t_sec") if "t_sec" in header else 0
        ci = header.index("cmd") if "cmd" in header else None
        for row in reader:
            if not row or not row[0].strip():
                continue
            times.append(float(row[ti]))
            labels.append(row[ci] if ci is not None and ci < len(row) else "")
    times = np.asarray(times)
    xyz, quat, in_range = interp_traj(traj, times)
    yaws = np.degrees(yaw_from_quat(quat))
    with open(a.out, "w") as f:
        f.write("# interpolated from: %s\n" % a.traj)
        f.write("# frame_id: %s\n" % traj["meta"].get("frame_id", "camera_init"))
        f.write("t_sec,cmd,x,y,z,yaw_deg,in_range\n")
        for k in range(len(times)):
            f.write("%.3f,%s,%.4f,%.4f,%.4f,%.2f,%d\n" %
                    (times[k], labels[k], xyz[k, 0], xyz[k, 1], xyz[k, 2], yaws[k], int(in_range[k])))
    print("wrote %d samples to %s (%d outside trajectory time range)"
          % (len(times), a.out, int((~in_range).sum())))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("compare", help="one reference vs one test trajectory")
    c.add_argument("--ref", required=True)
    c.add_argument("--test", required=True)
    c.add_argument("--goal", nargs=2, type=float, default=None, metavar=("GX", "GY"),
                   help="goal x y [m]; default: end of reference trajectory")
    c.add_argument("--success-radius", type=float, default=1.0)
    c.add_argument("--align-origin", action="store_true",
                   help="SE(2)-align test start pose to reference start pose (SLAM-mode recordings)")
    c.add_argument("--plot", default=None, help="save a trajectory figure to this path")
    c.add_argument("--json", action="store_true", help="also print one-line JSON")
    c.set_defaults(func=cmd_compare)

    s = sub.add_parser("summary", help="aggregate SR/NE/PR/HD over several pairs")
    s.add_argument("--pairs", nargs="+", required=True, metavar="ref.csv,test.csv")
    s.add_argument("--success-radius", type=float, default=1.0)
    s.add_argument("--align-origin", action="store_true")
    s.add_argument("--json", action="store_true")
    s.set_defaults(func=cmd_summary)

    i = sub.add_parser("interp", help="sample a trajectory at control-command timestamps")
    i.add_argument("--traj", required=True)
    i.add_argument("--times", required=True, help="CSV with header 't_sec' (optional 'cmd')")
    i.add_argument("--out", required=True)
    i.set_defaults(func=cmd_interp)

    a = p.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
