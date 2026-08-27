#!/usr/bin/env python3
"""Derives the joint angle ─▶ gripping width table from a GENERATED URDF.

Why from the URDF and not from a formula:  the fingers of the rg6_v2 are a four-bar chain (moment_arm ─▶ truss_arm ─▶
finger_tip ─▶ flex_finger), all three tied to the same angle via ``mimic``.  There is no closed formula for it, and an
approximation placed alongside would be exactly the second version on which the old model and its driver have already
drifted apart once (R19).

Why a TABLE and not an import:  the gripper bridge runs on the robot and is not supposed to import anything there that
does not belong to the robot.  A table is data, not a package -- it costs no dependency, and an outdated table is a
visible file with a date instead of a silent deviation.

    xacro robot.urdf.xacro > /tmp/robot.urdf
    python3 derive_finger_kinematics.py /tmp/robot.urdf \\
        > <husky-custom-setup>/scripts/rg6_finger_kinematics.json
    python3 derive_finger_kinematics.py /tmp/robot.urdf --format cpp \\
        > src/rg6_control/include/rg6_control/finger_kinematics.hpp

Two output formats, ONE source.  The table is needed in two places that cannot import each other: the gripper bridge on
the robot reads it as JSON, ``rg6_control_sim`` in the container needs it in C++.  Maintaining it twice by hand is
exactly the second version the paragraph above warns about -- which is why this script produces both.
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np

#: Step size of the sampling points.  0,05 rad keeps the interpolation error
#: below 0,05 mm; that is half the finger position resolution of the RG6.
STEP_RAD = 0.05
DRIVER = "rg6_finger_joint"


def _rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y))
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _kette(urdf):
    root = ET.parse(urdf).getroot()
    K, limit = {}, None
    for j in root.findall("joint"):
        par, ch, o = j.find("parent"), j.find("child"), j.find("origin")
        if j.get("name") == DRIVER and j.find("limit") is not None:
            # Both ends, not just the upper one.  ``lower`` is not always 0: the mechanical open stop of the RG6 does
            # not reach the geometric zero of the chain (measured 2026-08-27, see rg6_v2.yaml), and a table that
            # starts at 0 anyway would hand out an opening the hardware cannot deliver -- while the URDF, which does
            # read ``lower``, refuses it.  Two truths about the same joint.
            limit = (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
        if par is None or ch is None:
            continue
        ax = j.find("axis")
        K[ch.get("link")] = dict(
            parent=par.get("link"),
            typ=j.get("type"),
            xyz=np.array([float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]),
            rpy=np.array([float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]),
            axis=np.array([float(v) for v in ((ax.get("xyz") if ax is not None else None) or "0 0 0").split()]),
        )
    if limit is None:
        raise SystemExit(f"{DRIVER} has no joint limit in the URDF")
    return K, limit


def _pose(K, goal, q):
    T, kette, link = np.eye(4), [], goal
    while link in K:
        kette.append(K[link])
        link = K[link]["parent"]
    for j in reversed(kette):
        A = np.eye(4)
        A[:3, :3] = _rpy_to_R(*j["rpy"])
        A[:3, 3] = j["xyz"]
        if j["typ"] == "revolute":
            k = j["axis"] / np.linalg.norm(j["axis"])
            S = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            B = np.eye(4)
            B[:3, :3] = np.eye(3) + np.sin(q) * S + (1 - np.cos(q)) * S @ S
            A = A @ B
        T = T @ A
    return T


HPP_HEAD = """\
// GENERATED, NOT MAINTAINED -- tools/derive_finger_kinematics.py from the
// generated URDF of the gripper model.  Regenerate after every change to the
// model; numbers edited by hand drift away from the robot in silence.
//
// Joint angle {joint} [rad] -> clear width between the pad faces [m], measured
// between the two flex_finger meshes.  The same table lies as JSON next to the
// gripper bridge on the robot
// (husky-custom-setup/scripts/rg6_finger_kinematics.json) and in the robot
// profile (robot_contract, gripper.linkage.table).
//
// There is no closed formula: the fingers are a four-bar chain.
//
// Lower bound {qmin} rad = the mechanical open stop; the chain computes a wider
// opening below it that the hardware does not reach.
// Upper bound {qmax} rad = the zero crossing of the width; beyond it the
// fingers drive through one another in the model.
// Max. interpolation error against a 400-point grid: {error_mm} mm.

#ifndef RG6_CONTROL__FINGER_KINEMATICS_HPP_
#define RG6_CONTROL__FINGER_KINEMATICS_HPP_

#include <algorithm>
#include <array>
#include <cstddef>

namespace rg6_control::finger_kinematics
{{

inline constexpr double kQMinRad = {qmin};
inline constexpr double kQMaxRad = {qmax};

//: Sampling points (q [rad], width [m]), strictly monotonically falling in the width.
inline constexpr std::array<std::array<double, 2>, {n}> kTable = {{{{
{lines}
}}}};

inline constexpr double kMaxWidthM = kTable.front()[1];
inline constexpr double kMinWidthM = kTable.back()[1];

//: Clear width [m] at finger joint ``q`` [rad], linearly interpolated.
inline double width_from_angle(double q)
{{
  const double x = std::clamp(q, kQMinRad, kQMaxRad);
  for (std::size_t i = 1; i < kTable.size(); ++i) {{
    if (x <= kTable[i][0]) {{
      const double q0 = kTable[i - 1][0], w0 = kTable[i - 1][1];
      const double q1 = kTable[i][0], w1 = kTable[i][1];
      const double t = (q1 - q0) > 0.0 ? (x - q0) / (q1 - q0) : 0.0;
      return w0 + t * (w1 - w0);
    }}
  }}
  return kMinWidthM;
}}

//: Fingergelenk [rad] fuer die lichte Weite ``width_m``.  Die Weite wird auf
//: den darstellbaren Bereich GEKLEMMT -- ein Befehl ueber den Hub hinaus ist
//: gueltig und bedeutet "so weit wie es geht", nicht NaN.
inline double angle_from_width(double width_m)
{{
  const double w = std::clamp(width_m, kMinWidthM, kMaxWidthM);
  for (std::size_t i = 1; i < kTable.size(); ++i) {{
    if (w >= kTable[i][1]) {{
      const double q0 = kTable[i - 1][0], w0 = kTable[i - 1][1];
      const double q1 = kTable[i][0], w1 = kTable[i][1];
      const double t = (w0 - w1) > 0.0 ? (w0 - w) / (w0 - w1) : 0.0;
      return q0 + t * (q1 - q0);
    }}
  }}
  return kQMaxRad;
}}

}}  // namespace rg6_control::finger_kinematics

#endif  // RG6_CONTROL__FINGER_KINEMATICS_HPP_
"""


def _as_hpp(tab, qmin, qmax, error) -> str:
    lines = ",\n".join(f"  {{{{{q:.5f}, {w:.6f}}}}}" for q, w in tab)
    return HPP_HEAD.format(
        joint=DRIVER,
        qmin=f"{qmin:.5f}",
        qmax=f"{qmax:.5f}",
        n=len(tab),
        lines=lines,
        error_mm=f"{error * 1000:.3f}",
    )


def main(urdf: str, fmt: str = "json") -> int:
    K, (qmin, qmax) = _kette(urdf)
    faces = sorted(n for n in K if n.endswith("flex_finger"))
    if len(faces) != 2:
        raise SystemExit(f"erwarte zwei flex_finger-Greifflaechen, gefunden: {faces}")

    def width(q: float) -> float:
        a = _pose(K, faces[0], q)[:3, 3]
        b = _pose(K, faces[1], q)[:3, 3]
        return float(np.linalg.norm(a - b))

    qs = list(np.arange(qmin, qmax, STEP_RAD)) + [qmax]
    tab = [[round(float(q), 5), round(width(q), 6)] for q in qs]

    fine = np.linspace(qmin, qmax, 400)
    error = np.abs(np.array([width(q) for q in fine]) - np.interp(fine, [t[0] for t in tab], [t[1] for t in tab])).max()

    if fmt == "cpp":
        sys.stdout.write(_as_hpp(tab, qmin, qmax, float(error)))
        return 0

    json.dump(
        {
            "kommentar": [
                "Joint angle rg6_finger_joint [rad] -> clear width between the pad",
                "faces [m], measured between the two flex_finger meshes.",
                "GENERATED, not maintained: tools/derive_finger_kinematics.py from the",
                "generated URDF.  Regenerate after every change to the gripper model.",
                "The lower bound is the mechanical open stop; below it the chain",
                "computes a wider opening than the hardware reaches.",
                "The upper bound is the zero crossing of the width; beyond it the",
                "fingers drive through one another in the model and the width grows again.",
            ],
            "joint": DRIVER,
            "joint_limits_rad": [round(qmin, 5), round(qmax, 5)],
            "max_interpolationsfehler_m": round(float(error), 6),
            "table_q_rad_width_m": tab,
        },
        sys.stdout,
        indent=1,
    )
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    _p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    _p.add_argument("urdf", help="generiertes URDF mit dem Greifermodell")
    _p.add_argument(
        "--format",
        choices=("json", "cpp"),
        default="json",
        help="json = Tabelle fuer die Greiferbruecke (Vorgabe), " "cpp = Header fuer rg6_control_sim",
    )
    _a = _p.parse_args()
    raise SystemExit(main(_a.urdf, _a.format))
