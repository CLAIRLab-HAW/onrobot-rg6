#!/usr/bin/env python3
"""Leitet die Tabelle Gelenkwinkel -> Greifweite aus einem GENERIERTEN URDF ab.

Warum aus dem URDF und nicht aus einer Formel:  die Finger des rg6_v2 sind eine
Viergelenkkette (moment_arm -> truss_arm -> finger_tip -> flex_finger), alle
drei per ``mimic`` am selben Winkel.  Es gibt keine geschlossene Formel dafuer,
und eine danebengestellte Naeherung waere genau die Zweitfassung, an der das
alte Modell und sein Treiber schon einmal auseinandergelaufen sind (R19).

Warum eine TABELLE und kein Import:  die Greiferbruecke laeuft auf dem Roboter
und soll dort nichts importieren muessen, was nicht zum Roboter gehoert.  Eine
Tabelle ist Daten, kein Paket -- sie kostet keine Abhaengigkeit, und eine
veraltete Tabelle ist eine sichtbare Datei mit Datum statt einer stillen
Abweichung.

    xacro robot.urdf.xacro > /tmp/robot.urdf
    python3 derive_finger_kinematics.py /tmp/robot.urdf > scripts/rg6_finger_kinematics.json
"""
from __future__ import annotations

import json
import sys
import xml.etree.ElementTree as ET

import numpy as np

#: Schrittweite der Stuetzstellen.  0,05 rad haelt den Interpolationsfehler
#: unter 0,05 mm; das ist die halbe Fingerpositionsaufloesung des RG6.
SCHRITT_RAD = 0.05
TREIBER = "rg6_finger_joint"


def _rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _kette(urdf):
    root = ET.parse(urdf).getroot()
    K, grenze = {}, None
    for j in root.findall("joint"):
        par, ch, o = j.find("parent"), j.find("child"), j.find("origin")
        if j.get("name") == TREIBER and j.find("limit") is not None:
            grenze = float(j.find("limit").get("upper"))
        if par is None or ch is None:
            continue
        ax = j.find("axis")
        K[ch.get("link")] = dict(
            parent=par.get("link"), typ=j.get("type"),
            xyz=np.array([float(v) for v in
                          ((o.get("xyz") if o is not None else None) or "0 0 0").split()]),
            rpy=np.array([float(v) for v in
                          ((o.get("rpy") if o is not None else None) or "0 0 0").split()]),
            axis=np.array([float(v) for v in
                           ((ax.get("xyz") if ax is not None else None) or "0 0 0").split()]))
    if grenze is None:
        raise SystemExit(f"{TREIBER} hat im URDF keine obere Gelenkgrenze")
    return K, grenze


def _pose(K, ziel, q):
    T, kette, link = np.eye(4), [], ziel
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


def main(urdf: str) -> int:
    K, qmax = _kette(urdf)
    flaechen = sorted(n for n in K if n.endswith("flex_finger"))
    if len(flaechen) != 2:
        raise SystemExit(f"erwarte zwei flex_finger-Greifflaechen, gefunden: {flaechen}")

    def weite(q: float) -> float:
        a = _pose(K, flaechen[0], q)[:3, 3]
        b = _pose(K, flaechen[1], q)[:3, 3]
        return float(np.linalg.norm(a - b))

    qs = list(np.arange(0.0, qmax, SCHRITT_RAD)) + [qmax]
    tab = [[round(float(q), 5), round(weite(q), 6)] for q in qs]

    fein = np.linspace(0.0, qmax, 400)
    fehler = np.abs(np.array([weite(q) for q in fein])
                    - np.interp(fein, [t[0] for t in tab], [t[1] for t in tab])).max()

    json.dump({
        "kommentar": [
            "Gelenkwinkel rg6_finger_joint [rad] -> lichte Weite zwischen den",
            "Padflaechen [m], gemessen zwischen den beiden flex_finger-Meshes.",
            "ERZEUGT, nicht gepflegt: tools/derive_finger_kinematics.py aus dem",
            "generierten URDF.  Nach jeder Aenderung am Greifermodell neu erzeugen.",
            "Die Obergrenze ist der Nulldurchgang der Weite; darueber fahren die",
            "Finger im Modell durcheinander hindurch und die Weite waechst wieder.",
        ],
        "joint": TREIBER,
        "joint_limits_rad": [0.0, round(qmax, 5)],
        "max_interpolationsfehler_m": round(float(fehler), 6),
        "table_q_rad_width_m": tab,
    }, sys.stdout, indent=1)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1]))
