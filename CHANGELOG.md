# Changelog — onrobot-rg6

Was sich wann geändert hat. Der aktuelle Stand steht in der [README](README.md);
die Einbettung in den Onboard-Stack in
[docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md).

## 2026-08-19

- **Der Quick Changer sitzt wieder zwischen Flansch und Greifer.**
  `rg6_tool0_to_base` stand auf `xyz="0 0 0"` — der ganze Greifer hing damit
  19,6 mm zu hoch am Arm, und mit ihm sein TCP. Der Wert ist die Roboterseite
  des Quick Changers für I/O (QC-Datenblatt v2.0); die Werkzeugseite steckt
  schon im Mesh (unterste 20 mm von `base_link.stl`, Ø 71 mm). Der Owner hat
  nachgesehen: zwischen Flansch und Greifer liegt nichts ausser einem
  0,4-mm-Ring, der zum UR-Flansch gehört. Durch `xacro` verifiziert:
  `arm_0_tool0 → rg6_hand_tcp` ist jetzt 229,6 mm statt 210,0.
  **Zweitbeleg aus fremder Quelle:** im MIT-lizenzierten Modell
  `onrobot_description` (rg6_v2, INRIA) reicht `body.stl` 19 mm unter seinen
  eigenen Ursprung mit Ø 69,5 mm — dieselbe Scheibe, derselbe Bezugspunkt.
  **Achtung:** `hrl` greift damit 19,6 mm zu hoch, bis der kalibrierte
  `descend_offset` (0,065 gegen 0,045 in der Sim — ein Aufschlag von 20,0 mm,
  der genau diesen Fehler bezahlt hat) zurückgenommen ist. Siehe ROBOTER-TODO
  **R23**; die Rücknahme braucht einen Griff am Gerät.

## 2026-08-13

- Die Trigger-Aliasse `rg6_control/open_gripper` und
  `rg6_control/close_gripper` entfallen — in `rg6_control` **und** im
  Simulations-Zwilling `rg6_control_sim`. Sie waren seit
  „Simplify service call names to open and close" reine Rückwärtskompatibilität
  zum damals ausgerollten Stand und hingen an derselben `handle_open_close`;
  zwei Namen für dieselbe Tat sind an der Schnittstelle nur eine Frage mehr, die
  man beim Debuggen beantworten muss. Kanonisch sind `rg6_control/open` und
  `rg6_control/close`.
  **Kein Aufrufer im Workspace war betroffen** — Profil
  (`contract/.../a200_0553.yaml`), SDK (`husky_sdk.ros.client`, dessen
  Python-Methoden `open_gripper()`/`close_gripper()` unverändert bleiben und
  schon immer `…/open` bzw. `…/close` riefen), `scripts/record.sh` und der
  Container-Mock rufen ausschliesslich die kanonischen Namen. Angepasst wurden
  nur Beschreibungen: README, `rg6_bringup.launch.py`-Docstring,
  `docs/ARCHITECTURE.md`.
  **Auf dem a200-0553 läuft bis zum nächsten Bau + Deploy von `rg6_control`
  weiterhin der Stand mit Aliassen** (Aufgabe A9 in `docs/gesamtplan.md`); der
  Systemsnapshot vom 2026-07-31 listet sie deshalb weiterhin — er ist das
  Protokoll jenes Tages und wurde nicht nachgezogen.
