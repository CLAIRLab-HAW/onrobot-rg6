# Changelog — onrobot-rg6

Was sich wann geändert hat. Der aktuelle Stand steht in der [README](README.md);
die Einbettung in den Onboard-Stack in
[docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md).

## 2026-08-19

- **Der SRDF-Patch ist halb so gross und nicht mehr schaedlich.** Er trug die
  Linknamen des ALTEN Greifermodells hart ein (`left_inner_knuckle`,
  `right_outer_knuckle`, …) und haette sie nach jedem Boot in ein korrektes
  SRDF zurueckgeschrieben; sein Idempotenz-Check merkt das nicht, weil er nur
  prueft, *ob* sein Block dasteht, nicht *ob dessen Links existieren*.
  Gemessen und daraufhin entfernt: die **`disable_collisions`-Haelfte ist
  ueberfluessig geworden** — das frisch erzeugte SRDF enthaelt bereits **82**
  Greiferpaare, die der `moveit_collision_updater` aus dem neuen URDF selbst
  ableitet. Sie war ein Behelf fuer das selbstgebaute Modell.
  **Geblieben** sind Planungsgruppe, `group_states` open/close und
  `end_effector` (mit den neuen Gelenknamen und den Ankern 0,0 / 1,25478) sowie
  der `moveit.yaml`-Teil: GripperCommand-Controller, `max_effort` und
  `joint_limits`. Damit kann MoveIt den Greifer weiter oeffnen, schliessen,
  auf eine Weite fahren — und die Kraft kommt entweder aus `max_effort` oder
  je Ziel aus dem `effort`-Feld des Bahnpunkts (`gripper_command_controller_handle.hpp`
  Zeile 143 gegen 147).
  Gegenprobe am Container: das SRDF nennt **keinen** Link und **kein** Gelenk,
  das es im URDF nicht gibt (vorher sieben).


- **Das vermessene RG6-Modell ist einvendort** (`onrobot_description`, rg6_v2,
  MIT, INRIA — Herkunft in [LICENSE-THIRD-PARTY.md](LICENSE-THIRD-PARTY.md)):
  sieben visuelle Meshes plus eigene Kollisionsmeshes, Ursprünge und Grenzen
  als Daten in `config/rg6_v2.yaml` statt fest im Xacro. Der Treibergelenkname
  ist parametrisiert, damit `rg6_finger_joint` bleibt, was es ist. Gegenüber
  Upstream entfernt: der `<ros2_control>`-Block und der Gazebo-Aufruf — der
  Greifer hängt an diesem Roboter nicht am `controller_manager`, sondern an
  der XML-RPC-Brücke. Verdrahtet ist es noch nicht; das Modell im Betrieb ist
  weiterhin das alte.
- **Die Bracket-Geometrie des neuen Modells stimmt an der Hardware:** der
  Owner hat 81 mm Bracket-Höhe und ~41 mm Einstecktiefe des Greiferkörpers
  gemessen, das Mesh sagt 81,1 und 41,5 mm.
- **Ein Irrtum desselben Tages, am selben Tag zurückgenommen:** zwischenzeitlich
  stand `rg6_tool0_to_base` auf `xyz="0 0 0.0196"` — 19,6 mm für eine
  Quick-Changer-Roboterseite, die ROBOTER-TODO R20 aus dem Datenblatt
  erschlossen hatte. Am Gerät nachgesehen sitzt das Bracket **unmittelbar auf
  dem UR-Flansch**; sein Fuss ist selbst die Kopplungsfläche (Ø 71 mm, an
  beiden Modellen am Mesh nachgemessen). Der Wert steht wieder auf `0 0 0`.

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
