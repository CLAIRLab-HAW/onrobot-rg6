# Changelog — onrobot-rg6

Was sich wann geändert hat. Der aktuelle Stand steht in der [README](README.md);
die Einbettung in den Onboard-Stack in
[docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md).

Das Format folgt [Keep a Changelog](https://keepachangelog.com/de/1.1.0/),
die Versionierung [Semantic Versioning](https://semver.org/lang/de/).

## [Unreleased]

### Behoben
- **`rg6_moveit_patch` gibt das Paar `flex_finger` <-> `flex_finger` frei --
  ohne das war der Greifer in MoveIt nicht zu schliessen.** Die beiden
  Gummipads beruehren sich nur in den letzten 1,3 % des Hubs (gemessen am
  laufenden `move_group` per Bisektion: letzte gueltige Stellung 1,238486 rad
  = 3,01 mm lichte Weite, Kontaktband 0,016294 rad). Dort landet das Sampling
  des `moveit_collision_updater` praktisch nie, das Paar galt ihm als
  "manchmal in Kollision" und blieb aktiv.

  Folge: `move_group` verwies jede Stellung enger als 3 mm -- waehrend
  dieselbe SRDF einen `group_state` `close` bei 1,25478 rad (0,40 mm) anbot.
  Das Modell verbot seinen eigenen Named State, und nichts duenner als 3 mm
  war ueber MoveIt greifbar. Der Hub geht von 153,17 mm auf 0,40 mm.

  Freigegeben statt `close` entschaerft: die Selbstkollisionspruefung soll den
  Roboter vor sich selbst schuetzen, und ein leer zufahrender Greifer ist der
  Normalfall, den die Hardware bei jedem Griff ausfuehrt. Die zwei Pads SIND
  die Greifflaechen. Der echte Anschlag bleibt das Gelenklimit.

  `reason="User"`: weder `Adjacent` (im URDF keine Nachbarn) noch `Default`
  (in der Standardstellung 153 mm auseinander) trifft zu.

  Damit sind es drei gepatchte Paare statt zwei. Das dritte fehlte aus dem
  umgekehrten Grund wie die ersten beiden: die sind ueber den ganzen Hub in
  Kontakt, dieses fast nirgends.

## [0.2.0] - 2026-08-19

### README auf den Ist-Zustand
- **Die README beschrieb noch den geloeschten Tool-DO-Treiber.** Sie fuehrte
  neun `rg6_control/*`-Services, das Topic `rg6/state`, den
  `rg6_joint_state_broadcaster`, die AI2-Kennlinie samt Kalibrierverfahren, das
  17-Bit-URScript-Protokoll und `rg6_bringup.launch.py` als aktuellen Stand --
  und schrieb im ersten Absatz vor, die OnRobot-URCap muesse **aus** bleiben,
  waehrend genau sie der Steuerweg ist. Alle Beispielaufrufe zeigten auf
  Schnittstellen, die es nicht mehr gibt.

  Neu geschrieben auf das, was das Repo heute ist: Modell (`rg6_description`),
  MoveIt-Anbindung (`rg6_moveit_patch` + robot.yaml), joint_states-Verrohrung
  und Container-Mock (`rg6_control_sim`) -- mit dem ausdruecklichen Hinweis,
  dass der reale Greifer von `rg6_grip_bridge` in husky-custom-setup
  kommandiert wird. Die kanonische Schnittstelle (Action `gripper_cmd`,
  Zustand `rg6/bridge_state`, Gelenkwert in rad statt Weite) steht jetzt ganz
  oben, weil sie auf beiden Stufen dieselbe ist.
- **Historische Bezuege aus den Quellkommentaren entfernt.** Was einmal war,
  steht in dieser Datei; im Quelltext stand es doppelt. Betroffen:
  `rg6_control_sim.cpp` (der "AM 2026-08-19 ZURUECKGESCHNITTEN"-Block,
  dreimal "Bis 2026-08-19 stand hier ..."), `joint_states.launch.py`,
  `CMakeLists.txt`, `package.xml`, `rg6_moveit_patch`, `rg6_v2.yaml`.
  Die Warnungen selbst sind geblieben, nur ohne Datum und Vorgeschichte --
  etwa dass ein RobotState mit einem modellfremden Gelenknamen `move_group`
  ueber `getVariableIndex` zum Absturz bringt.
- **Ein toter Verweis dabei gefunden:** `rg6_moveit_patch` begruendete seine
  Endwinkel mit `rg6_control/analog_model.hpp` -- die Datei ist mit dem
  Analogmodell entfallen. Die Werte kommen aus `finger_kinematics.hpp`.
- Gemessene Zahlen behalten ihr Datum. Sie sagen, wie frisch der Wert ist.

### moveit.yaml zurueck an robot.yaml

- **`rg6_moveit_patch` patcht nur noch die SRDF und prueft den Rest.** Die
  zweite Haelfte des Tools -- GripperCommand-Controller
  `manipulators/rg6_gripper_controller` und
  `robot_description_planning.joint_limits.rg6_finger_joint` -- ist nach
  `robot.yaml` gewandert (`manipulators.moveit.ros_parameters.move_group`,
  husky-custom-setup). Nachgemessen im Container: der Clearpath-Generator
  schreibt aus diesem Block eine `moveit.yaml`, die mit der frueher gepatchten
  bis auf die **Reihenfolge** von `controller_names` identisch ist (als Menge
  gleich, Rest deckungsgleich); `robot.srdf` blieb byteweise identisch. Damit
  steht die Greifer-Anbindung in der versionierten SSOT statt in einem
  Python-Template, gilt fuer Roboter und Offboard-Container zugleich, und eine
  der beiden generierten Dateien verlaesst den Patch-Pfad.
- **Fuer die SRDF gibt es diesen Weg nicht -- am Quelltext belegt.**
  `clearpath_config` enthaelt das Wort `srdf` an **keiner** Stelle, es gibt
  kein Gegenstueck zu `platform.extras.urdf` (`ExtrasConfig` kennt nur `urdf`,
  `launch`, `ros_parameters`), `robot.srdf.xacro` entsteht ausschliesslich aus
  den drei Schleifen Arms/Grippers/Lifts, und `Gripper.MODEL` ist ein
  geschlossenes Enum (`franka_gripper`, `kinova_2f_lite`, `robotiq_2f_140`,
  `robotiq_2f_85`), das `rg6` mit `ValueError` ablehnt. Der einzige
  SRDF-Hebel in robot.yaml ist `poses` -> `group_state`, und der haengt an
  einem Manipulator-Objekt, das es fuer den RG6 nicht geben kann.
- **Neu: `verify_moveit_yaml` statt stiller Fahrt ohne Greifer.** Fehlen die
  Werte in der generierten `moveit.yaml` -- veraltete `robot.yaml` unter
  `/etc/clearpath` oder aus `ROBOT_YAML_URL` --, listet das Tool jeden
  fehlenden Schluessel auf, nennt die SSOT und endet mit **Exit-Code 1**.
  Ohne diese Pruefung faellt der Fall erst auf, wenn ein GripperCommand-Goal
  ins Leere laeuft. Kein Aufrufer bricht daran ab; Installer und
  offboard-Entrypoint loggen den Code. Entfallen sind die Argumente
  `--action-ns`, `--max-effort`, `--max-velocity`, `--max-acceleration` --
  ihre Werte stehen jetzt in robot.yaml.

### Altlasten

- **`rg6_control_sim` bildet nur noch die Oberflaeche der Bruecke nach.**
  Entfallen sind die Services `rg6_control/{open,close,grip,set_force_preset,
  set_tool_power}`, das Topic `rg6/state` (`rg6_msgs/GripperState`) und das
  AI2-Modell. Sie gehoerten zum geloeschten Tool-DO-Treiber; am Roboter gibt
  es sie nicht mehr, und seit `husky_sdk` auf die Action umgestellt ist, ruft
  sie niemand. Ein Mock, der eine Oberflaeche nachbildet, die es nirgends
  gibt, ist eine Vorlage fuer Code gegen etwas, das nicht existiert. Der Node
  ist von 542 auf 347 Zeilen geschrumpft; am laufenden Container nachgesehen
  bietet er jetzt genau `rg6_gripper_controller/gripper_cmd`,
  `rg6/bridge_state` und das Treibergelenk.
- **Das alte Greifermodell ist weg** — `onrobot_rg6.xacro`,
  `onrobot_rg6_model.xacro`, `onrobot_rg6_model_macro.xacro` und die vier
  Meshes `visual/{base_link,inner_finger,inner_knuckle,outer_knuckle}.stl`.
  Sie referenzierten nur noch einander: eine geschlossene tote Insel, die
  jederzeit wieder jemand haette einbinden koennen.
- **`analog_model.hpp` und `test_analog_model.cpp` sind entfallen.** Die
  Kennlinie Greifweite <-> Tool-AI2-Spannung beschreibt einen Kanal, an dem
  der RG6 nicht mehr haengt, und die Kurbelschwinge darin gehoerte zum alten
  Modell. Was der Greifer im Container tut, prueft jetzt eine E2E gegen den
  laufenden Stack (`plan-bridge/tests/test_offboard_gripper.py`).
- `std_srvs`, `rg6_msgs` und `ur_robot_driver` sind aus den Abhaengigkeiten
  von `rg6_control` entfallen.
- **`rg6_msgs` BLEIBT** und sagt jetzt in seiner Paketbeschreibung warum:
  alle fuenf Aufnahmen in `clearpath/data/recordings/` enthalten
  `/a200_0553/manipulators/rg6/state` vom Typ `rg6_msgs/msg/GripperState`.
  Ohne das Paket sind sie nicht mehr abspielbar -- es ist die Typdefinition
  eines Archivs, keine Altlast.

### Nachlese zum URCap-Umbau

- **Der SRDF-Patch traegt wieder zwei `disable_collisions` — und ohne sie plant
  `move_group` gar nichts.** Der Patch hatte seine Eintraege am selben Tag
  entfernt, in der Annahme, der `moveit_collision_updater` erzeuge alle 82
  Greiferpaare selbst und korrekt. Er erzeugt sie — bis auf
  `moment_arm ↔ finger_tip` desselben Fingers. Diese beiden Glieder der
  Viergelenkkette beruehren sich ueber den GANZEN Hub; gemessen mit
  `check_state_validity` bei `rg6_finger_joint` von 0,0 bis 1,25478: in jeder
  Stellung zwei Kontakte, immer dieses Paar. Die Folge war nicht eine
  schlechtere Planung, sondern keine: `CheckStartStateCollision` brach die
  Pipeline ab (`error_code 99999`, 0 Trajektorienpunkte). Mit den zwei Paaren
  ist der ganze Hub gueltig und dasselbe Ziel plant mit 63 Punkten.
- **Der Greifer-Mock spricht das Gelenk, das das Modell hat.** `rg6_control_sim`
  publizierte sechs Gelenke, fuenf davon mit den Namen des alten Modells
  (`left_inner_knuckle_joint` & Co.), und den Treiber bei ganz offener Hand mit
  **-0,93766 rad** — ausserhalb der Grenzen des rg6_v2 (0,0 … 1,25478). Der Wert
  kam aus der Kurbelschwinge des alten Modells (`rg6_control::linkage`), deren
  q=0 bei 93,4 mm liegt statt bei 153,2 mm. Jetzt: nur das Treibergelenk, aus
  der Tabelle des generierten URDF (`finger_kinematics.hpp`). Die fuenf
  Folgegelenke haengen per `<mimic>` am Treiber und werden ohnehin abgeleitet.
  **Das war kein Schoenheitsfehler:** ein `RobotState` mit einem dieser Namen
  brachte `move_group` ueber `RobotModel::getVariableIndex` zum Absturz
  (`std::terminate`, SIGABRT, zweimal reproduziert) — jeder Verbraucher, der
  `joint_states` liest und in eine MoveIt-Anfrage zurueckgibt, war eine
  Abschussrampe.
- **`rg6_control_sim` publiziert `rg6/bridge_state`** (`std_msgs/String`, flaches
  JSON) unter demselben Namen und mit denselben Feldern wie `rg6_grip_bridge`
  am Roboter. Ohne das las der `plan_server` im Container ins Leere.
- `tools/derive_finger_kinematics.py` kann die Getriebetabelle zusaetzlich als
  C++-Header ausgeben (`--format cpp`). Zwei Ausgabeformate, EINE Quelle: die
  Bruecke liest JSON, der Sim braucht C++, und zwei handgepflegte Tabellen sind
  genau die Zweitfassung, vor der die Datei selbst warnt.
- `ur_msgs` ist aus `package.xml`/`CMakeLists.txt` entfallen (die Abhaengigkeit
  gehoerte dem geloeschten Realtreiber); die Paketbeschreibung sagt jetzt, was
  das Paket noch enthaelt.

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

---

**Vor der Einführung von SemVer (2026-08-19)** wurde nach Datum
geführt. Die Abschnitte darunter behalten ihre Datumsüberschrift — ihnen
nachträglich Versionsnummern zu geben, würde eine Release-Historie
erfinden, die es nicht gab.
- **SemVer eingeführt.** Version auf `0.2.0`, dieses Changelog folgt
  [Keep a Changelog](https://keepachangelog.com/de/1.1.0/), Tag `v0.2.0`.
  Ältere Abschnitte behalten ihre Datumsüberschrift — ihnen nachträglich
  Versionsnummern zu geben, würde eine Release-Historie erfinden.
- **README nach dem Workspace-Schema** (readme.so): Features · Tech Stack ·
  Installation · Usage · Running Tests · Related · Versioning · License. Die
  vorhandene Prosa ist erhalten und unter den passenden Abschnitt gewandert.
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
