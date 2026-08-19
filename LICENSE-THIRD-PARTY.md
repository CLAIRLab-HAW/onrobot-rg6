# Fremdbestandteile

## onrobot_description (rg6_v2)

* Herkunft: <https://github.com/inria-paris-robotics-lab/onrobot_ros>, Zweig `ros2`
* Paket: `onrobot_description` v0.5.0, Maintainer Igor Kalevatykh (INRIA)
* Lizenz: MIT, Volltext in [LICENSE-UPSTREAM-MIT](LICENSE-UPSTREAM-MIT)
* Übernommen am 2026-08-19: `meshes/rg6_v2/` (15 STL), `config/rg6_v2.yaml`,
  `urdf/onrobot_rg.urdf.xacro` (hier `onrobot_rg_upstream.urdf.xacro`),
  `urdf/materials.urdf.xacro` (hier `materials_upstream.urdf.xacro`)

### Geändert gegenüber Upstream

* Paketpfade auf `rg6_description` umgebogen (16 Stellen).
* `<ros2_control>`-Block und der `rg_gazebo`-Aufruf entfernt, dazu die
  Includes für Transmission und Gazebo. **Grund:** der Greifer hängt an
  diesem Roboter nicht am `controller_manager`, sondern wird per XML-RPC an
  die OnRobot-URCap kommandiert (`rg6_grip_bridge`, ROBOTER-TODO R21). Ein
  zweites ros2_control-System im Roboter-URDF wäre ein Gerät, das niemand
  bedient; Gazebo läuft in diesem Workspace gar nicht.
* Kollisionsgeometrie auf die mitgelieferten `collision/`-Meshes umgestellt.
  Upstream referenziert dafür die `visual/`-Meshes (`body.stl` allein 10 486
  Dreiecke) und lässt die kleineren Kollisionsdateien ungenutzt daneben
  liegen. **Ausnahme `finger_tip`:** dort führt `collision/` zwei Hälften
  (`finger_tip_1/2.stl`) statt einer gleichnamigen Datei — dieser eine Link
  behält das visuelle Mesh (2154 Dreiecke).
* Treibergelenkname parametrisiert (s. Repo-CHANGELOG).
* Der Link `finger_tip` fuehrte den `<collision>`-Block **doppelt**; im
  kompilierten MuJoCo-Modell erschienen dadurch zwei identische
  Kollisionsmeshes. Einer entfernt.
* `config/rg6_v2.yaml`: `limit.upper` von **1.3** auf **1.25478** geklemmt.
  Bei 1,3 rad sind die Finger im Modell bereits durcheinander hindurch — die
  lichte Weite erreicht bei 1,25478 null und wächst danach wieder. Upstream
  faellt das nicht auf, weil dort niemand die Weite aus der Kinematik
  ableitet.

Die Meshes bilden Hardware von OnRobot A/S ab; das Urheberrecht am
abgebildeten Gerät bleibt dort. Übernommen wurde die MIT-lizenzierte
ROS-Beschreibung, nicht OnRobots CAD.
