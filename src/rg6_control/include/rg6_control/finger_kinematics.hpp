// ERZEUGT, NICHT GEPFLEGT -- tools/derive_finger_kinematics.py aus dem
// generierten URDF des Greifermodells.  Nach jeder Aenderung am Modell neu
// erzeugen; von Hand editierte Zahlen laufen still gegen den Roboter weg.
//
// Gelenkwinkel rg6_finger_joint [rad] -> lichte Weite zwischen den Padflaechen
// [m], gemessen zwischen den beiden flex_finger-Meshes.  Dieselbe Tabelle
// liegt als JSON neben der Greiferbruecke auf dem Roboter
// (husky-custom-setup/scripts/rg6_finger_kinematics.json) und im Roboterprofil
// (robot_contract, gripper.linkage.table).
//
// Es gibt keine geschlossene Formel:  die Finger sind eine Viergelenkkette.
// Die Vorgaengerfassung (rg6_control::linkage, Kurbelschwinge) gehoerte zum
// ALTEN Greifermodell und lag mit dem rg6_v2 um mehr als 60 mm daneben.
//
// Obergrenze 1.25478 rad = Nulldurchgang der Weite; darueber fahren die
// Finger im Modell durcheinander hindurch.
// Max. Interpolationsfehler gegen ein 400-Punkte-Gitter: 0.047 mm.

#ifndef RG6_CONTROL__FINGER_KINEMATICS_HPP_
#define RG6_CONTROL__FINGER_KINEMATICS_HPP_

#include <algorithm>
#include <array>
#include <cstddef>

namespace rg6_control::finger_kinematics
{

inline constexpr double kQMinRad = 0.0;
inline constexpr double kQMaxRad = 1.25478;

//: Stuetzstellen (q [rad], Weite [m]), streng monoton fallend in der Weite.
inline constexpr std::array<std::array<double, 2>, 27> kTable = {{
  {{0.00000, 0.153168}},
  {{0.05000, 0.150446}},
  {{0.10000, 0.147351}},
  {{0.15000, 0.143891}},
  {{0.20000, 0.140075}},
  {{0.25000, 0.135912}},
  {{0.30000, 0.131413}},
  {{0.35000, 0.126590}},
  {{0.40000, 0.121453}},
  {{0.45000, 0.116016}},
  {{0.50000, 0.110293}},
  {{0.55000, 0.104298}},
  {{0.60000, 0.098045}},
  {{0.65000, 0.091551}},
  {{0.70000, 0.084832}},
  {{0.75000, 0.077904}},
  {{0.80000, 0.070784}},
  {{0.85000, 0.063492}},
  {{0.90000, 0.056044}},
  {{0.95000, 0.048459}},
  {{1.00000, 0.040757}},
  {{1.05000, 0.032957}},
  {{1.10000, 0.025078}},
  {{1.15000, 0.017139}},
  {{1.20000, 0.009161}},
  {{1.25000, 0.001164}},
  {{1.25478, 0.000399}}
}};

inline constexpr double kMaxWidthM = kTable.front()[1];
inline constexpr double kMinWidthM = kTable.back()[1];

//: Lichte Weite [m] beim Fingergelenk ``q`` [rad], linear interpoliert.
inline double width_from_angle(double q)
{
  const double x = std::clamp(q, kQMinRad, kQMaxRad);
  for (std::size_t i = 1; i < kTable.size(); ++i) {
    if (x <= kTable[i][0]) {
      const double q0 = kTable[i - 1][0], w0 = kTable[i - 1][1];
      const double q1 = kTable[i][0], w1 = kTable[i][1];
      const double t = (q1 - q0) > 0.0 ? (x - q0) / (q1 - q0) : 0.0;
      return w0 + t * (w1 - w0);
    }
  }
  return kMinWidthM;
}

//: Fingergelenk [rad] fuer die lichte Weite ``width_m``.  Die Weite wird auf
//: den darstellbaren Bereich GEKLEMMT -- ein Befehl ueber den Hub hinaus ist
//: gueltig und bedeutet "so weit wie es geht", nicht NaN.
inline double angle_from_width(double width_m)
{
  const double w = std::clamp(width_m, kMinWidthM, kMaxWidthM);
  for (std::size_t i = 1; i < kTable.size(); ++i) {
    if (w >= kTable[i][1]) {
      const double q0 = kTable[i - 1][0], w0 = kTable[i - 1][1];
      const double q1 = kTable[i][0], w1 = kTable[i][1];
      const double t = (w0 - w1) > 0.0 ? (w0 - w) / (w0 - w1) : 0.0;
      return q0 + t * (q1 - q0);
    }
  }
  return kQMaxRad;
}

}  // namespace rg6_control::finger_kinematics

#endif  // RG6_CONTROL__FINGER_KINEMATICS_HPP_
