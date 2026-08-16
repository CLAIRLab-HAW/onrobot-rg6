// analog_model.hpp: Abbildung Greifweite <-> Tool-AI2-Spannung des RG6.
//
// Der Realtreiber kennt die Greifweite NICHT direkt -- er liest ausschliesslich
// den Analogeingang AI2 des UR-Tool-Anschlusses
// (rg6_control.cpp: width_raw_ = msg->analog_input2) und rechnet ihn ueber
// width_from_raw() in Meter um.  Der Sim-Zwilling geht denselben Weg rueckwaerts:
// er kennt die Weite und muss den Rueckkanal daraus erzeugen, damit nachgelagerte
// Verbraucher im Container dieselbe Evidenz sehen wie an der Hardware.
//
// Der wichtigste dieser Verbraucher ist der Verfuegbarkeits-Guard der plan-bridge
// (plan_bridge/reasons.py: gripper_available), der width_raw gegen die Totschwelle
// prueft.  Publiziert der Sim hier NaN, feuert der Guard bei jedem Greifer-
// Kommando -- der Greifer wird im Container untestbar, obwohl er korrekt faehrt.
//
// Die Kalibrierwerte sind 1:1 aus dem Realtreiber uebernommen
// (rg6_control.cpp, Parameter width_in_open/width_in_closed und
// dead_input_threshold).  Beide Nodes lesen jetzt DIESE Konstanten, damit sie
// nicht auseinanderlaufen koennen.

#ifndef RG6_CONTROL__ANALOG_MODEL_HPP_
#define RG6_CONTROL__ANALOG_MODEL_HPP_

#include <algorithm>
#include <cmath>

namespace rg6_control::analog
{

// AI2 bei ganz offen / ganz zu [V] (Defaults von width_in_open/width_in_closed).
inline constexpr double kWidthInOpenV = 10.0;
inline constexpr double kWidthInClosedV = 0.56;

// AI2 unter diesem Wert gilt als "kein gueltiges Feedback" -- Tool stromlos oder
// vor dem ersten Kommando (Default von dead_input_threshold).  Der Treiber ueberspringt
// dann seinen Pre-Check, die plan-bridge meldet not_available.
inline constexpr double kDeadInputThresholdV = 0.2;

// Was der Sim bei weggenommener Toolspannung meldet.  Am 2026-08-12 am echten
// a200_0553 lag AI2 stromlos bei ~0,05 V (gemessen: 0,051 / 0,064 V), also
// deutlich unter der Totschwelle -- der Totzustand bleibt damit im Container
// provozierbar und ist nicht bloss ein ausgedachter Wert.
inline constexpr double kSimDeadInputV = 0.05;

// Lineare Abbildung mit Klemmung auf das Zielintervall.
inline double map_clamped(double x, double x0, double x1, double y0, double y1)
{
  if (std::abs(x1 - x0) < 1e-12) {
    return y0;
  }
  const double y = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
  return std::clamp(y, std::min(y0, y1), std::max(y0, y1));
}

// AI2-Spannung -> Weite [m].  Identisch zu rg6_control.cpp: width_from_raw().
inline double width_from_raw(
  double raw_v,
  double in_closed_v = kWidthInClosedV, double in_open_v = kWidthInOpenV,
  double width_closed_m = 0.0, double width_open_m = 0.160)
{
  return map_clamped(raw_v, in_closed_v, in_open_v, width_closed_m, width_open_m);
}

// Weite [m] -> AI2-Spannung.  Die Umkehrung von width_from_raw() und damit der
// Weg, den nur der Sim geht (die Hardware misst, sie rechnet nicht zurueck).
//
// Durch die Klemmung liegt das Ergebnis ueber den GANZEN Hub zwischen
// kWidthInClosedV (0,56 V) und kWidthInOpenV (10,0 V) -- also immer oberhalb der
// Totschwelle (0,2 V).  Ein bestromter Greifer ist damit ueber jede Weite
// hinweg "verfuegbar", ein stromloser faellt mit kSimDeadInputV darunter.  Die
// Abbildung ist sauber getrennt, nicht grenzwertig.
inline double raw_from_width(
  double width_m,
  double width_closed_m = 0.0, double width_open_m = 0.160,
  double in_closed_v = kWidthInClosedV, double in_open_v = kWidthInOpenV)
{
  return map_clamped(width_m, width_closed_m, width_open_m, in_closed_v, in_open_v);
}

}  // namespace rg6_control::analog

// ---------------------------------------------------------------------------
// Greifweite <-> rg6_finger_joint, nach der GETRIEBEGEOMETRIE.
//
// Bis zum 2026-08-16 lief diese Abbildung linear zwischen zwei frei gesetzten
// Ankern (Parameter ``angle_open`` 0.0 und ``angle_closed`` 0.6).  Beide waren
// falsch, und die Folge war im Planungsmodell sichtbar: bei "ganz offen"
// standen die Backen 93,7 mm auseinander statt 160 mm, bei "ganz zu" blieben
// 4,3 mm Spalt stehen.  Der Greifer griff also in jedes Objekt hinein, das
// breiter als 94 mm war, und schloss nie ganz.  move_group prueft dieselbe
// URDF -- die modellierte Hand war damit bei gleicher kommandierter Weite
// SCHMALER als die echte, jede Freiraumpruefung um den Greifer herum also
// optimistisch.
//
// Die wahre Beziehung folgt aus den Gelenkursprungen in
// ``rg6_description/urdf/onrobot_rg6_model_macro.xacro``.  Das ``finger_joint``
// sitzt bei y = -0,024112 auf der Basis, das ``inner_finger_joint`` bei
// (y, z) = (-0,047335, +0,064495) auf dem ``outer_knuckle``; der
// ``inner_finger`` selbst dreht per ``mimic`` mit +1 zurueck, bleibt also
// parallel.  Damit ist der Abstand der beiden ``inner_finger``-Ursprunge
//
//     d(q) = 2 * (a + L * cos(q + phi0))
//
// mit a = 0,024112 m, L = |(0,047335, 0,064495)| = 0,080 m und
// phi0 = atan2(0,064495, 0,047335) = 0,93766 rad.  Die lichte Weite zwischen
// den Pads ist d(q) minus 2 * 0,0246 m (Padflaeche aus der Meshbox von
// ``inner_finger.stl``).
//
// Am laufenden husky-offboard-Container am 2026-08-16 auf DREI unabhaengigen
// Wegen belegt:
//   1. die Rechnung oben;
//   2. TF zwischen ``rg6_left_inner_finger`` und ``rg6_right_inner_finger``:
//      143 / 112 / 80 / 54 mm bei kommandierten 160 / 100 / 45 / 0 mm;
//   3. move_groups eigene ``check_state_validity`` gegen eine Box wachsender
//      Breite zwischen den Pads: 92 / 60 / 28 / 4 mm lichte Weite.
//
// Warum das Getriebe trotzdem RG6 ist und nicht der Robotiq, dem die
// Gelenknamen entstammen: die obere Gelenkgrenze (+0,628319) faellt bis auf
// 1,3 mrad mit der geschlossenen Stellung zusammen (dort -0,20 mm, also
// gerade eben ueberlappend), und das geometrische Maximum liegt bei 159,0 mm
// -- also am RG6-Hub von 160 mm.  Falsch war nur, auf welchen Ausschnitt
// dieses Wegs die Weite abgebildet wurde.
namespace rg6_control::linkage
{

// VIER abgelesene Zahlen, alles andere wird daraus gerechnet.  Diese Trennung
// ist der Punkt: ein zusaetzliches Literal fuer Kurbel oder Phase ist eine
// Zahl, die neben der Geometrie stehen kann, die sie beschreibt -- und genau
// das ist beim ersten Anlauf dieses Fixes passiert (0.0800005 / 0.9375699,
// 0,09 mrad daneben).  Dieselben vier Zahlen stehen im Roboterprofil
// (robot_contract, gripper.linkage) und werden dort genauso ausgewertet.
//
// Basisversatz des ``finger_joint`` in y [m] (URDF: 0,024112).
inline constexpr double kBaseOffsetM = 0.024112;
// Versatz des ``inner_finger_joint`` auf dem ``outer_knuckle`` [m]
// (URDF: 0,071447-0,024112 bzw. 0,201308-0,136813).
inline constexpr double kCrankYM = 0.047335;
inline constexpr double kCrankZM = 0.064495;
// Wie weit die Padflaeche vom Ursprung ihres Links nach INNEN reicht [m]
// (Meshbox von inner_finger.stl, lokal y_max = +0,0246005).
inline constexpr double kPadOffsetM = 0.0246;

// Kurbellaenge [m] und Anfangswinkel [rad] -- gerechnet, nicht gepflegt.
// ``inline const`` statt ``constexpr``, weil ``std::hypot``/``std::atan2``
// nicht ``constexpr`` sind; benutzt werden sie nur zur Laufzeit.
inline const double kCrankM = std::hypot(kCrankYM, kCrankZM);
inline const double kCrankPhaseRad = std::atan2(kCrankZM, kCrankYM);

// Groesste lichte Weite, die das Getriebe hergibt [m] -- die Kurbel-Totlage
// bei q = -kCrankPhaseRad.  Etwas UNTER dem nominellen RG6-Hub von 0,160 m;
// eine breitere Weite ist mechanisch nicht darstellbar und wird geklemmt.
inline const double kMaxWidthM =
  2.0 * (kBaseOffsetM + kCrankM - kPadOffsetM);

// Gelenkwert der ganz geschlossenen Hand [rad] -- die Stellung mit lichter
// Weite null.
//
// Die OBERE GELENKGRENZE der URDF (+0,628319) liegt 1,3 mrad dahinter; dort
// ueberlappen sich die Pads rechnerisch um 0,20 mm.  Nah genug, dass die
// Grenze erkennbar auf die geschlossene Stellung gelegt wurde -- aber es ist
// nicht dieselbe Zahl, und die hier gebrauchte ist die geometrische.  Die
// Grenze bleibt, wo sie ist: sie gibt dem Regler etwas Weg hinter dem Ziel,
// und 0,2 mm Ueberlapp sind weniger als das Umkehrspiel des Geraets
// (0,1..0,3 mm laut Handbuch v6.6.2, Abschnitt 8.1.4).
inline const double kClosedAngleRad =
  std::acos((kPadOffsetM - kBaseOffsetM) / kCrankM) - kCrankPhaseRad;

// Lichte Weite [m] beim Fingergelenk ``q`` [rad].
inline double width_from_angle(double q)
{
  return 2.0 * (kBaseOffsetM + kCrankM * std::cos(q + kCrankPhaseRad)
                - kPadOffsetM);
}

// Fingergelenk [rad] fuer die lichte Weite ``width_m``.  Die Umkehrung von
// :func:`width_from_angle`, mit Klemmung auf den darstellbaren Bereich: ohne
// sie liefe ``acos`` fuer die kommandierten 160 mm aus dem Definitionsbereich
// und alle sechs Gelenkwerte wuerden NaN -- der Greifer verschwaende dann aus
// dem Kollisionsmodell, statt bloss falsch dazustehen.
// Geklemmt wird die WEITE, nicht erst das ``acos``-Argument. Eine negative
// Weite liegt naemlich noch im Definitionsbereich des Kosinus und ergaebe
// einen Gelenkwert JENSEITS der geschlossenen Stellung (fuer -50 mm etwa
// 0,944 rad statt 0,628) -- die Finger fuhren dann durcheinander hindurch,
// still und ohne NaN.
inline double angle_from_width(double width_m)
{
  const double w = std::clamp(width_m, 0.0, kMaxWidthM);
  const double cos_arg = (0.5 * w + kPadOffsetM - kBaseOffsetM) / kCrankM;
  return std::acos(std::clamp(cos_arg, -1.0, 1.0)) - kCrankPhaseRad;
}

}  // namespace rg6_control::linkage

#endif  // RG6_CONTROL__ANALOG_MODEL_HPP_
