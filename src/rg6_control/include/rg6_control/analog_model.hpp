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

#endif  // RG6_CONTROL__ANALOG_MODEL_HPP_
