// Pinnt den analogen Rueckkanal des RG6-Sims in BEIDE Richtungen:
// bestromt -> die plan-bridge sieht einen gueltigen Eingang (Greifer verfuegbar),
// stromlos -> sie sieht einen toten Eingang (not_available).
//
// Der Guard, gegen den hier gepinnt wird, ist plan_bridge/reasons.py:
//
//     width_raw is None or float(width_raw) <= GRIPPER_DEAD_INPUT_V  -> nicht verfuegbar
//
// mit GRIPPER_DEAD_INPUT_V = 0.2, dort seinerseits gegen die am 2026-08-12 am
// echten a200_0553 gemessene Tabelle gepinnt (plan-bridge/tests/test_reasons.py).
// Die fruehere Fassung meldete hier NaN -- in Python ein float, das JEDEN
// Vergleich verliert -> der Guard feuerte bei jedem Kommando.

#include <cmath>

#include <gtest/gtest.h>

#include "rg6_control/analog_model.hpp"

namespace analog = rg6_control::analog;

namespace
{
// Hub des RG6 (rg6_control.cpp: width_open_m / width_closed_m).
constexpr double kOpenM = 0.160;
constexpr double kClosedM = 0.0;
}  // namespace

// --- Richtung 1: bestromt -> verfuegbar --------------------------------------

TEST(SimAnalogModel, PoweredFeedbackStaysAboveTheDeadThresholdOverTheWholeStroke)
{
  // Die Zielweiten, die der Container am 2026-08-12 tatsaechlich angefahren hat
  // (0.16 / 0.0 / 0.08 / 0.02), plus die Raender.
  for (const double width_m : {0.0, 0.005, 0.02, 0.049, 0.08, 0.16}) {
    const double raw = analog::raw_from_width(width_m, kClosedM, kOpenM);
    EXPECT_GT(raw, analog::kDeadInputThresholdV)
      << "Weite " << width_m << " m -> " << raw << " V liegt auf/unter der "
      << "Totschwelle; die plan-bridge wuerde den Greifer fuer stromlos halten.";
    EXPECT_TRUE(std::isfinite(raw)) << "NaN war genau der Bug: " << width_m;
  }
}

TEST(SimAnalogModel, PoweredFeedbackHitsTheDriverEndpointsExactly)
{
  // Paritaet mit dem Realtreiber: ganz zu -> 0,56 V, ganz offen -> 10,0 V.
  EXPECT_DOUBLE_EQ(analog::raw_from_width(kClosedM, kClosedM, kOpenM), analog::kWidthInClosedV);
  EXPECT_DOUBLE_EQ(analog::raw_from_width(kOpenM, kClosedM, kOpenM), analog::kWidthInOpenV);

  // Auch ausserhalb des Hubs bleibt der Wert im gueltigen Band (Klemmung) --
  // ein Ueberschwinger im Bewegungsmodell darf keinen toten Eingang vortaeuschen.
  EXPECT_DOUBLE_EQ(analog::raw_from_width(-0.05, kClosedM, kOpenM), analog::kWidthInClosedV);
  EXPECT_DOUBLE_EQ(analog::raw_from_width(0.5, kClosedM, kOpenM), analog::kWidthInOpenV);
}

// --- Richtung 2: stromlos -> not_available -----------------------------------

TEST(SimAnalogModel, UnpoweredFeedbackFallsBelowTheDeadThreshold)
{
  // Das ist die Bedingung, unter der der simulierte Analogwert vertretbar ist:
  // set_tool_power(false) muss im Container weiterhin als Totzustand ankommen.
  EXPECT_LT(analog::kSimDeadInputV, analog::kDeadInputThresholdV)
    << "Der stromlose Sim-Wert liegt nicht mehr unter der Totschwelle -- der "
    << "Totzustand waere im Container nicht mehr provozierbar.";

  // Und er darf nicht versehentlich in den gueltigen Bereich rutschen: die
  // geschlossene Backe (0,56 V) liegt sauber ueber der Schwelle, nicht knapp.
  EXPECT_GT(analog::kWidthInClosedV, analog::kDeadInputThresholdV);
}

// --- Kalibrierung gegen die gemessene Hardware -------------------------------

TEST(SimAnalogModel, CalibrationMatchesTheRealDriverDefaults)
{
  // Diese drei Zahlen sind die Schnittstelle zur Hardware.  Aendert sie jemand
  // im Realtreiber, ohne den Sim mitzuziehen, faellt es hier auf.
  EXPECT_DOUBLE_EQ(analog::kWidthInOpenV, 10.0);      // width_in_open
  EXPECT_DOUBLE_EQ(analog::kWidthInClosedV, 0.56);    // width_in_closed
  EXPECT_DOUBLE_EQ(analog::kDeadInputThresholdV, 0.2);  // dead_input_threshold
}

TEST(SimAnalogModel, RawAndWidthAreInverseOfEachOther)
{
  // Der Sim rechnet rueckwaerts, was der Realtreiber vorwaerts rechnet -- wenn
  // die beiden auseinanderlaufen, meldet der Sim eine andere Weite als die,
  // die er tatsaechlich faehrt.
  for (const double width_m : {0.0, 0.02, 0.049, 0.08, 0.16}) {
    const double raw = analog::raw_from_width(width_m, kClosedM, kOpenM);
    EXPECT_NEAR(analog::width_from_raw(raw), width_m, 1e-9) << "bei " << width_m << " m";
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// --- Richtung 3: Weite <-> Fingergelenk, nach der GETRIEBEGEOMETRIE ----------
//
// Bis 2026-08-16 lief diese Abbildung linear zwischen zwei frei gesetzten
// Ankern (`angle_open` 0.0, `angle_closed` 0.6).  Beide Anker waren falsch, und
// zwar sichtbar: bei "ganz offen" standen die Backen 93,7 mm auseinander statt
// 160 mm, bei "ganz zu" blieben 4,3 mm Spalt.  Der Greifer griff im
// Planungsmodell in jedes Objekt hinein.
//
// Die wahre Beziehung folgt aus den Gelenkursprungen in
// rg6_description/urdf/onrobot_rg6_model_macro.xacro:
//
//     Ursprungsabstand(q) = 2 * (a + L * cos(q + phi0))
//
// mit a = 0,024112 m (Basisversatz), L = 0,080 m (Kurbel), phi0 = 0,93766 rad,
// und die lichte Weite ist der Ursprungsabstand minus 2 * 0,0246 m (Padflaeche
// aus der Meshbox von inner_finger.stl).
//
// Am laufenden husky-offboard-Container am 2026-08-16 auf DREI unabhaengigen
// Wegen belegt: Rechnung, TF zwischen den beiden inner_finger-Frames, und
// move_groups eigene Kollisionspruefung (check_state_validity) gegen eine Box
// wachsender Breite zwischen den Pads.

namespace linkage = rg6_control::linkage;

TEST(LinkageModel, TheCrankIsDerivedFromTheTwoUrdfOffsetsAndNotMaintained)
{
  // Kurbel und Phase sind gerechnet, nicht gepflegt.  Der erste Anlauf dieses
  // Fixes trug sie als eigene Literale (0.0800005 / 0.9375699) und lag damit
  // 0,09 mrad neben der Geometrie, die sie beschreiben sollten -- klein, aber
  // genau die Art Drift, gegen die der ganze Umbau da ist.  Dieselben vier
  // abgelesenen Zahlen fuehrt das Roboterprofil (robot_contract,
  // gripper.linkage); driften duerfen sie nicht.
  EXPECT_NEAR(linkage::kCrankM, std::hypot(0.047335, 0.064495), 1e-12);
  EXPECT_NEAR(linkage::kCrankPhaseRad, std::atan2(0.064495, 0.047335), 1e-12);
  EXPECT_NEAR(linkage::kCrankM, 0.0800013, 1e-7);
  EXPECT_NEAR(linkage::kCrankPhaseRad, 0.9376577, 1e-7);
}

TEST(LinkageModel, TheClosedPoseSitsPracticallyOnTheJointsUpperLimit)
{
  // Die geschlossene Stellung (lichte Weite null) und die obere Gelenkgrenze
  // der URDF liegen 1,3 mrad auseinander -- erkennbar dieselbe Absicht, aber
  // nicht dieselbe Zahl.  Die Grenze bleibt die groessere: 0,20 mm Ueberlapp
  // sind weniger als das Umkehrspiel des Geraets (0,1..0,3 mm laut Handbuch
  // v6.6.2, Abschnitt 8.1.4), und der Regler bekommt Weg hinter dem Ziel.
  EXPECT_NEAR(linkage::width_from_angle(linkage::kClosedAngleRad), 0.0, 1e-12);
  EXPECT_NEAR(linkage::angle_from_width(0.0), linkage::kClosedAngleRad, 1e-9);
  EXPECT_NEAR(linkage::kClosedAngleRad, 0.6270387, 1e-7);
  EXPECT_NEAR(linkage::width_from_angle(0.628319), -0.0002, 1e-4);
  // ...und bei q = 0,6, dem alten ``angle_closed``, blieben 4,3 mm Spalt --
  // genau das, was der Owner in Foxglove als "nicht ganz zu" gesehen hat.
  EXPECT_NEAR(linkage::width_from_angle(0.600000), 0.0043, 5e-4);
}

TEST(LinkageModel, TheOldOpenAnchorWasNotOpenAtAll)
{
  // q = 0 war `angle_open` und ist in Wahrheit die halb offene Hand.
  EXPECT_NEAR(linkage::width_from_angle(0.0), 0.0937, 5e-4);
}

TEST(LinkageModel, TheJointsLowerLimitReachesAlmostTheFullStroke)
{
  EXPECT_NEAR(linkage::width_from_angle(-0.628319), 0.1514, 5e-4);
  // ...und das geometrische Maximum liegt bei der Kurbel-Totlage.
  EXPECT_NEAR(linkage::width_from_angle(-linkage::kCrankPhaseRad),
              linkage::kMaxWidthM, 1e-6);
  EXPECT_NEAR(linkage::kMaxWidthM, 0.1590, 5e-4);
}

TEST(LinkageModel, TheFourWidthsMeasuredInTheContainerMapToTheirJointValues)
{
  // Ursprungsabstaende per TF gemessen (143 / 112 / 80 / 54 mm), daraus die
  // lichte Weite; hier die Umkehrung -- Weite -> Gelenkwert.
  //
  // Toleranz 6e-3 rad, und das ist die MESSUNG, nicht Nachsicht: tf2_echo
  // gibt Millimeter aus, ein halber Millimeter Ableseunsicherheit in der
  // Weite entspricht hier rund 0,004 rad. Enger zu pinnen hiesse, die
  // eigene Rundung fuer eine Messung zu halten.
  EXPECT_NEAR(linkage::angle_from_width(0.0937), 0.0, 6e-3);
  EXPECT_NEAR(linkage::angle_from_width(0.0625), 0.225, 6e-3);
  EXPECT_NEAR(linkage::angle_from_width(0.0311), 0.43125, 6e-3);
  EXPECT_NEAR(linkage::angle_from_width(0.0043), 0.600, 6e-3);
}

TEST(LinkageModel, TheMappingRoundTrips)
{
  for (const double w : {0.0, 0.01, 0.05, 0.0937, 0.12, 0.1514, 0.159}) {
    EXPECT_NEAR(linkage::width_from_angle(linkage::angle_from_width(w)), w, 1e-6)
      << "Weite " << w << " m ueberlebt den Hin- und Rueckweg nicht";
  }
}

TEST(LinkageModel, AWidthBeyondTheLinkageIsClampedInsteadOfProducingNaN)
{
  // Kommandiert werden 0..160 mm, das Getriebe gibt nur 159,0 mm her. Ohne
  // Klemmung liefe acos() aus dem Definitionsbereich und die Gelenkwerte
  // wuerden NaN -- der Greifer verschwaende dann aus dem Kollisionsmodell.
  const double q = linkage::angle_from_width(0.160);
  EXPECT_TRUE(std::isfinite(q));
  EXPECT_NEAR(q, -linkage::kCrankPhaseRad, 1e-9);
  // Und eine NEGATIVE Weite muss auf "ganz zu" klemmen. Sie liegt noch im
  // Definitionsbereich des Kosinus, ergaebe also klaglos einen Gelenkwert
  // jenseits der geschlossenen Stellung -- die Finger fuhren durcheinander
  // hindurch, ohne NaN und ohne Warnung.
  EXPECT_TRUE(std::isfinite(linkage::angle_from_width(-0.05)));
  EXPECT_NEAR(linkage::angle_from_width(-0.05), linkage::kClosedAngleRad, 1e-6);
}
