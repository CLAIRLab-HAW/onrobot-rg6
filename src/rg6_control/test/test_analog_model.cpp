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
