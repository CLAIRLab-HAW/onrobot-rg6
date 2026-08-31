// GENERATED, NOT MAINTAINED -- tools/derive_finger_kinematics.py from the
// generated URDF of the gripper model.  Regenerate after every change to the
// model; numbers edited by hand drift away from the robot in silence.
//
// Joint angle rg6_finger_joint [rad] -> clear width between the pad faces [m], measured
// between the two flex_finger meshes.  The same table lies as JSON next to the
// gripper bridge (scripts/rg6_finger_kinematics.json, read by rg6_grip_bridge
// on the robot) and in the robot profile (robot_contract,
// gripper.linkage.table).
//
// There is no closed formula: the fingers are a four-bar chain.
//
// Lower bound 0.03800 rad = the mechanical open stop; the chain computes a wider
// opening below it that the hardware does not reach.
// Upper bound 1.25478 rad = the zero crossing of the width; beyond it the
// fingers drive through one another in the model.
// Max. interpolation error against a 400-point grid: 0.046 mm.

#ifndef RG6_CONTROL__FINGER_KINEMATICS_HPP_
#define RG6_CONTROL__FINGER_KINEMATICS_HPP_

#include <algorithm>
#include <array>
#include <cstddef>

namespace rg6_control::finger_kinematics
{

inline constexpr double kQMinRad = 0.03800;
inline constexpr double kQMaxRad = 1.25478;

//: Sampling points (q [rad], width [m]), strictly monotonically falling in the width.
inline constexpr std::array<std::array<double, 2>, 26> kTable = {{
  {{0.03800, 0.151133}},
  {{0.08800, 0.148127}},
  {{0.13800, 0.144754}},
  {{0.18800, 0.141023}},
  {{0.23800, 0.136942}},
  {{0.28800, 0.132523}},
  {{0.33800, 0.127776}},
  {{0.38800, 0.122714}},
  {{0.43800, 0.117348}},
  {{0.48800, 0.111692}},
  {{0.53800, 0.105761}},
  {{0.58800, 0.099568}},
  {{0.63800, 0.093131}},
  {{0.68800, 0.086464}},
  {{0.73800, 0.079584}},
  {{0.78800, 0.072509}},
  {{0.83800, 0.065257}},
  {{0.88800, 0.057844}},
  {{0.93800, 0.050291}},
  {{0.98800, 0.042616}},
  {{1.03800, 0.034837}},
  {{1.08800, 0.026975}},
  {{1.13800, 0.019049}},
  {{1.18800, 0.011079}},
  {{1.23800, 0.003084}},
  {{1.25478, 0.000399}}
}};

inline constexpr double kMaxWidthM = kTable.front()[1];
inline constexpr double kMinWidthM = kTable.back()[1];

//: Clear width [m] at finger joint ``q`` [rad], linearly interpolated.
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

//: Finger joint [rad] for the clear width ``width_m``.  The width is CLAMPED to
//: the representable range -- a command past the stroke is valid and means "as
//: far as it goes", not NaN.
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
