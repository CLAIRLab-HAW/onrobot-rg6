"""The finger linkage table exists three times and is interpolated three times.  All six must agree.

The RG6 fingers are a four-bar chain with no closed form, so the joint angle -> clear width relation is a generated
TABLE (``tools/derive_finger_kinematics.py``).  It has to be readable in three places that cannot import one another,
and so it was copied into three:

* ``src/rg6_control/include/rg6_control/finger_kinematics.hpp`` -- the container mock, C++
* ``robot/husky-custom-setup/scripts/rg6_finger_kinematics.json`` -- the gripper bridge on the robot, Python
* ``contract/robot-contract`` profile, ``gripper.linkage.table`` -- the workstation code, Python

The generator writes the first two.  The third is a HAND copy, exactly like the arm poses were before
``test_ssot_parity.py`` -- and those had already drifted (``packed[5]`` lost a digit).  A drift here is worse than a
wrong number: the container mock would then compute a different width from the same command than the real robot, and
the entire point of the mock is that it does not.

Each copy also carries its own interpolation code -- generated C++, ``GripperLinkage`` in ``robot_contract``, and
``FingerKinematics`` in the bridge.  Same table, three implementations, and only their agreement makes the table one
source rather than three.  So this compares the data AND the results.

Neither ROS nor a robot is needed; the C++ side is compiled here, and skips with a named cause where no compiler is.
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess

import pytest
import yaml

from table_sources import sibling

PROFILE_RELPATH = "contract/robot-contract/src/robot_contract/profiles/a200_0553.yaml"
JSON_RELPATH = "robot/husky-custom-setup/scripts/rg6_finger_kinematics.json"
BRIDGE_RELPATH = "robot/husky-custom-setup/scripts/rg6_grip_bridge.py"

#: Angles to compare the three implementations at: both bounds, both sides of every kind of edge, and a spread of
#: interior points.  Values outside the range are deliberate -- the clamp is behaviour, not an accident.
PROBE_ANGLES = [-1.0, 0.0, 0.038, 0.0381, 0.05, 0.3, 0.6, 0.9, 1.0, 1.2, 1.25478, 1.3, 5.0]


def _header_text(repo_root):
    return (repo_root / "src/rg6_control/include/rg6_control/finger_kinematics.hpp").read_text()


def _cpp_table(repo_root):
    """The sampling points as written into the generated header."""
    return [[float(q), float(w)] for q, w in re.findall(r"\{\{([\d.]+), ([\d.]+)\}\}", _header_text(repo_root))]


def _json_table():
    return [[float(q), float(w)] for q, w in json.loads(sibling(JSON_RELPATH).read_text())["table_q_rad_width_m"]]


def _profile_table():
    raw = yaml.safe_load(sibling(PROFILE_RELPATH).read_text())
    return [[float(q), float(w)] for q, w in raw["gripper"]["linkage"]["table"]]


# ---- the table itself ------------------------------------------------------------------------------------------
def test_the_header_carries_a_table_at_all(repo_root):
    """Guards the tests below: an empty parse would make every comparison pass for the wrong reason."""
    table = _cpp_table(repo_root)
    assert len(table) > 2


def test_the_json_copy_matches_the_header(repo_root):
    assert _json_table() == _cpp_table(repo_root)


def test_the_profile_copy_matches_the_header(repo_root):
    """The hand copy.  Compared with ``==``, not ``approx``: any difference means it was retyped, not generated."""
    assert _profile_table() == _cpp_table(repo_root)


def test_the_declared_joint_limits_match_the_table_ends(repo_root):
    table = _cpp_table(repo_root)
    header = _header_text(repo_root)
    q_min = float(re.search(r"kQMinRad = ([\d.]+)", header).group(1))
    q_max = float(re.search(r"kQMaxRad = ([\d.]+)", header).group(1))
    assert [q_min, q_max] == [table[0][0], table[-1][0]]
    assert json.loads(sibling(JSON_RELPATH).read_text())["joint_limits_rad"] == [q_min, q_max]


def test_the_width_falls_strictly_over_the_whole_table(repo_root):
    """Both inversions search for the first sampling point at or below the target width; a rise would break them."""
    widths = [w for _q, w in _cpp_table(repo_root)]
    assert all(b < a for a, b in zip(widths, widths[1:]))


def test_the_angle_rises_strictly_over_the_whole_table(repo_root):
    angles = [q for q, _w in _cpp_table(repo_root)]
    assert all(b > a for a, b in zip(angles, angles[1:]))


# ---- the three implementations ---------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def cpp_probe(repo_root, tmp_path_factory):
    """Compile the generated header and read its results back for ``PROBE_ANGLES``.

    The header is plain C++17 with no ROS in it, so this needs a compiler and nothing else.  Without one the
    comparison is skipped by name rather than quietly dropped.
    """
    compiler = shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        pytest.skip("no C++ compiler on PATH -- cannot check the generated header against the Python copies")

    tmp = tmp_path_factory.mktemp("rg6_cpp")
    src = tmp / "probe.cpp"
    src.write_text(
        "#include <cstdio>\n"
        '#include "rg6_control/finger_kinematics.hpp"\n'
        "int main(int argc, char** argv) {\n"
        "  using namespace rg6_control::finger_kinematics;\n"
        "  for (int i = 1; i < argc; ++i) {\n"
        "    const double q = atof(argv[i]);\n"
        "    const double w = width_from_angle(q);\n"
        '    printf("%.12f %.12f\\n", w, angle_from_width(w));\n'
        "  }\n"
        "  return 0;\n"
        "}\n"
    )
    binary = tmp / "probe"
    include = repo_root / "src/rg6_control/include"
    subprocess.run(
        [compiler, "-std=c++17", "-O0", "-I", str(include), str(src), "-o", str(binary)],
        check=True,
        capture_output=True,
    )
    out = subprocess.run(
        [str(binary), *(f"{q!r}" for q in PROBE_ANGLES)], check=True, capture_output=True, text=True
    ).stdout
    rows = [tuple(float(v) for v in line.split()) for line in out.splitlines()]
    assert len(rows) == len(PROBE_ANGLES)
    return dict(zip(PROBE_ANGLES, rows))


@pytest.fixture(scope="module")
def profile_linkage():
    from robot_contract import RobotProfile

    return RobotProfile.load("a200_0553").gripper.linkage


@pytest.fixture(scope="module")
def bridge_kinematics():
    """``FingerKinematics`` from the gripper bridge, loaded straight from the script on the robot side."""
    import importlib.util
    import sys

    path = sibling(BRIDGE_RELPATH)
    name = "rg6_grip_bridge_under_test"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    # Register BEFORE executing: the bridge defines dataclasses, and ``dataclasses._is_type`` resolves a field
    # annotation through ``sys.modules[cls.__module__]``.  For a module loaded by path that entry does not exist
    # yet, and the decorator dies with ``'NoneType' object has no attribute '__dict__'``.
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
        return module.FingerKinematics(str(sibling(JSON_RELPATH)))
    finally:
        sys.modules.pop(name, None)


@pytest.mark.parametrize("q", PROBE_ANGLES)
def test_the_generated_cpp_and_the_profile_agree_on_the_width(cpp_probe, profile_linkage, q):
    assert cpp_probe[q][0] == pytest.approx(profile_linkage.width_from_angle(q), abs=1e-12)


@pytest.mark.parametrize("q", PROBE_ANGLES)
def test_the_bridge_and_the_profile_agree_on_the_width(bridge_kinematics, profile_linkage, q):
    assert bridge_kinematics.width_from_angle(q) == pytest.approx(profile_linkage.width_from_angle(q), abs=1e-12)


@pytest.mark.parametrize("q", PROBE_ANGLES)
def test_the_generated_cpp_and_the_profile_agree_on_the_inversion(cpp_probe, profile_linkage, q):
    width = profile_linkage.width_from_angle(q)
    assert cpp_probe[q][1] == pytest.approx(profile_linkage.angle_from_width(width), abs=1e-12)


@pytest.mark.parametrize("q", PROBE_ANGLES)
def test_the_bridge_and_the_profile_agree_on_the_inversion(bridge_kinematics, profile_linkage, q):
    width = profile_linkage.width_from_angle(q)
    assert bridge_kinematics.angle_from_width(width) == pytest.approx(profile_linkage.angle_from_width(width), 1e-12)


def test_an_angle_beyond_the_stop_yields_the_stop_not_an_extrapolation(profile_linkage):
    """A command past the mechanical stop means 'as far as it goes', not a width the hardware cannot reach."""
    assert profile_linkage.width_from_angle(-5.0) == pytest.approx(profile_linkage.max_width_m)
    # ``approx`` and not ``==``: the clamp runs through the same interpolation with t = 1, so the upper end comes
    # back with a float remainder (0.00039900000000000005 against 0.000399).  A tenth of a picometre is not the
    # behaviour under test -- extrapolating past the stop would be.
    assert profile_linkage.width_from_angle(5.0) == pytest.approx(profile_linkage.min_width_m)


def test_a_width_beyond_the_stroke_yields_the_stop_not_nan(profile_linkage):
    assert profile_linkage.angle_from_width(10.0) == pytest.approx(profile_linkage.open_rad)
    assert profile_linkage.angle_from_width(-1.0) == pytest.approx(profile_linkage.closed_rad)
