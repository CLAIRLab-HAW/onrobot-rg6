# rg6_description
URDF file of onrobot rg6 gripper based on the repository of [syuntoku14 /
fusion2urdf](https://github.com/syuntoku14/fusion2urdf.git) and the STEP file provided by [OnRobot](https://onrobot.com/en/downloads)

Note:

This fork's URDF is xacro-based and does not contain the `g_main` joint or the
`rg6_description.xacro` file from the original fusion2urdf output. The gripper
itself — links, finger joints and their `<mimic>` multipliers — comes from the
upstream rg6_v2 model in `urdf/onrobot_rg_upstream.urdf.xacro` (see
[LICENSE-THIRD-PARTY](../../LICENSE-THIRD-PARTY.md)). What this workspace adds
sits in `urdf/clearpath_extras.urdf.xacro`: the base link
`rg6_onrobot_rg6_base_link`, the fixed transform from the UR tool to it
(`rg6_tool0_to_base`, `parent arm_0_tool0` → `child
rg6_onrobot_rg6_base_link`) and the TCP frame. Adjust the `rpy` there if the
adapter angle changes.
