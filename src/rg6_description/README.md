# rg6_description
URDF file of onrobot rg6 gripper based on the repository of [syuntoku14 /
fusion2urdf](https://github.com/syuntoku14/fusion2urdf.git) and the STEP file provided by [OnRobot](https://onrobot.com/en/downloads)

Note:

This fork's URDF is xacro-based and does not contain the `g_main` joint or the
`rg6_description.xacro` file from the original fusion2urdf output. The gripper
base link is `${prefix}onrobot_rg6_base_link`, defined in
`urdf/onrobot_rg6.xacro`; the finger joints and their `<mimic>` multipliers
live in `urdf/onrobot_rg6_model_macro.xacro`. The fixed transform from the UR
tool to the gripper base is `rg6_tool0_to_base` in
`urdf/clearpath_extras.urdf.xacro` (`parent arm_0_tool0` → `child
rg6_onrobot_rg6_base_link`). Adjust the `rpy` there if the adapter angle
changes.
