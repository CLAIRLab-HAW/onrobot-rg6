# rg6_description
URDF file of onrobot rg6 gripper based on the repository of [syuntoku14 /
fusion2urdf](https://github.com/syuntoku14/fusion2urdf.git) and the STEP file provided by [OnRobot](https://onrobot.com/en/downloads)

Note:

If the default angle between the adapter and the main body of the gripper has been changed by the user, update rpy under the joint g_main in rg6_description.xacro:

```xml
<joint name="g_main" type="fixed">
  <origin xyz="-0.031849 1e-06 0.04953" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="g_main_1"/>
</joint>
```
