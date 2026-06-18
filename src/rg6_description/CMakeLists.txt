cmake_minimum_required(VERSION 3.5)
project(rg6_description)

find_package(ament_cmake REQUIRED)

# rg6_description haelt jetzt das Greifermodell (urdf + meshes) UND die
# clearpath_extras-Glue-Datei (das fruehere onrobot_rg6_visualization wurde hierher
# gemergt; rg6_control bleibt separat).
install(
  DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
