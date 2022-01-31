cmake_minimum_required(VERSION 2.8.3)
project(passport)

find_package(catkin_simple REQUIRED roscpp rospy tf)

find_package(Eigen3 REQUIRED)

## GeographicLib
find_package(GeographicLib REQUIRED)

add_definitions (${GeographicLib_DEFINITIONS})
include_directories(
# include  
  ${GeographicLib_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_simple()

cs_add_executable(sensors_sim src/sensors_sim.cpp)  
cs_add_executable(gps_class src/gps_class.cpp)  

target_link_libraries(sensors_sim ${GeographicLib_LIBRARIES})
target_link_libraries(gps_class ${GeographicLib_LIBRARIES})

cs_install()

cs_install_scripts(scripts/passport_script)

cs_export()
