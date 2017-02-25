cmake_minimum_required(VERSION 2.8.3)
project(ada_meal_scenario)
 
find_package(catkin REQUIRED COMPONENTS) 


catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

