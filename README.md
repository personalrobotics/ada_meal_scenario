# ada_meal_scenario
A set of scripts for a meal serving scenario using Ada. 

water_serving.py
============================
This script enables Ada to give water to a person. First, a glass in the environment is detected and localized using April Tags. Then, the robot grasps the glass using TSRs, moves it to a sequence of pre-determined configurations, so that the human user can drink water from the glass, and then places the glass back to a pre-defined location. 


Marshmallow demo commands:
Terminal 1
	1. $ source devel/setup.bash
	2. $ roslaunch ada_launch default_launch

Terminal 2
The blue depth camera
	1. $ source devel/setup.bash
	2. $ roslaunch openni2_launch openni2.launch
(unsupported ,,, red error is ok)

Terminal 3
Morsel detector
	1. $ source devel/setup.bash
	2. $ cd src/morsel/detector/
	3. $ python biteserver.py structureio_settings.json verbose.json

Terminal 4
Ada_meal_scenario
	1. $ source devel/setup.bash
	2. $ rosrun ada_meal_scenario runBiteServing.py --real --view=interactivemarker

Terminal 5
RVIZ
	
