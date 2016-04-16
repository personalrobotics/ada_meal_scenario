# ada_meal_scenario
A set of scripts for a meal serving scenario using Ada. 

water_serving.py
============================
This script enables Ada to give water to a person. First, a glass in the environment is detected and localized using April Tags. Then, the robot grasps the glass using TSRs, moves it to a sequence of pre-determined configurations, so that the human user can drink water from the glass, and then places the glass back to a pre-defined location. 


###Marshmallow demo commands:
1. Terminal 1 － roscore and controller
	- $ source devel/setup.bash
	- $ roslaunch ada_launch default_launch

2. Terminal 2 － The blue depth camera
	- $ source devel/setup.bash
	- $ roslaunch openni2_launch openni2.launch
	- unsupported ,,, red error is ok

3. Terminal 3 － Morsel detector
	- $ source devel/setup.bash
	- $ cd src/morsel/detector/
	- $ python biteserver.py structureio_settings.json verbose.json

4. Terminal 4 － Ada_meal_scenario
	- $ source devel/setup.bash
	- $ rosrun ada_meal_scenario runBiteServing.py --real --view=interactivemarker

5. Terminal 5
	- RVIZ
	
