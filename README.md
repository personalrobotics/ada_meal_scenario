# ada_meal_scenario
A set of scripts for a meal serving scenario using Ada. 


## runBiteServing.py (Marshmallow demo):

Note: in all terminals, first `source devel/setup.bash`. Each of the following commands happens in its own terminal.

1. Set up a roscore
	```bash
	roscore
	```
	
1. Make sure the Kinova USB joystick is plugged in, then launch it
	```bash
	roslaunch ada_launch kinova_joystick.launch
	```

1. Open rviz
	```bash
	rosrun rviz rviz
	```

1. Start the ADA controller
	```bash
	roslaunch ada_launch default.launch
	```

1. Make sure the depth camera is attached and powered on, then launch it
	```bash
	roslaunch openni2_launch openni2.launch
	```
	Note: The error "Unsupported color video mode" in red is ok
	
	At this point, the camera feed should be showing in rviz (subscribe to the /camera/depth/image topic).

1. Start the morsel detector
	```bash
	cd src/morsel/detector/
	python biteserver.py structureio_settings.json verbose.json
	```

1. Start the demo
	```bash
	rosrun ada_meal_scenario runBiteServing.py --real --viewer=interactivemarker --userid=99 --no-pupil-tracking
	```

	You may see a prompt that says that user 099 already exists. Press enter to continue (this is just you so don't write participant info to the wrong place, but doesn't matter for running the demo).
	
At this point, a GUI should appear that allows you to select the operation mode. Select a mode (e.g., shared autonomy) and an input device (e.g., Kinova USB), then press the start next trial button.


##water_serving.py
============================
This script enables Ada to give water to a person. First, a glass in the environment is detected and localized using April Tags. Then, the robot grasps the glass using TSRs, moves it to a sequence of pre-determined configurations, so that the human user can drink water from the glass, and then places the glass back to a pre-defined location. 

