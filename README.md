# ada_meal_scenario
A set of scripts for a meal serving scenario using Ada. 


## runBiteServing.py (Marshmallow demo):

There are two ways to run this demo. The first uses a ROS launch file, and should work for most cases. If you need additional access to arguments other than `--real` and `--detection-sim`, use the second option which involves launching all associated files separately. 

### Option 1: roslaunch

```
roslaunch ada_meal_scenario biteserving.launch real:=true
```

Omit the ```real:=true``` argument to run in simulation. 


### Option 2: individual file launches
Note: in all terminals, first `source devel/setup.bash`.

1. Terminal 1 － ADA controller
	```
	$ roslaunch ada_launch default.launch
	```

2. Terminal 2 － The depth camera
	```
	$ roslaunch openni2_launch openni2.launch
	```
	Note: The error "Unsupported color video mode" in red is ok

3. Terminal 3 － Morsel detector
	```
	$ cd src/morsel/detector/
	$ python biteserver.py structureio_settings.json verbose.json
	```

4. Terminal 4 － Ada_meal_scenario
	```
	$ rosrun ada_meal_scenario runBiteServing.py --real --viewer=interactivemarker
	```

5. Terminal 5 - rviz
	```
	$ rosrun rviz rviz
	```
	



##water_serving.py
============================
This script enables Ada to give water to a person. First, a glass in the environment is detected and localized using April Tags. Then, the robot grasps the glass using TSRs, moves it to a sequence of pre-determined configurations, so that the human user can drink water from the glass, and then places the glass back to a pre-defined location. 

