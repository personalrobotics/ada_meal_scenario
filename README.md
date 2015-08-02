# ada_meal_scenario
A set of scripts for a meal serving scenario using Ada. 

Running ada_meal_scenario demo 
===========================

To run the ada_meal scenario demo, first make sure that both the ADA robot, and the StructureIO sensor are connected to the PC. Additionally, you will need to have installed the openni2 drivers of the StructureIO sensor: http://structure.io/openni

Then, create a new catkin workspace. Go to the src folder, and copy-paste the latest ``.rosinstall`` file from the /releases branch. Then, run ``wstool up`` so that you get the appropriate versions of the git repositories. Then, run ``catkin_make`` from the parent directory. 

You will then need to open four different console windows. In the first window, start the robot controller:
``roslaunch ada_launch default.launch``
In the second console window, start streaming data from the sensor. Run 
``roslaunch openni2.launch openni2_launch``
In the third window, start the bite-detection stream. To do that, go to ``/src/morsel/detector/biteserver.py`` and run
``python biteserver.py structureio_settings.json``
Finally, in the fourth window, do ``rosrun ada_meal_scenario runBiteServing.py --real`` to start the demo. The robot will go, grasp a bite and place it in a serving position. If you press ``Enter``, the robot will repeat the action sequence. Removing the ``--real`` flag will result in the demo running in simulation. 


water_serving.py
============================
This script enables Ada to give water to a person. First, a glass in the environment is detected and localized using April Tags. Then, the robot grasps the glass using TSRs, moves it to a sequence of pre-determined configurations, so that the human user can drink water from the glass, and then places the glass back to a pre-defined location. 
