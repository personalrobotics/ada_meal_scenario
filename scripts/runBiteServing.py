#!/usr/bin/env python

import adapy, argparse, logging, numpy, os, openravepy, prpy, rospy, random
from catkin.find_in_workspaces import find_in_workspaces
from ada_meal_scenario.actions.bite_serving import BiteServing
from ada_meal_scenario.actions.bypassable_action import ActionException
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import numpy as np
import IPython
from time import sleep

from prpy.tsr.rodrigues import *


project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)

def setup(sim=False, viewer=None, debug=True):
    global robot, serving_phrases

    data_base_path = find_in_workspaces(
        search_dirs=['share'],
        project=project_name,
        path='data',
        first_match_only=True)
    if len(data_base_path) == 0:
        raise Exception('Unable to find environment path. Did you source devel/setup.bash?')

    env_path = os.path.join(data_base_path[0], 'environments', 'table.env.xml')
    
    # Initialize logging
    if debug:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
    else:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()
    prpy.logger.initialize_logging()

    # Load the environment
    env, robot = adapy.initialize(attach_viewer=viewer, sim=sim, env_path=env_path)

    #TODO get this from a rosparam
    right_handed = True

    # Set the active manipulator on the robot
    robot.arm.SetActive()

    #if sim is True:
    #   startConfig = numpy.array([  3.33066907e-16,   2.22044605e-16,   1.66608370e+00,
    #    -1.65549603e+00,  -1.94424475e-01,   1.06742772e+00,
    #    -1.65409614e+00,   1.30780704e+00])
    if sim is True:
        #set configuration to look at plate if sim else plan to look at plate
        indices, values = robot.configurations.get_configuration('ada_meal_scenario_lookingAtPlateConfiguration')
        robot.SetDOFValues(dofindices=indices, values=values)
    else:
        robot.arm.PlanToNamedConfiguration('ada_meal_scenario_lookingAtPlateConfiguration')
    #    robot.SetDOFValues(startConfig)
    # Now set everything to the right location in the environment
    robot_pose = numpy.array([[1., 0., 0., 0.409],
                              [0., 1., 0., 0.338],
                              [0., 0., 1., 0.795],
                              [0., 0., 0., 1.]])
    with env:
        robot.SetTransform(robot_pose)

    # Load the fork into the robot's hand
    tool = env.ReadKinBodyURI('objects/kinova_tool.kinbody.xml')
    env.Add(tool)
    
    # Fork in end-effector
    ee_in_world = robot.GetLink('j2n6a300_link_6').GetTransform()
#    tool_in_ee = numpy.array([[ -1., 0.,  0., 0.],
#                              [ 0.,  1., 0., -0.002],
#                              [ 0.,  0.,  -1., -0.118],
#                              [ 0.,  0.,  0., 1.]])


    ee_in_world = robot.arm.GetEndEffectorTransform()
    if right_handed:
        y_trans_tool = 0.004
    else:
        y_trans_tool = -0.004
    tool_in_ee = numpy.array([[ 1., 0.,  0., 0.],
                            [ 0.,  1., 0., y_trans_tool],
                            [ 0.,  0.,  1., -0.042],
                            [ 0.,  0.,  0., 1.]])
    rotate_tool_in_ee = rodrigues([0., 0., -np.pi/32.])
    tool_in_ee[0:3, 0:3] = np.dot(rotate_tool_in_ee, tool_in_ee[0:3, 0:3])

    tool_in_world = numpy.dot(ee_in_world, tool_in_ee)
    tool.SetTransform(tool_in_world)
    
    fork = env.ReadKinBodyURI('objects/fork.kinbody.xml')
    env.Add(fork)
    fork_in_hole = numpy.array([[1.,0.,0.,0.],
                                [0.,1.,0.,0.],
                                [0.,0.,1.,-0.03],
                                [0.,0.,0.,1.]])
    hole_in_tool = numpy.array([[0.,0.,1.,0.],
                                [0.,1.,0.,-0.0225],
                                [-1.,0.,0.,0.0408],
                                [0.,0.,0.,1.]])
    fork_in_tool = numpy.dot(hole_in_tool, fork_in_hole)                           
    fork_in_ee = numpy.dot(tool_in_ee, fork_in_tool)
    fork_in_world = numpy.dot(ee_in_world, fork_in_ee)
    fork.SetTransform(fork_in_world)
    
    robot.Grab(tool)
    robot.Grab(fork)

    # Set serving phrases
    serving_phrases = ['That looks like a delicious bite ', 
                        'Here you go, I hope you enjoy it ',
                        'That was a good choice ']
    
    return env, robot

def pose_to_arrow_markers(pose, ns='axes', id_start=0, lifetime_secs=10):
  markers = []
  scale_size = 0.1
  dims = [.01, .02, 0.1]
  dirs = []
  dirs.append(np.array([1.,0.,0.]))
  dirs.append(np.array([0.,1.,0.]))
  dirs.append(np.array([0.,0.,1.]))
  for ind,dir in enumerate(dirs):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.ARROW
    marker.action = Marker.ADD;
    marker.id = id_start + ind
    marker.lifetime.secs=lifetime_secs
    marker.ns = ns

    marker.scale.x = dims[0]
    marker.scale.y = dims[1]
    marker.scale.z = dims[2]

    marker.pose.position.x = 0.
    marker.pose.position.y = 0.
    marker.pose.position.z = 0.
    marker.pose.orientation.x = 0.
    marker.pose.orientation.y = 0.
    marker.pose.orientation.z = 0.
    marker.pose.orientation.w = 1.

 

    start_pt = pose[0:3,3]
    end_pt = np.dot(pose[0:3,0:3], dir)*scale_size + start_pt
    marker.points = [np_array_to_point(start_pt), np_array_to_point(end_pt)]

    marker.color.r = dir[0]
    marker.color.g = dir[1]
    marker.color.b = dir[2]
    marker.color.a = 1.0

    markers.append(marker)

  return markers

def np_array_to_point(pt_np):
  pt = Point()
  pt.x = pt_np[0]
  pt.y = pt_np[1]
  pt.z = pt_np[2]
  return pt

def joystick_callback(data):
    global joystick_go_signal
    # If the left button is pressed, update go signal flag
    if data.buttons[0]:
        joystick_go_signal = True

def tasks_callback(data):
    global serving_phrases
    # Announce the bite!
    if data.data == 'SERVING':
        # Randomly select one of the serving phrases
        phrase = random.choice(serving_phrases) + username
        robot.Say(phrase)


if __name__ == "__main__":
    global joystick_go_signal, robot, username
        
    rospy.init_node('bite_serving_scenario', anonymous=True)

    parser = argparse.ArgumentParser('Ada meal scenario')
    parser.add_argument("--debug", action="store_true", help="Run with debug")
    parser.add_argument("--real", action="store_true", help="Run on real robot (not simulation)")
    parser.add_argument("--viewer", type=str, default='interactivemarker', help="The viewer to load")
    parser.add_argument("--detection-sim", action="store_true", help="Simulate detection of morsal")
    parser.add_argument("--interaction", default='keyboard', help="The type of interaction (joystick, keyboard, username)")
    args = parser.parse_args(rospy.myargv()[1:]) # exclude roslaunch args

    sim = not args.real
    env, robot = setup(sim=sim, viewer=args.viewer, debug=args.debug)
    interaction_mode = args.interaction


    #slow robot down
    #save old limits
    old_acceleration_limits = robot.GetDOFAccelerationLimits()
    old_velocity_limits = robot.GetDOFVelocityLimits()

    #slow down robot
    #robot.SetDOFVelocityLimits(0.6*robot.GetDOFVelocityLimits())
    #robot.SetDOFAccelerationLimits(0.8*robot.GetDOFAccelerationLimits())

    robot.SetDOFVelocityLimits(old_velocity_limits)
    robot.SetDOFAccelerationLimits(old_acceleration_limits)
 

    #from IPython import embed
    #embed()

    #start by going to ada_meal_scenario_servingConfiguration
    if sim:
        indices, values = robot.configurations.get_configuration('ada_meal_scenario_servingConfiguration')
        robot.SetDOFValues(dofindices=indices, values=values)
    else:
        robot.PlanToNamedConfiguration('ada_meal_scenario_servingConfiguration', execute=True)

#    camera_link = robot.GetLink('Camera_RGB_Frame')
#    camera_transform = camera_link.GetTransform()
#    with prpy.viz.RenderPoses([camera_transform], env):

    # Subscribe to the 'ada_tasks' topic (for talking during certain tasks)
    task_listener = rospy.Subscriber('ada_tasks', String, tasks_callback)

    # If input device is joystick, create subscriber to joystick topic
    if interaction_mode == 'joystick' or interaction_mode == 'username':
        joy_listener = rospy.Subscriber('/ada/joy', Joy, joystick_callback)
        joystick_go_signal = False

    while True:

        # Wait for a go signal based on the input device (joystick or keyboard)
        if interaction_mode == 'keyboard':
            c = raw_input('Press enter to run (q to quit)')
            if c == 'q':
                break
        elif interaction_mode == 'joystick':
            rospy.logwarn('Press left joystick button to continue (Ctrl+C to quit)')
            while not joystick_go_signal:
                sleep(0.5)
        elif interaction_mode == 'username':
            username = raw_input("Enter the user's name: ")
            rospy.logwarn('Press left joystick button to continue (Ctrl+C to quit)')
            while not joystick_go_signal:
                sleep(0.5)
        else:
            rospy.logerr("Invalid input device (should be 'joystick' or 'keyboard'): "
                        % interaction_mode)
            break

        # Start bite collection and presentation
        try:
            manip = robot.GetActiveManipulator()
            action = BiteServing()
            action.execute(manip, env, detection_sim=args.detection_sim)
        except ActionException, e:
            logger.info('Failed to complete bite serving: %s' % str(e))
        finally:
            morsal = env.GetKinBody('morsal')
            if morsal is not None:
                logger.info('Removing morsal from environment')
                env.Remove(morsal)

            if interaction_mode == 'joystick' or interaction_mode == 'username':
                joystick_go_signal = False

    #restore old limits
    robot.SetDOFVelocityLimits(old_velocity_limits)
    robot.SetDOFAccelerationLimits(old_acceleration_limits)

    import IPython
    IPython.embed()
