#!/usr/bin/env python

## TODO: separate file into component pieces, because it's quite long now and contains many general pieces


import adapy, argparse, logging, numpy, os, openravepy, prpy, rospy, rospkg, time
import numpy as np
from catkin.find_in_workspaces import find_in_workspaces
from std_msgs.msg import String

from prpy.planning.base import PlanningError
from prpy.tsr.rodrigues import rodrigues

from ada_meal_scenario.actions.bite_serving import BiteServing
from ada_meal_scenario.actions.bypassable_action import ActionException
from ada_meal_scenario.gui_handler import *
from ada_meal_scenario.add_constraints import AddConstraints

import warnings
#warnings.simplefilter(action = "ignore", category = FutureWarning)

project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)
constraints = AddConstraints()

def setup(sim=False, viewer=None, debug=True):
    # load the robot and environment for meal serving

    # find the openrave environment file
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

    # Load the environment and robot
    env, robot = adapy.initialize(attach_viewer=viewer, sim=sim, env_path=env_path)

    # Set the active manipulator on the robot
    robot.arm.SetActive()

    # Now set everything to the right location in the environment
    #if using jaco, assume we have the portable mount, and robot should have a different distance to table
    using_jaco = robot.GetName() == 'JACO'
    if using_jaco:
      robot_pose = numpy.array([[1., 0., 0., 0.409],
                              [0., 1., 0., 0.338],
                              [0., 0., 1., 0.754],
                              [0., 0., 0., 1.]])
    else:
      robot_pose = numpy.array([[1., 0., 0., 0.409],
                              [0., 1., 0., 0.338],
                              [0., 0., 1., 0.795],
                              [0., 0., 0., 1.]])

    with env:
        robot.SetTransform(robot_pose)

    # Set the robot joint configurations
    ResetTrial(robot)

    load_fork_and_tool(env, robot)

    # add boxes for constraint to not hit user
    constraints.AddConstraintBoxes(env, robot)
    return env, robot

def load_fork_and_tool(env, robot):
    # Loads the fork and kinova tool holder into the environment, 
    # sets up transforms, and has robot grab the tool
    # also adds objects on those tools, because collision checks with the 
    # meshes were not working properly


    tool = env.ReadKinBodyURI('objects/kinova_tool.kinbody.xml')
    env.Add(tool)
    
    #set the tool and fork transforms
    ee_in_world = robot.arm.GetEndEffectorTransform()
    y_trans_tool = 0.004  #should flip this if we try left_handed
    tool_in_ee = numpy.array([[ 1., 0.,  0., 0.],
                            [ 0.,  1., 0., y_trans_tool],
                            [ 0.,  0.,  1., -0.042],
                            [ 0.,  0.,  0., 1.]])
    rotate_tool_in_ee = rodrigues([0., 0., np.pi/32.])
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


    # since there were problems with the meshes and collision checking, 
    # these boxes were added around the objects. They help make sure 
    # the robot avoids collisions of the fork while it is grabbed

    fork_box = constraints.make_collision_box_body(fork, add_to_pos=np.array([0.0, 0.0, 0.05]), add_to_extents=np.array([0.02, 0.02, 0.1]))
    tool_box = constraints.make_collision_box_body(tool, add_to_pos=np.array([0.0, 0.0, 0.04]), add_to_extents=np.array([0.055, 0.055, 0.055]))

    #find all finger links
    finger_link_inds = []
    grab_link = None
    for ind,link in enumerate(robot.GetLinks()): 
        if 'finger' in link.GetName().lower():
            finger_link_inds.append(ind)
        if 'end_effector' in link.GetName().lower():
            grab_link = link

    robot.arm.hand.CloseHand(1.2)

    # grab the tool and fork, tell collision checker to ignore collisions with fingers
    robot.Grab(tool, grablink=grab_link, linkstoignore=finger_link_inds)
    robot.Grab(fork, grablink=grab_link, linkstoignore=finger_link_inds)
    robot.Grab(tool_box, grablink=grab_link, linkstoignore=finger_link_inds)
    robot.Grab(fork_box, grablink=grab_link, linkstoignore=finger_link_inds)



def ResetTrial(robot):
  # set the robot to the start configuration for next trial
  logger.info('Resetting Robot')
  if robot.simulated:
      indices, values = robot.configurations.get_configuration('ada_meal_scenario_servingConfiguration')
      robot.SetDOFValues(dofindices=indices, values=values)
  else:
    #first try to plan to serving
    try:
      robot.PlanToNamedConfiguration('ada_meal_scenario_servingConfiguration', execute=True)
    except PlanningError, e:
      logger.info('Failed to plan to start config')
      #if it doesn't work, unload controllers


def setup_trial_recording(record_next_trial, file_directory_user):
    # creates user directory if we will be recording
    if record_next_trial and not os.path.exists(file_directory_user):
        os.makedirs(file_directory_user)
    

if __name__ == "__main__":
    state_pub = rospy.Publisher('ada_tasks',String, queue_size=10)
        
    rospy.init_node('bite_serving_scenario', anonymous=True)

    parser = argparse.ArgumentParser('Ada meal scenario')
    parser.add_argument("--debug", action="store_true", help="Run with debug")
    parser.add_argument("--real", action="store_true", help="Run on real robot (not simulation)")
    parser.add_argument("--viewer", type=str, default='interactivemarker', help="The viewer to load")
    parser.add_argument("--detection-sim", action="store_true", help="Simulate detection of morsel")
    parser.add_argument("--userid", type=int, help="User ID number")
    args = parser.parse_args(rospy.myargv()[1:]) # exclude roslaunch args

    sim = not args.real
    env, robot = setup(sim=sim, viewer=args.viewer, debug=args.debug)

    gui_get_event, gui_trial_starting_event, gui_queue, gui_process = start_gui_process()

    # Where to store rosbags and other user data - set this manually if userid was provided,
    # otherwise dynamically generate it as one more than highest in directory
    file_directory_user = None
    file_directory = rospkg.RosPack().get_path('ada_meal_scenario') + '/trajectory_data'
    if args.userid:
        from ada_teleoperation.DataRecordingUtils import get_filename
        file_directory_user = get_filename(file_directory=file_directory, filename_base='user_', file_ind=args.userid, file_type="")
        # Check whether the file_directory_user exists
        if os.path.exists(file_directory_user):
            inp = raw_input("Filename " + file_directory_user + " exists. Press q to quit or enter to continue")
            if inp == 'q':
                raise OSError("Filename already exists. Quitting.")
    else:
        from ada_teleoperation.DataRecordingUtils import get_next_available_user_ind
        user_number, file_directory_user = get_next_available_user_ind(file_directory=file_directory, make_dir=False)

    while True:
        #signal to gui that we want the currently selected options
        empty_queue(gui_queue)
        gui_get_event.set()
        while gui_queue.empty():
            time.sleep(0.05)
            continue

        gui_return = gui_queue.get()

        if gui_return['quit']:
            break
        elif gui_return['start']:
            #tell gui we are starting to reset next trial
            gui_trial_starting_event.set()
            # Start bite collection and presentation
            try:
                manip = robot.GetActiveManipulator()
                action = BiteServing()
                setup_trial_recording(gui_return['record'], file_directory_user)
                action.execute(manip, env, method=gui_return['method'], ui_device=gui_return['ui_device'], state_pub=state_pub, detection_sim=args.detection_sim, record_trial=gui_return['record'], file_directory=file_directory_user)
            except ActionException, e:
                logger.info('Failed to complete bite serving: %s' % str(e))

                ResetTrial(robot)


    gui_process.terminate()

