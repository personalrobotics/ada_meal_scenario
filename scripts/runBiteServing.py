#!/usr/bin/env python

import adapy, argparse, logging, numpy, os, openravepy, prpy, rospy, rospkg, time, math
import numpy as np
from catkin.find_in_workspaces import find_in_workspaces
from std_msgs.msg import String

from prpy.planning.base import PlanningError
from prpy.tsr.rodrigues import rodrigues

from ada_meal_scenario.actions.bite_serving import BiteServing
from ada_meal_scenario.actions.bypassable_action import ActionException
from ada_meal_scenario.gui_handler import *

from tf.transformations import *

import warnings
warnings.simplefilter(action = "ignore", category = FutureWarning)

project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)

import IPython

def setup(sim=False, viewer=None, debug=True):
    # load the robot and environment for meal serving

    # Initialize logging
    if debug:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
    else:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()
    prpy.logger.initialize_logging()

    # Load the environment and robot
    env, robot = adapy.initialize(attach_viewer=viewer, sim=sim)

    # Set the active manipulator on the robot
    robot.arm.SetActive()

    # Now set everything to the right location in the environment (origin bottom left corner of table, on the floor)
    
    # some measurements that are useful
    tabletop = 0.021+0.7135+0.0127
    robotheight = 0.795
    
    # table
    table = env.ReadKinBodyURI('objects/table.kinbody.xml')
    Rx = rotation_matrix(math.pi/2.0, [1, 0, 0])
    Tr = translation_matrix([0.92575, 0.0, -0.3705])
    T = np.dot(Rx, Tr)
    table.SetTransform(T)
    env.Add(table)
    
    # move robot
    Tr = translation_matrix([0.8763, 0.1143, robotheight])
    Rz = rotation_matrix(math.pi, [0, 0, 1])
    T = np.dot(Tr, Rz)
    with env:
        robot.SetTransform(T)

    # plate (w.r.t. robot)
    plate = env.ReadKinBodyURI('objects/plastic_plate.kinbody.xml')
    T = translation_matrix([0.55,0.30,tabletop])
    plate.SetTransform(T)
    env.Add(plate)

    # Set the robot joint configurations
    ResetTrial(robot)

    load_fork_and_tool(env, robot)

    # add boxes for constraint to not hit user
    #AddConstraintBoxes(env, robot)
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
    fork_box = make_collision_box_body(fork, add_to_pos=np.array([0.0, 0.0, 0.05]), add_to_extents=np.array([0.02, 0.02, 0.1]))
    tool_box = make_collision_box_body(tool, add_to_pos=np.array([0.0, 0.0, 0.04]), add_to_extents=np.array([0.055, 0.055, 0.055]))

    #find all finger links
    finger_link_inds = []
    grab_link = None
    for ind,link in enumerate(robot.GetLinks()):
        if 'inger' in link.GetName():
            finger_link_inds.append(ind)
        if 'end_effector' in link.GetName():
            grab_link = link

    robot.arm.hand.CloseHand(1.2)

    # grab the tool and fork, tell collision checker to ignore collisions with fingers
    robot.Grab(tool, grablink=grab_link, linkstoignore=finger_link_inds)
    robot.Grab(fork, grablink=grab_link, linkstoignore=finger_link_inds)
    robot.Grab(tool_box, grablink=grab_link, linkstoignore=finger_link_inds)
    robot.Grab(fork_box, grablink=grab_link, linkstoignore=finger_link_inds)


def make_collision_box_body(kinbody, add_to_pos=np.array([0.0, 0.0, 0.0]), add_to_extents = np.array([0.0, 0.0, 0.0])):
  """Creates a box at position of kin body with optionally modified values
  @param kinbody body we want to create a box around
  @param env openrave environment
  @param add_to_pos modifications to the position of the axis aligned bounding box
  @param add_to_extents modifications to the extents of the axis aligned bounding box
  @return kinbody of box object
  """
  env = kinbody.GetEnv()

  with env:
    old_transform_body = kinbody.GetTransform()
    kinbody.SetTransform(np.eye(4))

    
    kinbody_aabb = kinbody.ComputeAABB()
    box_aabb_arr = np.append(kinbody_aabb.pos()+add_to_pos, kinbody_aabb.extents()+add_to_extents)
    box_aabb_arr = box_aabb_arr.reshape(1,6)
    #print 'aabb box: ' + str(box_aabb_arr)

    box_body = openravepy.RaveCreateKinBody(env, '')
    box_body.SetName(kinbody.GetName() + '_box')
    box_body.InitFromBoxes(box_aabb_arr, False)

    env.Add(box_body)

    kinbody.SetTransform(old_transform_body)
    box_body.SetTransform(old_transform_body)
    
    box_body.Enable(True)

  return box_body

def AddConstraintBoxes(env, robot, handedness='right', name_base="constraint_boxes_", visible=False):
    # Modifies environment to keep the robot inside a defined space
    # Does so by adding invisible boxes around the robot, which planners
    # avoid collisions with

    #add a box behind the robot
    box_behind = openravepy.RaveCreateKinBody(env,'')
    box_behind.SetName(name_base + 'behind')
    box_behind.InitFromBoxes(np.array([[0.,0.,0., 0.4, 0.1, 1.0]]), False)
    env.Add(box_behind)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    T[1,3] = 0.57
    if handedness == 'right':
        T[0,3] += 0.25
    else:
        T[0,3] -= 0.25
    box_behind.SetTransform(T)


    #add a box above so we don't swing that way too high
    box_above = openravepy.RaveCreateKinBody(env,'')
    box_above.SetName(name_base + 'above')
    box_above.InitFromBoxes(np.array([[0.,0.,0., 0.5, 0.5, 0.1]]), visible)
    env.Add(box_above)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    T[0,3] += 0.25
    T[1,3] -= 0.25
    T[2,3] += 0.90
    box_above.SetTransform(T)


    box_left = openravepy.RaveCreateKinBody(env,'')
    box_left.SetName(name_base + 'left')
    box_left.InitFromBoxes(np.array([[0.,0.,0., 0.1, 0.5, 1.0]]), visible)
    env.Add(box_left)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    if handedness == 'right':
        T[0,3] += 0.9
    else:
        T[0,3] += 0.25
    T[1,3] = 0.25
    box_left.SetTransform(T)

    box_right = openravepy.RaveCreateKinBody(env,'')
    box_right.SetName(name_base + 'right')
    box_right.InitFromBoxes(np.array([[0.,0.,0., 0.1, 0.5, 1.0]]), visible)
    env.Add(box_right)
    T = np.eye(4)
    T[0:3,3] = robot.GetTransform()[0:3,3]
    if handedness == 'right':
        T[0,3] -= 0.25
    else:
        T[0,3] -= 0.9
    T[1,3] = 0.25
    box_right.SetTransform(T)
    



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
        elif gui_return['scan']:
            
            gui_trial_starting_event.set()
            
            # go to scanning pose
            plate = env.GetKinBody('plastic_plate')
            scan_height = 0.20
            scan_angle = 10.0/180.0*math.pi
            scan_radius = math.tan(scan_angle)*scan_height
            start_angle = math.radians(-90.0)
            stop_angle = math.radians(90.0)
            num = 4
            
            # generate a workspace trajectory
            traj = openravepy.RaveCreateTrajectory(env, '')
            spec = openravepy.IkParameterization.GetConfigurationSpecificationFromType(
                    openravepy.IkParameterizationType.Transform6D, 'linear')
            traj.Init(spec)
            plate_pose = plate.GetTransformPose()
            
            # for debugging
            import tf
            br = tf.TransformBroadcaster()
            
            manip = robot.GetActiveManipulator()
            T = manip.GetEndEffectorTransform()
            z = plate_pose[6] + scan_height
            for theta in np.linspace(start_angle, stop_angle, num):
                x = plate_pose[4] + math.sin(theta)*scan_radius
                y = plate_pose[5] + math.cos(theta)*scan_radius

                T = translation_matrix([x,y,z])
                #new_z = np.array([math.sin(theta)*scan_radius, math.cos(theta)*scan_radius, scan_height])
                #old_z = np.array([0,0,1])
                #rot_ax = np.cross(old_z, new_z)
                #rot_ang = np.dot(old_z, new_z)
                #RT = rotation_matrix(angle=rot_ang, direction=rot_ax)
                #TT = translation_matrix([x,y,z])
                #T[0,3] = x
                #T[1,3] = y
                #T[2,3] = z
                next_T = T#np.dot(TT, RT)
                
                # debug, broadcast target frame.
                br.sendTransform(translation_from_matrix(next_T),
                                 quaternion_from_matrix(next_T),
                                 rospy.Time.now(),
                                 'planning_target',
                                 'map')
                
                IPython.embed()
                robot.PlanToEndEffectorPose(openravepy.poseFromMatrix(next_T), execute=True)
                #traj.Insert(traj.GetNumWaypoints(), openravepy.poseFromMatrix(next_T))
            
            IPython.embed()
            # call the planner
            #max_planning_time = 10.0 # seconds
            #with robot:
            #    qtraj = robot.PlanWorkspacePath(traj, timelimit=max_planning_time, position_tolerance=0.02, angular_tolerance=math.radians(3.0))
            
            #IPython.embed()
            
            

    gui_process.terminate()

