#!/usr/bin/env python

import adapy, argparse, logging, numpy, os, openravepy, prpy, rospy
from catkin.find_in_workspaces import find_in_workspaces
from ada_meal_scenario.actions.bite_serving import BiteServing
from ada_meal_scenario.actions.bypassable_action import ActionException


project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)

def setup(sim=False, viewer=None, debug=True):

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

    # Set the active manipulator on the robot
    robot.arm.SetActive()

    if sim is True:
    	startConfig = numpy.array([  3.33066907e-16,   2.22044605e-16,   1.66608370e+00,
        -1.65549603e+00,  -1.94424475e-01,   1.06742772e+00,
        -1.65409614e+00,   1.30780704e+00])
        robot.SetDOFValues(startConfig)
    # Now set everything to the right location in the environment
    robot_pose = numpy.array([[1., 0., 0., 0.409],
                              [0., 1., 0., 0.338],
                              [0., 0., 1., 0.795],
                              [0., 0., 0., 1.]])
    with env:
        robot.SetTransform(robot_pose)

    iksolver = openravepy.RaveCreateIkSolver(env,"NloptIK")
    robot.arm.SetIKSolver(iksolver)

    # Load the fork into the robot's hand
    fork_path = os.path.join(data_base_path[0], 'objects', 'fork.kinbody.xml')
    fork = env.ReadKinBodyXMLFile(fork_path)
    env.Add(fork)
    
    # Fork in end-effector
    ee_in_world = robot.arm.GetEndEffectorTransform()
    fork_in_ee = numpy.array([[ 0., -1.,  0., -0.025],
                              [ 0.,  0., 1., 0.],
                              [ -1.,  0.,  0., -0.145],
                              [ 0.,  0.,  0., 1.]])
    fork_in_world = numpy.dot(ee_in_world, fork_in_ee)
    fork.SetTransform(fork_in_world)
    robot.Grab(fork)

    return env, robot

if __name__ == "__main__":
        
    rospy.init_node('bite_serving_scenario', anonymous=True)

    parser = argparse.ArgumentParser('Ada meal scenario')
    parser.add_argument("--debug", action="store_true", help="Run with debug")
    parser.add_argument("--real", action="store_true", help="Run on real robot (not simulation)")
    parser.add_argument("--viewer", type=str, default='qtcoin', help="The viewer to load")
    parser.add_argument("--detection-sim", action="store_true", help="Simulate detection of morsal")
    args = parser.parse_args()

    sim = not args.real
    env, robot = setup(sim=sim, viewer=args.viewer, debug=args.debug)

    from IPython import embed
    embed()
    while True:
        c = raw_input('Press enter to run (q to quit)')
        if c == 'q':
            break
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

    import IPython
    IPython.embed()
