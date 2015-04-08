import argparse, logging, numpy, os, openravepy
from catkin.find_in_workspaces import find_in_workspaces

project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)

def setup(sim=False, viewer=None, debug=True):

    env_base_path = find_in_workspaces(
        search_dirs=['share'],
        project=project_name,
        path='environments',
        first_match_only=True)[0]
    env_path = os.join(env_base_path, 'table.env.xml')
    
    # Initialize logging
    if debug:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
    else:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()

    # Load the environment
    env, robot = adapy.initialize(attach_viewer=viewer, sim=sim, env_path=env_path)

    # Set the active manipulator on the robot
    robot.SetActiveManipulator('Mico')

    # Now set everything to the right location in the environment
    robot_pose = numpy.array([[1., 0., 0., 0.409],
                              [0., 1., 0., 0.338],
                              [0., 0., 1., 0.795],
                              [0., 0., 0., 1.]])
    with env:
        robot.SetTransform(robot_pose)

    # Add a ball - TODO this should be done in the bite detection callback
    object_base_path = find_in_workspaces(
        search_dirs=['share'],
        project=project_name,
        path='objects',
        first_match_only=True)[0]
    ball_path = os.path.join(object_base_path, 'smallsphere.kinbody.xml')
    ball = env.ReadKinBodyURI(ball_path)
    env.Add(ball)

    return env, robot

if __name__ == "__main__":
        
    parser, args = task_helper.setupArgs()
    parser.add_argument("--debug", action="store_true", help="Run with debug")
    parser.add_argument("--real", action="store_true", help="Run on real robot (not simulation)")
    parser.add_argument("--viewer", type=str, default='qtcoin', help="The viewer to load")
    args = parser.parse_args()

    sim = not args.real
    env, robot = setup(sim=sim, viewer=args.viewer, debug=args.debug)
