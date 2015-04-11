#!/usr/bin/env python

import adapy, argparse, logging, numpy, os, openravepy, prpy, rospy
from catkin.find_in_workspaces import find_in_workspaces
from runBiteServing import setup
from prpy.planning import PlanningError

project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)

def save_path(path, filename):
    with open(filename, 'w') as f:
        path_str = path.serialize()
        f.write(path_str)

    logger.info('Saved path to file: %s' % filename)

if __name__ == "__main__":
        
    rospy.init_node('bite_serving_scenario', anonymous=True)

    parser = argparse.ArgumentParser('Ada meal scenario')
    parser.add_argument("--debug", action="store_true", help="Run with debug")
    parser.add_argument("--real", action="store_true", help="Run on real robot (not simulation)")
    parser.add_argument("--viewer", type=str, default='qtcoin', help="The viewer to load")
    args = parser.parse_args()

    sim = not args.real
    env, robot = setup(sim=sim, viewer=args.viewer, debug=args.debug)

    try:
        path_to_start = robot.PlanToNamedConfiguration('ada_meal_scenario_start')
        path_to_looking_at_face = robot.PlanToNamedConfiguration('ada_meal_scenario_lookingAtFaceConfiguration')    
        save_path(path_to_looking_at_face, 'traj_lookingAtFace.xml')
        
        path_to_looking_at_plate = robot.PlanToNamedConfiguration('ada_meal_scenario_lookingAtPlateCOnfiguration')
        save_path(path_to_looking_at_plate, 'traj_lookingAtPlate.xml')
        
        path_to_serving_configuration = robot.PlanToNamedConfiguration('ada_meal_scenario_servingConfiguration')
        save_path(path_to_serving_configuration, 'traj_serving.xml')
    except PlanningError, e:
        logger.error('Failed to plan: %s' % str(e))
    
    import IPython
    IPython.embed()
