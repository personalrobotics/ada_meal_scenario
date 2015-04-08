import logging, prpy, os
import prpy.rave, prpy.util
from bypassable_action import BypassableAction

project_name = 'ada_meal_scenario'
logger = logging.getLogger(project_name)

class RunTrajectory(BypassableAction):

    def __init__(self, name, trajfile, bypass=False):
        """
        @param name The name of the action
        @param trajfile The full path ato the trajectory file
        @param bypass If true, move world to a configuration
          that represents the end of the action
        """
        BypassableAction.__init__(self, name, bypass=bypass)
        self.traj = None
        self.trajfile = trajfile

    def _load_traj(self, env):
        self.traj = prpy.rave.load_trajectory(env, self.trajfile)
        
    def _run(self, robot):
        
        if self.traj is None:
            self._load_traj(robot.GetEnv())

        logger.info('Executing trajectory for action %s' % self.name)
        robot.ExecuteTrajectory(self.traj)

    def _bypass(self, robot):
        
        if self.traj is None:
            self._load_traj(robot.GetEnv())

        cspec = self.traj.GetConfigurationSpecification()
        last_wpt = self.traj.GetWaypoint(self.traj.GetNumWaypoints()-1)

        dofindices = prpy.util.GetTrajectoryIndices(self.traj)
        dofvalues = cspec.ExtractJointValues(last_wpt)
        
        with robot.GetEnv():
            robot.SetDOFValues(values=dofvalues, dofindices=dofindices)
        

class LookAtFace(RunTrajectory):
    
    def __init__(self, bypass=False):
                
        from catkin.find_in_workspaces import find_in_workspaces
        traj_path = find_in_workspaces(
            search_dirs=['share'],
            project=project_name,
            path='data',
            first_match_only=True)
        if len(traj_path) == 0:
            raise ActionException(self, 'Failed to find trajectory data path')
        
        trajfile = os.path.join(traj_path[0], 'trajectories', 'traj_lookingAtFace.xml')

        RunTrajectory.__init__(self, "LOOKING_AT_FACE", trajfile, bypass=bypass)

    
class LookAtPlate(RunTrajectory):

    def __init__(self, bypass=False):
        
        from catkin.find_in_workspaces import find_in_workspaces
        traj_path = find_in_workspaces(
            search_dirs=['share'],
            project=project_name,
            path='data',
            first_match_only=True)
        if len(traj_path) == 0:
            raise ActionException(self, 'Failed to find trajectory data path')
        
        trajfile = os.path.join(traj_path[0], 'trajectories', 'traj_lookingAtPlate.xml')

        RunTrajectory.__init__(self, "LOOKING_AT_PLATE", trajfile, bypass=bypass)

class Serve(RunTrajectory):

    def __init__(self, bypass=False):
        
        from catkin.find_in_workspaces import find_in_workspaces
        traj_path = find_in_workspaces(
            search_dirs=['share'],
            project=project_name,
            path='data',
            first_match_only=True)
        if len(traj_path) == 0:
            raise ActionException(self, 'Failed to find trajectory data path')
        
        trajfile = os.path.join(traj_path[0], 'trajectories', 'traj_serving.xml')

        RunTrajectory.__init__(self, "SERVING", trajfile, bypass=bypass)

