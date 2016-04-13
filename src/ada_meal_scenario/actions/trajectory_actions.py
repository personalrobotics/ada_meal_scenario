import logging, numpy, prpy, os
import prpy.rave, prpy.util
from bypassable_action import ActionException, BypassableAction
from prpy.planning.base import PlanningError
from prpy.exceptions import TrajectoryNotExecutable
import time

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
        
    def _run(self, manip):
        
        robot = manip.GetRobot()
        env = robot.GetEnv()

        if self.traj is None:
            self._load_traj(env)



        cspec = self.traj.GetConfigurationSpecification()
        first_wpt = self.traj.GetWaypoint(0)
        first_config = cspec.ExtractJointValues(first_wpt, robot, manip.GetArmIndices())
        current_config = manip.GetDOFValues()

        fork = env.GetKinBody('fork')
        
        # Plan to the start of the trajectory if we aren't already there
        #if numpy.linalg.norm(first_config - current_config) > 0.001: # TODO: What is the right epsilon
        if not prpy.util.IsAtTrajectoryStart(robot, self.traj):
            logger.info('Planning to start of trajectory for action %s' % self.name)
            print 'Planning to start of trajectory for action ' + self.name
            try:
                #from IPython import embed
                #embed()
                with prpy.rave.Disabled(fork):
                   robot.PlanToConfiguration(first_config, execute=True)
            except PlanningError, e:
                raise ActionException(self, 'Failed to plan to start of trajectory: %s' % str(e))

        # logger.info('Executing trajectory for action %s. Num points: %d' % (self.name, self.traj.GetNumWaypoints()))
        # SHERVIN removed smoothing because saved trajectories now smooth
        #import openravepy
        #openravepy.planningutils.SmoothTrajectory(self.traj,1, 1, 'ParabolicSmoother', '')

        try:
            robot.ExecuteTrajectory(self.traj)
        except e:
            print str(e)
            print 'Executing saved traj failed. Trying to replan to config'
            last_wpt = self.traj.GetWaypoint(self.traj.GetNumWaypoints()-1)
            last_config = cspec.ExtractJointValues(last_wpt, robot, manip.GetArmIndices())
            robot.PlanToConfiguration(last_config, execute=True)
            
        #time.sleep(3)

    def _bypass(self, manip):
        
        robot = manip.GetRobot()
        env = robot.GetEnv()
        if self.traj is None:
            self._load_traj(env)

        cspec = self.traj.GetConfigurationSpecification()
        last_wpt = self.traj.GetWaypoint(self.traj.GetNumWaypoints()-1)

        dofindices = prpy.util.GetTrajectoryIndices(self.traj)
        dofvalues = cspec.ExtractJointValues(last_wpt, robot, manip.GetArmIndices())
        
        with env:
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
        #looking at plate traj
        #starts at: ada_meal_scenario_servingConfiguration
        #end at: ada_meal_scenario_lookingAtPlateConfiguration
        
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

