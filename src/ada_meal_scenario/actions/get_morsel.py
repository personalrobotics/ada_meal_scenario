import numpy, prpy.viz
from bypassable_action import ActionException, BypassableAction
from prpy.planning.base import PlanningError
import time
import openravepy

from prpy.ik_ranking import MultipleNominalConfigurations

from assistance_policy_action import AssistancePolicyAction
from direct_teleop_action import DirectTeleopAction

from detect_morsel import morsel_index_to_name

import rospkg

import logging
logger = logging.getLogger('ada_meal_scenario')

import yaml, os

def get_all_morsel_bodies(env):
  all_bodies = []
  for i in range(1000):
        morsel_name = morsel_index_to_name(i)
        morsel_body = env.GetKinBody(morsel_name)
        if morsel_body:
            all_bodies.append(morsel_body)
        else:
            break
  return all_bodies


class GetMorsel(BypassableAction):

    def __init__(self, bypass=False):
        
        BypassableAction.__init__(self, 'GetMorsel', bypass=bypass)
        
        
    def _run(
            self,
            manip,
            method,
            ui_device,
            state_pub=None,
            filename_trajdata=None):
        """
        Execute the specified method for picking up the morsel

        @param manip: pointer to robot manipulator
        @param method: method we should use to get morsel
        @type  method: string
        @param ui_device: input device for teleoperation/shared autonomy
        @type  ui_device: string
        @param state_pub: ros publisher for specifying when
                          we start and stop parts of execution
        @param filename_trajdata: filename for recording trial
        @type  filename_trajdata: string
        """
        robot = manip.GetRobot()
        env = robot.GetEnv()
        all_morsels = get_all_morsel_bodies(env)

        if all_morsels is None:
            raise ActionException(self, 'Failed to find morsel in environment.')

        fork = env.GetKinBody('fork')
        all_desired_ee_pose = [get_prestab_pose_for_morsel(morsel, fork, manip)
                                for morsel in all_morsels]

        #remove ones that we could not find prestab pose for
        all_desired_ee_pose = [pose for pose in all_desired_ee_pose
                                if pose is not None]
            
          
        all_desired_stab_ee_pose = [numpy.copy(pose) for pose in all_desired_ee_pose]
        zoffset = -0.06
        for pose in all_desired_stab_ee_pose:
            pose[2,3] += zoffset

  
        if state_pub:
            state_pub.publish("getting morsel with method " + str(method))
            if filename_trajdata and 'direct' not in method:
                state_pub.publish("recording data to " + str(filename_trajdata))

        if 'shared_auton' in method:
            #see if we should constrain magnitude of robot action 
            if method == 'shared_auton_prop':
                fix_magnitude_user_command = True
            else: #regular version without 
                fix_magnitude_user_command = False
            assistance_policy_action = AssistancePolicyAction(bypass=self.bypass)
            assistance_policy_action.execute(
                    manip, all_morsels, all_desired_ee_pose, ui_device, 
                    fix_magnitude_user_command=fix_magnitude_user_command,
                    filename_trajdata=filename_trajdata)

        elif method == 'blend':
            assistance_policy_action = AssistancePolicyAction(bypass=self.bypass)
            assistance_policy_action.execute(
                    manip, all_morsels, all_desired_ee_pose, ui_device, 
                    blend_only=True, filename_trajdata=filename_trajdata)
        elif method == 'direct':
            direct_teleop_action = DirectTeleopAction(bypass=self.bypass)
            direct_teleop_action.execute(
                    manip, ui_device, filename_trajdata=filename_trajdata)
        elif method == 'autonomous':
            desired_ee_pose = all_desired_ee_pose[0]
            try:
                with prpy.viz.RenderPoses([desired_ee_pose], env):

                    #since we know we will soon go to stabbed, rank iks based on both stabbed and current
                    ik_ranking_nominal_configs = [
                            robot.arm.GetDOFValues(),
                            numpy.array(robot.configurations.get_configuration('ada_meal_scenario_morselStabbedConfiguration')[1])
                            ]
                    ik_ranker = MultipleNominalConfigurations(ik_ranking_nominal_configs)
                    path = robot.PlanToEndEffectorPose(
                            desired_ee_pose,
                            execute=True,
                            ranker=ik_ranker)

            except PlanningError, e:
                raise ActionException(self, 'Failed to plan to pose near morsel: %s' % str(e))


        # Now stab the morsel
        try:
            direction = numpy.array([0., 0., -1.])
            
            #for now, desired height is mean of all morsel heights
            desired_height = numpy.mean([pose[2,3] for pose in all_desired_stab_ee_pose])
            curr_height = manip.GetEndEffectorTransform()[2,3]

            distance = max(curr_height - desired_height, 0.0)
            print 'distance moving down: ' + str(distance)

            with prpy.viz.RenderVector(manip.GetEndEffectorTransform()[:3,3],
                                       direction=direction, length=distance, env=env):
                with prpy.rave.Disabled(fork):
                    T = robot.arm.GetEndEffectorTransform()
                    path = robot.arm.PlanToEndEffectorOffset(direction=direction,
                                                 distance=distance,
                                                 execute=True)  #TODO: add some tolerance

                    #plan back up
                    path = robot.arm.PlanToEndEffectorOffset(direction=-direction,
                                                 distance=0.07,
                                                 execute=True)  #TODO: add some tolerance


        except PlanningError, e:
            raise ActionException(self, 'Failed to plan straight line path to grab morsel: %s' % str(e))

        # Grab the kinbody
        #robot.Grab(morsel)


def get_prestab_pose_for_morsel(morsel, fork, manip):
    """
    Given a morsel object, find the pose we want to target to stab
    Output pose holds the form above the morsel center

    @param morsel: pointer to morsel kinbody
    @param fork: pointer to fork kinbody method we should use to get morsel
    @param manip: pointer to robot manipulator
    """

    #fork top facing towards user
    desired_fork_tip_in_world = numpy.array([[-1.,  0., 0., 0.],
                                            [ 0.,  1., 0., 0.],
                                            [ 0.,  0.,-1., 0.],
                                            [ 0.,  0., 0., 1.]])

    morsel_pose = morsel.GetTransform()

    #read the offsets from the offsets file
    dir_path = os.path.dirname(os.path.realpath(__file__))
    interm_path = os.path.abspath(os.path.join(os.path.join(dir_path, os.pardir),os.pardir))
    parent_path = os.path.abspath(os.path.join(interm_path,os.pardir))
    offsets_file_path = os.path.join(parent_path, 'data/yaml/morsel_offsets.yaml')

    offsets_file_stream = open(offsets_file_path,'r')

    offsets = yaml.load(offsets_file_stream)
    xoffset = offsets.get('xoffset')
    yoffset = offsets.get('yoffset')
    zoffset = offsets.get('zoffset')


    desired_fork_tip_in_world[0,3] = morsel_pose[0,3] + xoffset
    desired_fork_tip_in_world[1,3] = morsel_pose[1,3] + yoffset
    desired_fork_tip_in_world[2,3] = morsel_pose[2,3] + zoffset

    fork_tip_in_world = fork.GetLink('tinetip').GetTransform()
    ee_in_world = manip.GetEndEffectorTransform()
    ee_in_fork_tip = numpy.dot(numpy.linalg.inv(fork_tip_in_world),
                            ee_in_world)
    desired_ee_pose = numpy.dot(desired_fork_tip_in_world, ee_in_fork_tip)

    #check to make sure ik solutions exist
    robot = manip.GetRobot()
    with robot:
        logger.info('looking for ik for morsel ' + morsel.GetName())
        ik_filter_options = openravepy.IkFilterOptions.CheckEnvCollisions
        #first call FindIKSolution which is faster if it succeeds
        ik_sol = manip.FindIKSolution(desired_ee_pose, ik_filter_options)
        #if it fails, call FindIKSolutions, which is slower but samples other start configurations
        if ik_sol is None:
            ik_sols = manip.FindIKSolutions(desired_ee_pose, ik_filter_options)
            if ik_sols is None:
                logger.info('Found no iks for morsel ' + morsel.GetName() + '. Removing from detected list.')
                return None
    logger.info('Found ik for morsel ' + morsel.GetName())
    return desired_ee_pose






  

