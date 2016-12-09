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
        
        
    def _run(self, manip, method, ui_device, state_pub=None, filename_trajdata=None):
        """
        Execute a sequence of plans that pick up the morsel
        @param manip The manipulator
        """
        global time
        robot = manip.GetRobot()
        env = robot.GetEnv()
        all_morsels = get_all_morsel_bodies(env)
        #morsel = all_morsels[0]
        if all_morsels is None:
            raise ActionException(self, 'Failed to find morsel in environment.')

        fork = env.GetKinBody('fork')
        #if True: #fork is None:
        if fork is None:
            all_desired_ee_pose = [numpy.array([[-0.06875708,  0.25515971, -0.96445113,  0.51087426],
                                           [ 0.2036257 ,  0.9499768 ,  0.23681355,  0.03655854],
                                           [ 0.97663147, -0.18010443, -0.11727471,  0.92 ],
                                           [ 0.        ,  0.        ,  0.        ,  1.        ]])
                                           for morsel in all_morsels]
        else:
            all_desired_ee_pose = [Get_Prestab_Pose_For_Morsel(morsel, fork, manip) for morsel in all_morsels]
            #remove None
            all_desired_ee_pose = [pose for pose in all_desired_ee_pose if pose is not None]
            
          
        all_desired_stab_ee_pose = [numpy.copy(pose) for pose in all_desired_ee_pose]
        zoffset = -0.06
        for pose in all_desired_stab_ee_pose:
            pose[2,3] += zoffset

  
        if state_pub:
          state_pub.publish("getting morsel with method " + str(method))
          if filename_trajdata and 'direct' not in method:
            state_pub.publish("recording data to " + str(filename_trajdata))

        if 'shared_auton' in method:
          if method == 'shared_auton_prop':
            fix_magnitude_user_command = True
          else:
            fix_magnitude_user_command = False
          assistance_policy_action = AssistancePolicyAction(bypass=self.bypass)
          assistance_policy_action.execute(manip, all_morsels, all_desired_ee_pose, ui_device, fix_magnitude_user_command=fix_magnitude_user_command, filename_trajdata=filename_trajdata)
        elif method == 'blend':
          assistance_policy_action = AssistancePolicyAction(bypass=self.bypass)
          assistance_policy_action.execute(manip, all_morsels, all_desired_ee_pose, ui_device, blend_only=True, filename_trajdata=filename_trajdata)
        elif method == 'direct':
          direct_teleop_action = DirectTeleopAction(bypass=self.bypass)
          direct_teleop_action.execute(manip, ui_device, filename_trajdata=filename_trajdata)
        elif method == 'autonomous':
          desired_ee_pose = all_desired_ee_pose[0]
          try:
              with prpy.viz.RenderPoses([desired_ee_pose], env):

                  #since we know we will soon go to stabbed, rank iks based on both stabbed and current
                  ik_ranking_nominal_configs = [robot.arm.GetDOFValues(), numpy.array(robot.configurations.get_configuration('ada_meal_scenario_morselStabbedConfiguration')[1])]
                  ik_ranker = MultipleNominalConfigurations(ik_ranking_nominal_configs)
                  path = robot.PlanToEndEffectorPose(desired_ee_pose, execute=True, ranker=ik_ranker)

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



def Get_Prestab_Pose_For_Morsel(morsel, fork, manip):
    #fork top facing towards user
    desired_fork_tip_in_world = numpy.array([[-1.,  0., 0., 0.],
                                            [ 0.,  1., 0., 0.],
                                            [ 0.,  0.,-1., 0.],
                                            [ 0.,  0., 0., 1.]])

    morsel_pose = morsel.GetTransform()

    #old values
    #xoffset = -0.185
    #yoffset = 0.06
    
    xoffset, yoffset = read_offsets_from_file()
    #xoffset = 0.0
    #yoffset = 0.0#
    zoffset = 0.06

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


def read_offsets_from_file(filename='morsel_offsets.txt', xoffset=0., yoffset=0.):
  full_filename = rospkg.RosPack().get_path('ada_meal_scenario') + '/' + filename
  with open(full_filename, 'r') as f:
    while True:
      nextline = f.readline()
      if len(nextline) == 0:
        break
      split_line = nextline.split()
      #first check if second item in split is a number
      try:
        offset = float(split_line[1])
        if split_line[0] == 'xoffset':
          xoffset = offset
        elif split_line[0] == 'yoffset':
          yoffset = offset
        else:
          logger.info('unrecognized offset from line: ' + nextline)

      except ValueError:
        logger.info('could not read the following line because second value is not a float: ' + nextline)
        continue

  logger.info('read offsets: ' + str(xoffset) +', ' + str(yoffset))
  return xoffset,yoffset






  

