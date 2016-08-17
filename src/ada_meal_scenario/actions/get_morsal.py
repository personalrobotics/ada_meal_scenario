import numpy, prpy.viz
from bypassable_action import ActionException, BypassableAction
from prpy.planning.base import PlanningError
import time
import openravepy

from assistance_policy_action import AssistancePolicyAction

from detect_morsal import morsal_index_to_name


#get_morsal_modes = ['direct', 'blend', 'shared_auton_1', 'shared_auton_2', 'auton']


def get_all_morsal_bodies(env):
  all_bodies = []
  for i in range(1000):
        morsal_name = morsal_index_to_name(i)
        morsal_body = env.GetKinBody(morsal_name)
        if morsal_body:
            all_bodies.append(morsal_body)
        else:
            break
  return all_bodies


class GetMorsal(BypassableAction):

    def __init__(self, bypass=False):
        
        BypassableAction.__init__(self, 'EXECUTING_TRAJECTORY', bypass=bypass)
        
        
    def _run(self, manip, method, ui_device):
        """
        Execute a sequence of plans that pick up the morsal
        @param manip The manipulator
        """
        global time
        robot = manip.GetRobot()
        env = robot.GetEnv()
        all_morsals = get_all_morsal_bodies(env)
        #morsal = all_morsals[0]
        if all_morsals is None:
            raise ActionException(self, 'Failed to find morsal in environment.')
  

        fork = env.GetKinBody('fork')
        #if True: #fork is None:
        if fork is None:
            all_desired_ee_pose = [numpy.array([[-0.06875708,  0.25515971, -0.96445113,  0.51087426],
                                           [ 0.2036257 ,  0.9499768 ,  0.23681355,  0.03655854],
                                           [ 0.97663147, -0.18010443, -0.11727471,  0.92 ],
                                           [ 0.        ,  0.        ,  0.        ,  1.        ]])
                                           for morsal in all_morsals]
        else:
            all_desired_ee_pose = [Get_Prestab_Pose_For_Morsal(morsal, fork, manip) for morsal in all_morsals]
            
          
        all_desired_stab_ee_pose = [numpy.copy(pose) for pose in all_desired_ee_pose]
        xoffset = 0.0
        yoffset = 0.00
        zoffset = -0.07
        for pose in all_desired_ee_pose:
            pose[0,3] += xoffset
            pose[1,3] += yoffset
        for pose in all_desired_stab_ee_pose:
            pose[2,3] += zoffset
        #import openravepy
        #h3 = openravepy.misc.DrawAxes(env, desired_ee_pose)

#        #save old limits
#        old_acceleration_limits = robot.GetDOFAccelerationLimits()
#        old_velocity_limits = robot.GetDOFVelocityLimits()
#
#        #slow down robot
#        robot.SetDOFVelocityLimits(0.5*robot.GetDOFVelocityLimits())
#        robot.SetDOFAccelerationLimits(0.8*robot.GetDOFAccelerationLimits())


        #TODO add plan to some start pose?

      

        if 'shared_auton' in method:
          if method == 'shared_auton_prop':
            fix_magnitude_user_command = True
          else:
            fix_magnitude_user_command = False
          assistance_policy_action = AssistancePolicyAction(bypass=self.bypass)
          assistance_policy_action.execute(manip, all_morsals, all_desired_ee_pose, ui_device, fix_magnitude_user_command)
        elif method == 'autonomous':
          desired_ee_pose = all_desired_ee_pose[0]
          try:
              with prpy.viz.RenderPoses([desired_ee_pose], env):

                  path = robot.PlanToEndEffectorPose(desired_ee_pose, execute=True)
                  
                  #path = robot.PlanToEndEffectorPose(desired_ee_pose, execute=False)
                  #res = openravepy.planningutils.SmoothTrajectory(path,1, 1, 'HauserParabolicSmoother', '')
                  #robot.ExecuteTrajectory(path)

          except PlanningError, e:
              raise ActionException(self, 'Failed to plan to pose near morsal: %s' % str(e))

        

#        # Plan near morsal
#        try:
#            with prpy.viz.RenderPoses([desired_ee_pose, desired_fork_tip_in_world], env):
#                
#
#
#                path = robot.PlanToEndEffectorPose(desired_ee_pose, execute=True)
#                
#                #path = robot.PlanToEndEffectorPose(desired_ee_pose, execute=False)
#                #res = openravepy.planningutils.SmoothTrajectory(path,1, 1, 'HauserParabolicSmoother', '')
#                #robot.ExecuteTrajectory(path)
#
#        except PlanningError, e:
#            raise ActionException(self, 'Failed to plan to pose near morsal: %s' % str(e))



        #time.sleep(4)
        # Now stab the morsal
        
        #restore velocity limits
#        robot.SetDOFVelocityLimits(old_velocity_limits)
#        robot.SetDOFAccelerationLimits(old_acceleration_limits)

        try:
            direction = numpy.array([0., 0., -1.])
            
            #for now, desired height is mean of all morsal heights
            desired_height = numpy.mean([pose[2,3] for pose in all_desired_stab_ee_pose])
            curr_height = manip.GetEndEffectorTransform()[2,3]

            distance = curr_height - desired_height
            print 'distance: ' + str(distance)

            with prpy.viz.RenderVector(manip.GetEndEffectorTransform()[:3,3],
                                       direction=direction, length=distance, env=env):
                with prpy.rave.Disabled(fork):
                    T = robot.arm.GetEndEffectorTransform()
                    path = robot.arm.PlanToEndEffectorOffset(direction=direction,
                                                 distance=distance,
                                                 execute=True)  #TODO: add some tolerance


               
                    #from IPython import embed
                    #embed()
                    
        except PlanningError, e:
            raise ActionException(self, 'Failed to plan straight line path to grab morsal: %s' % str(e))

        # Grab the kinbody
        #robot.Grab(morsal)




def Get_Prestab_Pose_For_Morsal(morsal, fork, manip):
    #TODO add checking of IKs before adding pose

    #fork top facing towards user
    desired_fork_tip_in_world = numpy.array([[-1.,  0., 0., 0.],
                                            [ 0.,  1., 0., 0.],
                                            [ 0.,  0.,-1., 0.],
                                            [ 0.,  0., 0., 1.]])

    morsal_pose = morsal.GetTransform()

    #old values
    #xoffset = -0.185
    #yoffset = 0.06
    
    xoffset = 0.01
    yoffset = -0.01#-0.005
    zoffset = 0.06

    desired_fork_tip_in_world[0,3] = morsal_pose[0,3] + xoffset
    desired_fork_tip_in_world[1,3] = morsal_pose[1,3] + yoffset
    desired_fork_tip_in_world[2,3] = morsal_pose[2,3] + zoffset

    fork_tip_in_world = fork.GetLink('tinetip').GetTransform()
    ee_in_world = manip.GetEndEffectorTransform()
    ee_in_fork_tip = numpy.dot(numpy.linalg.inv(fork_tip_in_world),
                            ee_in_world)
    desired_ee_pose = numpy.dot(desired_fork_tip_in_world, ee_in_fork_tip)

    return desired_ee_pose


