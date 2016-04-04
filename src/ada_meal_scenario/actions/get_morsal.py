import numpy, prpy.viz
from bypassable_action import ActionException, BypassableAction
from prpy.planning.base import PlanningError
import time
import openravepy

class GetMorsal(BypassableAction):

    def __init__(self, bypass=False):
        
        BypassableAction.__init__(self, 'EXECUTING_TRAJECTORY', bypass=bypass)
        
        
    def _run(self, manip):
        """
        Execute a sequence of plans that pick up the morsal
        @param manip The manipulator
        """
        global time
        robot = manip.GetRobot()
        env = robot.GetEnv()
        morsal = env.GetKinBody('morsal')
        if morsal is None:
            raise ActionException(self, 'Failed to find morsal in environment.')
  

        fork = env.GetKinBody('fork')
        #if True: #fork is None:
        if fork is None:
            desired_ee_pose = numpy.array([[-0.06875708,  0.25515971, -0.96445113,  0.51087426],
                                           [ 0.2036257 ,  0.9499768 ,  0.23681355,  0.03655854],
                                           [ 0.97663147, -0.18010443, -0.11727471,  0.92 ],
                                           [ 0.        ,  0.        ,  0.        ,  1.        ]])
        else:
            #TODO instead of fixing the pose, switch to TSR to sample orientations that face downward
            desired_fork_tip_in_world = numpy.array([[ 0.,  0., 1., 0.],
                                                     [-1.,  0., 0., 0.],
                                                     [ 0., -1., 0., 0.],
                                                     [ 0.,  0., 0., 1.]])

            morsal_pose = morsal.GetTransform()

            #old values
            #xoffset = -0.185
            #yoffset = 0.06
            
            xoffset = 0.0
            yoffset = 0.0
            zoffset = 0.06

            desired_fork_tip_in_world[0,3] = morsal_pose[0,3] + xoffset
            desired_fork_tip_in_world[1,3] = morsal_pose[1,3] + yoffset
            desired_fork_tip_in_world[2,3] = morsal_pose[2,3] + zoffset

            fork_tip_in_world = fork.GetLink('tinetip').GetTransform()
            ee_in_world = manip.GetEndEffectorTransform()
            ee_in_fork_tip = numpy.dot(numpy.linalg.inv(fork_tip_in_world),
                                       ee_in_world)
            desired_ee_pose = numpy.dot(desired_fork_tip_in_world, ee_in_fork_tip)

        import openravepy
        h3 = openravepy.misc.DrawAxes(env, desired_ee_pose)

        # Plan near morsal
        try:
            with prpy.viz.RenderPoses([desired_ee_pose], env):
                path = robot.PlanToEndEffectorPose(desired_ee_pose, execute=False)
                #path = robot.PlanToConfiguration(desired_configuration, execute=False)
                #import openravepy
                res = openravepy.planningutils.SmoothTrajectory(path,1, 1, 'ParabolicSmoother', '')
                robot.ExecuteTrajectory(path)
                #robot.ExecutePath(path)
        except PlanningError, e:
            raise ActionException(self, 'Failed to plan to pose near morsal: %s' % str(e))
        #time.sleep(4)
        # Now stab the morsal
        time.sleep(2)
        try:
            direction = numpy.array([0., 0., -1.])
            distance = 0.06
            with prpy.viz.RenderVector(manip.GetEndEffectorTransform()[:3,3],
                                       direction=direction, length=distance, env=env):
                with prpy.rave.Disabled(fork):
                    path = robot.PlanToEndEffectorOffset(direction=direction,
                                                 distance=distance,
                                                 execute=True)  #TODO: add some tolerance
                    #import openravepy


                    #res = openravepy.planningutils.SmoothTrajectory(path,1, 1, 'ParabolicSmoother', '')

                    #from IPython import embed
                    #embed()
                    
                    #robot.ExecuteTrajectory(path)
                    #robot.ExecutePath(path)
                    #import time
                    #time.sleep(2)
        except PlanningError, e:
            raise ActionException(self, 'Failed to plan straight line path to grab morsal: %s' % str(e))

        # Grab the kinbody
        #robot.Grab(morsal)
