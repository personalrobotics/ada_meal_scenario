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
  

        desired_ee_pose = numpy.array([[-0.06875708,  0.25515971, -0.96445113,  0.51087426],
       [ 0.2036257 ,  0.9499768 ,  0.23681355,  0.03655854],
       [ 0.97663147, -0.18010443, -0.11727471,  0.94 ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]])

        desired_configuration = numpy.asarray([1.58427139, -0.46873633, -0.1000997, 0.68067841, -1.18999731,  2.32882465])
        morsal_pose = morsal.GetTransform()
        #xoffset = -0.11
        xoffset = -0.23
        #yoffset = 0.035
        yoffset = 0.04
        #yoffset = -0.11

        desired_ee_pose[0,3] = morsal_pose[0,3] + xoffset
        desired_ee_pose[1,3] = morsal_pose[1,3] + yoffset

        import openravepy
        h3 = openravepy.misc.DrawAxes(env, desired_ee_pose)

        #from IPython import embed
        #embed()
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
        #from IPython import embed
        #embed()
        # Now stab the morsal
        try:
            direction = numpy.array([0., 0., -1.])
            distance = 0.08
            with prpy.viz.RenderVector(manip.GetEndEffectorTransform()[:3,3],
                                       direction=direction, length=distance, env=env):
                path = robot.PlanToEndEffectorOffset(direction=direction,
                                                 distance=distance,
                                                 execute=False)  #TODO: add some tolerance
                #import openravepy
                res = openravepy.planningutils.SmoothTrajectory(path,1, 1, 'ParabolicSmoother', '')
                robot.ExecuteTrajectory(path)
                #robot.ExecutePath(path)
                import time
                #time.sleep(2)
        except PlanningError, e:
            raise ActionException(self, 'Failed to plan straight line path to grab morsal: %s' % str(e))

        # Grab the kinbody
        #robot.Grab(morsal)
