import numpy, prpy.viz
from bypassable_action import ActionException, BypassableAction
from prpy.planning.base import PlanningError

class GetMorsal(BypassableAction):

    def __init__(self, bypass=False):
        
        BypassableAction.__init__(self, 'EXECUTING_TRAJECTORY', bypass=bypass)
        
        
    def _run(self, robot):
        """
        Execute a sequence of plans that pick up the morsal
        @param robot The robot 
        """

        env = robot.GetEnv()
        morsal = env.GetKinBody('morsal')
        if morsal is None:
            raise ActionException(self, 'Failed to find morsal in environment.')

        desired_ee_pose = numpy.array([[0.04367424,  0.02037604, -0.99883801,  0.],
                                       [-0.99854746,  0.03246594, -0.04299924, 0],
                                       [ 0.03155207,  0.99926512,  0.02176437, 0.98],
                                       [ 0.        ,  0.        ,  0.        ,  1.        ]])

        morsal_pose = morsal.GetTransform()
        xoffset = -0.11
        yoffset = 0.035

        desired_ee_pose[0,3] = morsal_pose[0,3] + xoffset
        desired_ee_pose[1,3] = morsal_pose[1,3] + yoffset

        # Plan near morsal
        try:
            with prpy.viz.RenderPoses([numpy.eye(4), desired_ee_pose, robot.manip.GetEndEffectorTransform()], env):
                path = robot.PlanToEndEffectorPose(desired_ee_pose, execute=False)
                robot.ExecutePath(path)
        except PlanningError, e:
            raise ActionException(self, 'Failed to plan to pose near morsal: %s' % str(e))

        # Now stab the morsal
        try:
            direction = numpy.array([0., 0., -1.])
            distance = 0.11
            with prpy.viz.RenderVector(robot.manip.GetEndEffectorTransform()[:3,3],
                                       direction=direction, length=distance, env=env):
                path = robot.PlanToEndEffectorOffset(direction=direction,
                                                 distance=0.11,
                                                 execute=False)  #TODO: add some tolerance
                robot.ExecutePath(path)
        except PlanningError, e:
            raise ActionException(self, 'Failed to plan straight line path to grab morsal: %s' % str(e))

        # Grab the kinbody
        robot.Grab(morsal)
