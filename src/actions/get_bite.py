
from bypassable_action import BypassableAction

class GetBite(BypassableAction):

    def __init__(self, bypass=False):
        
        BypassableAction.__init__(self, 'EXECUTING_TRAJECTORY', bypass=bypass)
        
        
    def _run(self, robot):
        
        morsal = env.GetKinBody('morsal')
        if morsal is None:
            raise ActionException(self, 'Failed to find morsal in environment.')

        desired_ee_pose = numpy.array([[0.04367424,  0.02037604, -0.99883801,  0.65296864],
                                       [-0.99854746,  0.03246594, -0.04299924, -0.00927059],
                                       [ 0.03155207,  0.99926512,  0.02176437,  1.03388379],
                                       [ 0.        ,  0.        ,  0.        ,  1.        ]])

        morsal_pose = morsal.GetTransform()
        xoffset = -0.11
        yoffset = 0.035

        desired_ee_pose[0,3] = morsal_pose[0,3] + xoffset
        desired_ee_pose[1,3] = morsal_pose[1,3] + yoffset

        # Plan near morsal
        path = robot.PlanToEndEffectorPose(desired_ee_pose, execute=False)
        robot.ExecutePath(path)

        # Now stab the morsal
        path = robot.PlanToEndEffectorOffset(direction=numpy.array([0., 0., -1.]),
                                             distance = 0.11,
                                             execute=False)  #TODO: add some tolerance
        robot.ExecutePath(path)

        # Grab the kinbody
        robot.Grab(morsal)
