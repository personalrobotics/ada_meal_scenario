import numpy, prpy.viz
from bypassable_action import ActionException, BypassableAction
from prpy.planning.base import PlanningError
import time
import openravepy

class GetMorsal(BypassableAction):

    def __init__(self, bypass=False):
        
        BypassableAction.__init__(self, 'EXECUTING_TRAJECTORY', bypass=bypass)
        
    def _fix_angle(self,v):
        x = v[0]
        y = v[1]
        if x is 0 and y > 0:
             return numpy.pi/2
        elif x is 0 and y < 0:
             return -numpy.pi/2

        a = numpy.math.atan(numpy.math.fabs(y/x))
        if x >=0 and y>=0:
          return a
        elif x<=0 and y >=0:
          return numpy.pi - a
        elif x <=0 and y<=0:
           return numpy.pi + a
        else:
           return 2*numpy.pi - a
        

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
  
        if True: #fork is None:
            #desired_ee_pose = numpy.array([[-0.06875708,  0.25515971, -0.96445113,  0.51087426],
            #                               [ 0.2036257 ,  0.9499768 ,  0.23681355,  0.03655854],
            #                               [ 0.97663147, -0.18010443, -0.11727471,  0.92 ],
            #                               [ 0.        ,  0.        ,  0.        ,  1.        ]])
            #desired_ee_pose = numpy.array([[-0.21913245,  0.04642337, -0.97459009,  0.56724615],
            #              [ 0.80746562,  0.56934083, -0.15443539,  0.06885348],
            #              [ 0.54770452, -0.82078979, -0.16224633,  0.98467413],
            #              [ 0.        ,  0.        ,  0.        ,  1.        ]])
            #desired_ee_pose = numpy.array([[0 ,  0, -1.0,  0.56724615],
            #              [ 1.0,  0., -0,  0.06885348],
            #              [ 0, -1.0, 0,  0.98467413],
            #              [ 0.        ,  0.        ,  0.        ,  1.        ]])
            desired_ee_pose = numpy.array([[ 0.        ,  0.        , -1.        ,  0.46670763],
                                              [ 1.        ,  0.        ,  0.        ,  0.02193009],
                                              [ 0.        , -1.        ,  0.        ,  0.98467413],
                                              [ 0.        ,  0.        ,  0.        ,  1.        ]])


        else:
            
            desired_fork_tip_in_world = numpy.array([[ 0.,  0., 1., 0.],
                                                     [-1.,  0., 0., 0.],
                                                     [ 0., -1., 0., 0.],
                                                     [ 0.,  0., 0., 1.]])
            ee_in_world = manip.GetEndEffectorTransform()
            fork_tip_in_world = fork.GetLink('tinetip').GetTransform()
            ee_in_fork_tip = numpy.dot(numpy.linalg.inv(fork_tip_in_world),
                                       ee_in_world)
            desired_ee_pose = numpy.dot(desired_fork_tip_in_world, ee_in_fork_tip)

	morsal_pose = morsal.GetTransform();
	
	direction_morsal = [[morsal_pose[0,3] - desired_ee_pose[0,3]] , [morsal_pose[1,3] - desired_ee_pose[1,3]] , [0] , [0]]  
	direction_spoon = [[desired_ee_pose[0,0]],[desired_ee_pose[1,0]],[desired_ee_pose[2,0]],[desired_ee_pose[3,0]]];
	
        direction_morsal = direction_morsal[0:3] 
        direction_spoon = direction_spoon[0:3] 

        direction_morsal = numpy.transpose(numpy.array(direction_morsal))
        direction_spoon = numpy.transpose(numpy.array(direction_spoon))


	#angle_rot_spoon = numpy.math.asin(numpy.linalg.norm(numpy.cross(direction_spoon, direction_morsal)) / (numpy.linalg.norm(direction_spoon) * numpy.linalg.norm(direction_morsal)));
        angle_rot_spoon = numpy.math.acos(numpy.inner(direction_spoon, direction_morsal) / (numpy.linalg.norm(direction_spoon) * numpy.linalg.norm(direction_morsal)))
        angle_rot_spoon2 = self._fix_angle(direction_spoon[0])-self._fix_angle(direction_morsal[0])
        #from IPython import embed
        #embed()
        #angle_rot_spoon = angle_rot_spoon2


        #xoffset = -0.11
        #xoffset = -0.175
        #xoffset = -0.195 #this needs to change for spoon grasp
        #yoffset = 0.035
        #yoffset = 0.08
        #yoffset = -0.1
        #yoffset = -0.11

        #desired_ee_pose[0,3] = morsal_pose[0,3] + xoffset
        #desired_ee_pose[1,3] = morsal_pose[1,3] + yoffset
        #desired_ee_pose[2,3] = 1.03

        import openravepy

        #angle calculation 
        #numpy.subtract(1.0, 4.0)
        #def angle(v1, v2):
        #return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))
  
        # Plan near morsal
        #angle = -20.0/180*numpy.pi
        angle = angle_rot_spoon2
        Tx = numpy.array([[1,0,0,0],[0,numpy.cos(angle), -numpy.sin(angle), 0],[0,numpy.sin(angle),numpy.cos(angle),0],[0,0,0,1]])
        T = numpy.array([[numpy.cos(angle),0,numpy.sin(angle),0],[0,1,0, 0],[-numpy.sin(angle),0,numpy.cos(angle),0],[0,0,0,1]])
        #T = numpy.array([[numpy.cos(angle), -numpy.sin(angle), 0, 0],[numpy.sin(angle),numpy.cos(angle),0,0],[0,0,1,0],[0,0,0,1]])
        #from IPython import embed
        #embed()
        #desired_ee_pose = numpy.dot(T, desired_ee_pose)
	openravepy.misc.DrawAxes(env,desired_ee_pose)
        print "before rotation"
        #from IPython import embed
        #embed()
        desired_ee_pose = numpy.dot(desired_ee_pose,T)
        
        #openravepy.misc.DrawAxes(env,manip.GetEndEffectorTransform())

  
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
        h3 = openravepy.misc.DrawAxes(env, desired_ee_pose)
        print "after rotation"
        	
	desired_ee_pose_temp = desired_ee_pose
        scaling = 0.1 ## make it member variable of detect_morsal class
        gripper_offset = 0.15
        offset_redx_endeff =  scaling - 0.01 - 0.02       # Center distance of food - radius of food -radius of hand
        offset_bluez_endeff = gripper_offset
        
        trans_matrix = numpy.array([[1,0,0,offset_redx_endeff],[0,1, 0, 0],[0,0,1,offset_bluez_endeff ],[0,0,0,1]])
	desired_ee_pose = numpy.dot(desired_ee_pose_temp , trans_matrix )
	
	try:
            with prpy.viz.RenderPoses([desired_ee_pose], env):
                path_CHANGE = robot.PlanToEndEffectorPose(desired_ee_pose, execute=False)
                #path = robot.PlanToConfiguration(desired_configuration, execute=False)
                #import openravepy
                res_CHANGE = openravepy.planningutils.SmoothTrajectory(path_CHANGE,1, 1, 'ParabolicSmoother', '')
                robot.ExecuteTrajectory(path_CHANGE)
                #robot.ExecutePath(path)
        except PlanningError, e:
            raise ActionException(self, 'Failed to plan to pose near morsal: %s' % str(e))
        h_CHANGE = openravepy.misc.DrawAxes(env, desired_ee_pose)
        print

	#from IPython import embed
        #embed()
        time.sleep(2)

        #path = robot.PlanToEndEffectorPose(desired_ee_pose, execute=False)
        #path = robot.PlanToConfiguration(desired_configuration, execute=False)
        #import openravepy
        #res = openravepy.planningutils.SmoothTrajectory(path,1, 1, 'ParabolicSmoother', '')
        #robot.ExecuteTrajectory(path)
        try:
            direction = numpy.array([0., 0., -1.])
            #distance = 0.075  
            distance = 0.05
            plate = env.GetKinBody('plate')
            morsal = env.GetKinBody('morsal')
            with prpy.viz.RenderVector(manip.GetEndEffectorTransform()[:3,3],
                                       direction=direction, length=distance, env=env):
                with prpy.rave.Disabled(fork), prpy.rave.Disabled(plate), prpy.rave.Disabled(morsal):
                    path = robot.PlanToEndEffectorOffset(direction=direction,
                                                 distance=distance,
                                                 execute=False)  #TODO: add some tolerance


                    res = openravepy.planningutils.SmoothTrajectory(path,1, 1, 'ParabolicSmoother', '')

                    #robot.ExecutePath(path)
                    robot.ExecuteTrajectory(path)

                    import time
  
                    defaultLimits = robot.arm.GetVelocityLimits()
                    #robot.SetActiveDOFVelocities([0.05,0.05,0.05,0.05,0.05,0.05])
                    #time.sleep(1)
                    #path = robot.PlanToEndEffectorOffset(direction=[0,1,0],
                    #                             distance=0.02,
                    #                             execute=False)  #TODO: add some tolerance
                    #res = openravepy.planningutils.SmoothTrajectory(path,1, 1, 'ParabolicSmoother', '')
                    #robot.ExecuteTrajectory(path)
                    #robot.ExecutePath(path)
                    #from IPython import embed
                    #embed()
                    armDOFValues = robot.arm.GetDOFValues()
                    #armDOFValues[5] = 1.60
                    armDOFValues[5] = armDOFValues[5] - 40.0/180*numpy.pi 

                    robot.arm.PlanToConfiguration(armDOFValues)
                    #res = openravepy.planningutils.SmoothTrajectory(path,1, 1, 'ParabolicSmoother', '')
                    #robot.ExecuteTrajectory(path)
                    print "grasped morsel" 
                    from IPython import embed
                    embed()
                    #robot.SetActiveDOFVelocities(defaultLimits)

        except PlanningError, e:
            raise ActionException(self, 'Failed to plan straight line path to grab morsal: %s' % str(e))

        # Grab the kinbody
        #robot.Grab(morsal)

