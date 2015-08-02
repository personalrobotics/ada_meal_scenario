#!/usr/bin/env python
import json
import numpy
import openravepy
import adapy
import prpy
import numpy as np


import tf
import rospkg

import os
import time

import rospy

from std_msgs.msg import String

from tasklogger import TaskLogger

slowVelocityLimits = np.asarray([ 0.3,0.3,0.3,0.3,0.3,0.3,0.78,0.78])

# lookingAtPlateConfiguration = np.asarray([0.81812309, -1.14344739,  0.36382416,  2.64179402, -1.37920679, -0.55691865])


defaultEndEffectorPose = np.asarray([[ 0.04367424,  0.02037604, -0.99883801,  0.65296864],
        [-0.99854746,  0.03246594, -0.04299924, -0.00927059],
        [ 0.03155207,  0.99926512,  0.02176437,  1.03388379],
        [ 0.        ,  0.        ,  0.        ,  1.        ]])

class AdaBiteServing(object):
  
  #Finite State Machine
  ROBOT_STATE = "INITIAL"
  plateDetectedTimes = 1
  RESOLUTION = 0.02
  NUMTRAJ = 9

  def addWaterServingTask(self):
    self.waterServing_task = self.tasklist.add_task('WaterServing')
    self.findGlass_subtask = self.waterServing_task.add_task('Find Glass')
    self.graspGlass_subtask = self.waterServing_task.add_task('Grasp Glass')
    self.liftGlass_subtask = self.waterServing_task.add_task('Lift Glass')
    self.drinkGlass_subtask = self.waterServing_task.add_task('Drink Glass')
    self.returnGlass_subtask = self.waterServing_task.add_task('Return Glass')
    self.placeGlass_subtask = self.waterServing_task.add_task('Place Glass')



  def initSimple(self):

    self.NUM_UPDATE_CALLS = 30
    self.Initialized = False

    rospy.init_node('bite_serving_scenario', anonymous = True)
    env_path = '/environments/table.env.xml'

    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
    openravepy.misc.InitOpenRAVELogging();
    self.env, self.robot = adapy.initialize(attach_viewer='qtcoin', sim=False, env_path = env_path)
    self.robot.SetActiveManipulator('Mico')
    self.manip = self.robot.GetActiveManipulator()


    # find the ordata
    rospack = rospkg.RosPack()
    file_root = rospack.get_path('pr_ordata')

    self.table = self.env.GetKinBody('table')

    robot_pose = numpy.array([[1, 0, 0, 0.409],[0, 1, 0, 0.338],[0, 0, 1, 0.795],[0, 0, 0, 1]])
    self.robot.SetTransform(robot_pose)

    ViewSide1Obj = self.env.GetKinBody('ViewSide1')
    ViewSide1Trans = ViewSide1Obj.GetTransform()

    ViewSide2Obj = self.env.GetKinBody('ViewSide2')
    ViewSide2Trans = ViewSide2Obj.GetTransform()

    ViewTopObj = self.env.GetKinBody('ViewTop')
    ViewTopTrans = ViewTopObj.GetTransform()

    self.numOfGrasps = 0
    self.numOfUpdateCalls = 100


    viewer = self.env.GetViewer()

    self.trajectoryPlanned = True


    # tf listener for querying transforms
    self.tfListener = tf.TransformListener()

    self.manip = self.robot.arm
    #self.robot.SetActiveManipulator(manip)
    activedofs = [i for i in range(6)]
    self.robot.SetActiveDOFs(activedofs)
    #self.robot.planner = prpy.planning.Sequence(self.robot.cbirrt_planner)      


    self.Initialized = True

    self.pub = rospy.Publisher('ada_tasks',String, queue_size=10)

    self.tasklist = TaskLogger()  
    self.addWaterServingTask()

    self.ball=self.env.ReadKinBodyURI('objects/smallsphere.kinbody.xml')
    self.ball.SetName('smallsphere')
    self.env.Add(self.ball)

    self.manip = self.robot.arm
    #self.manip.SetIKSolver(iksolver)

    self.statePub = rospy.Publisher('ROBOT_STATE', String, queue_size=10)
    #rospy.init_node('Robot State')
    self.rospyRate = rospy.Rate(33.33) # 10hz

    self.ROBOT_STATE = "EXECUTING_TRAJECTORY"
    self.statePub.publish(adaBiteServing.ROBOT_STATE)

    self.manip = self.robot.arm

    #load trajectories
    self.folderPath = os.path.dirname(os.path.abspath(os.path.join(__file__, os.pardir)))


    #path = self.robot.PlanToConfiguration(lookingAtPlateConfiguration, execute = False)
    path = self.robot.PlanToNamedConfiguration('ada_meal_scenario_lookingAtPlateConfiguration', execute=False)
    self.robot.ExecutePath(path)
    time.sleep(6)

  
    iksolver = openravepy.RaveCreateIkSolver(self.env,"NloptIK")
    self.manip.SetIKSolver(iksolver)
    self.bite_detected = False

    self.ROBOT_STATE = "LOOKING_AT_FACE"
    self.statePub.publish(adaBiteServing.ROBOT_STATE)

  

  def generateTrajectories(self):    
    defaultVelocityLimits = self.robot.GetDOFVelocityLimits()
    platecenter = numpy.array([0.729, -0.052,0.7612])
    startPose = numpy.zeros(2)
    startPose[0] = platecenter[0] - self.NUMTRAJ*self.RESOLUTION/2.0
    startPose[1] = platecenter[1] - self.NUMTRAJ*self.RESOLUTION/2.0


    #offsets for gripper
 


    for ii in range(0,self.NUMTRAJ):
    	for jj in range(0,self.NUMTRAJ):    
    	  bitePose = numpy.zeros(3)
          bitePose[0] = startPose[0] + ii*self.RESOLUTION
          bitePose[1] = startPose[1] + jj*self.RESOLUTION
          endEffectorPose = defaultEndEffectorPose.copy()
          endEffectorPose[0,3] = bitePose[0]-0.11
          endEffectorPose[1,3] = bitePose[1]+0.035
          endEffectorPose[2,3] = 0.98
          #from IPython import embed
          #embed()    
          print ii,jj
          path1 = self.robot.PlanToEndEffectorPose(endEffectorPose, execute = False)
          self.robot.ExecutePath(path1)
          time.sleep(4)
          path2 = self.robot.PlanToEndEffectorOffset(numpy.asarray([0, 0, -1]),0.11, execute = False)          
          traj_name1 =self.folderPath + '/data/trajectories/traj1_x%d_y%d.xml' % (ii, jj)
          traj_name2 =self.folderPath + '/data/trajectories/traj2_x%d_y%d.xml' % (ii, jj)
          prpy.rave.save_trajectory(path1, traj_name1)
          prpy.rave.save_trajectory(path2, traj_name2)
          time.sleep(3)
          path = self.robot.PlanToNamedConfiguration('ada_meal_scenario_lookingAtPlateConfiguration', execute=False)
          self.robot.ExecutePath(path)
          time.sleep(6)

    # traj1 = prpy.rave.load_trajectory(self.env,traj_name1)
    # traj2 = prpy.rave.load_trajectory(self.env,traj_name2)   

    # #self.robot.ExecutePath(path1)
    # #time.sleep(2)
    # #self.robot.ExecutePath(path2)
    # #time.sleep(2)
    # self.robot.PlanToConfiguration(lookingAtPlateConfiguration)
    # #self.robot.ExecuteTrajectory(self.traj_lookingAtPlate)
    # time.sleep(4)
    # self.robot.ExecuteTrajectory(traj1)
    # time.sleep(3)
    # self.robot.ExecuteTrajectory(traj2)
    # time.sleep(2)

    # #time.sleep(4)
    # #@self.robot.planner = prpy.planning.Sequence(self.robot.greedyik_planner, self.robot.cbirrt_planner) 
    # #embed()
    # #exit()
    # #from IPython import embed
    # #embed()
    # self.robot.ExecutePath(path)

    # time.sleep(2)

    # self.robot.ExecuteTrajectory(self.traj_serving)


    # time.sleep(5)
    # #print str(self.tasklist)
    # if(self.waterServing_task.is_complete()):
    #   self.addWaterServingTask()
    #   self.pub.publish(str(self.tasklist))
    # #self.robot.planner = prpy.planning.Sequence(self.robot.cbirrt_planner) 
    # self.ROBOT_STATE = "LOOKING_AT_FACE"

  
if __name__ == "__main__":
  adaBiteServing = AdaBiteServing()
  #while not rospy.is_shutdown():
  #adaBiteServing.ROBOT_STATE == "INITIAL"):
  adaBiteServing.initSimple()
  #adaBiteServing.lookingAtPlate()
  #  elif(adaBiteServing.ROBOT_STATE == "EXECUTING_TRAJECTORY"):
  adaBiteServing.generateTrajectories()
  #  else:
  #    print "Error: Unknown ROBOT_STATE"
  #adaBiteServing.statePub.publish(adaBiteServing.ROBOT_STATE)
  #adaBiteServing.rospyRate.sleep()
