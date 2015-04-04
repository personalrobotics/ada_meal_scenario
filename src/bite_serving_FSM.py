#!/usr/bin/env python
import Tkinter as tk
import json
import numpy
#import roslib
#mport rospy
import openravepy
import adapy
import prpy
import numpy as np
import math
from IPython import embed
import tf
import rospkg
import sys

from threading import Timer
from time import sleep

import os
import time

import rospy
#from ar_track_alvar.msg import AlvarMarkers
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_matrix,quaternion_from_matrix

from std_msgs.msg import String
#rom TransformMatrix import *
#from str2num import *

import IPython

from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import *
#from adapy.tsr import glass as glassUtils

from tasklogger import TaskLogger

startConfiguration = np.asarray([ 1.83548311, -1.74693327, -0.65738594,  0.9805578 , -1.22807706,
        1.55532642])

#readyToGraspConfiguration = np.asarray([ 1.59678399, -1.0000351 ,  0.14533722,  0.9793679 , -1.22807706,
#        1.55532642])
readyToGraspConfiguration = np.asarray([  1.71132122e+00,  -4.20611407e-01,   7.18985850e-01, 9.59137999e-01,  -1.12216727e+00,   1.42323670e+00])


readyToServeConfiguration =  np.asarray([ 0.59867355, -1.39080925, -0.65546094,  0.9805578 , -1.22807706,
        1.55532642])


servingConfiguration = np.asarray([ 0.56979876, -1.40909664, -0.68818586,  0.9805578 , -1.22807706,
        0.58785866])

lookingAtFaceConfiguration = np.asarray([ 0.37537415, -0.81812309, -0.06833738,  1.13049803, -1.11502733,1.96230524])

lookingAtPlateConfiguration = np.asarray([0.81812309, -1.14344739,  0.36382416,  2.64179402, -1.37920679, -0.55691865])

slowVelocityLimits = np.asarray([ 0.3,0.3,0.3,0.3,0.3,0.3,0.78,0.78])

#defaultEndEffectorPose = np.asarray([[-0.00947845,  0.02440905, -0.99965712,  0.65161017],
#                                          [-0.98453928,  0.17463525,  0.01359925,  0.01263427],
#                                          [ 0.17490731,  0.9843306 ,  0.0223764 ,  1.10983745],
#                                          [ 0.        ,  0.        ,  0.        ,  1.        ]])
defaultEndEffectorPose = np.asarray([[ 0.04367424,  0.02037604, -0.99883801,  0.65296864],
        [-0.99854746,  0.03246594, -0.04299924, -0.00927059],
        [ 0.03155207,  0.99926512,  0.02176437,  1.03388379],
        [ 0.        ,  0.        ,  0.        ,  1.        ]])
#defaultEndEffectorPose = np.asarray([[-0.07987843,  0.13887   , -0.98708387,  0.39577101],
       # [-0.99675664, -0.02084251,  0.07772891,  0.02984476],
       # [-0.00977909,  0.99009127,  0.14008446,  1.01364661],
       # [ 0.        ,  0.        ,  0.        ,  1.        ]])

class ADAmanipulationTester:
  
  #Finite State Machine
  ROBOT_STATE = "INITIAL"
  plateDetectedTimes = 1
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
    #embed()

    self.manip = self.robot.arm
    inds, pos = self.robot.configurations.get_configuration('home')
    pos[1] = -1.57;
    pos[2] = 0;
    self.robot.SetDOFValues(self.robot.GetDOFValues())

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
    self.robot.planner = prpy.planning.Sequence(self.robot.cbirrt_planner)      


    self.Initialized = True

    self.pub = rospy.Publisher('ada_tasks',String, queue_size=10)
    self.sub = rospy.Subscriber("/perception/morsel_detection", String, self._MorselDetectionCallback, queue_size=1)

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
    self.statePub.publish(adaManipulationTester.ROBOT_STATE)

    #embed()
    #exit()
    self.manip = self.robot.arm

    #load trajectories
    folderPath = os.path.dirname(os.path.abspath(os.path.join(__file__, os.pardir)))
    self.traj_lookingAtFace = prpy.rave.load_trajectory(self.env,folderPath + "/data/trajectories/traj_lookingAtFace.xml")   
    self.traj_lookingAtPlate = prpy.rave.load_trajectory(self.env,folderPath + "/data/trajectories/traj_lookingAtPlate.xml")   
    self.traj_serving = prpy.rave.load_trajectory(self.env,folderPath + "/data/trajectories/traj_serving.xml")   


    self.robot.ExecuteTrajectory(self.traj_lookingAtFace)
    time.sleep(4)
    #embed()
    #self.robot.planner.PlanToConfiguration(self.robot,lookingAtFaceConfiguration)
    #time.sleep(3)
    #prpy.rave.save_trajectory(new_traj, "traj_lookingAtFace.xml")
    #embed()
  
    iksolver = openravepy.RaveCreateIkSolver(self.env,"NloptIK")
    self.manip.SetIKSolver(iksolver)
    #embed()
    self.bite_detected = False

    self.ROBOT_STATE = "LOOKING_AT_FACE"
    self.statePub.publish(adaManipulationTester.ROBOT_STATE)
    #embed()

  def lookingAtPlate(self):
    if(self.bite_detected == True):
      print "registering ball pose"
      self.ball.SetTransform(self.bite_world_pose)
      self.ROBOT_STATE = "EXECUTING_TRAJECTORY"

  def lookingAtFace(self):
    #get face recognition
    self.bite_detected = False
    self.ROBOT_STATE = "EXECUTING_TRAJECTORY"
    self.statePub.publish(adaManipulationTester.ROBOT_STATE)
    self.robot.ExecuteTrajectory(self.traj_lookingAtPlate)
    #self.manip.PlanToConfiguration(lookingAtPlateConfiguration)
    time.sleep(3)
    #embed()
    self.ROBOT_STATE = "LOOKING_AT_PLATE"
    self.statePub.publish(adaManipulationTester.ROBOT_STATE)

  def _MorselDetectionCallback(self, msg):
    obj =  json.loads(msg.data)
    arr = obj['pts3d']
    pos = np.asarray(arr)
    #embed()
    if(pos is None) or(len(pos)==0) or (self.ROBOT_STATE!="LOOKING_AT_PLATE"):
      return
    else:
      #self.plateDetectedTimes = self.plateDetectedTimes + 1
      relative_pos = pos[0]
      relative_pose = np.eye(4)
      relative_pose[0,3] = relative_pos[0] 
      relative_pose[1,3] = relative_pos[1]
      relative_pose[2,3] = relative_pos[2]
      world_camera = self.robot.GetLinks()[7].GetTransform()
      self.bite_world_pose = np.dot(world_camera,relative_pose)
      self.bite_detected = True


        

  def executeTrajectory(self):    
    defaultVelocityLimits = self.robot.GetDOFVelocityLimits()
    
    endEffectorPose = defaultEndEffectorPose.copy()
    endEffectorPose[0,3] = self.bite_world_pose[0,3]-0.09
    endEffectorPose[1,3] = self.bite_world_pose[1,3]+0.01
    endEffectorPose[2,3] = 0.95
    
    self.manip.PlanToEndEffectorPose(endEffectorPose)
    time.sleep(4)
    self.robot.planner = prpy.planning.Sequence(self.robot.greedyik_planner, self.robot.cbirrt_planner) 

    self.manip.PlanToEndEffectorOffset([0, 0, -1],0.07)

    time.sleep(2)
    #embed()
    #self.manip.PlanToEndEffectorOffset([0,0,-1],0.15)
    #time.sleep(2)
    #self.robot.planner = prpy.planning.Sequence(self.robot.cbirrt_planner) 
    #traj_servingConfiguration = self.robot.planner.PlanToConfiguration(self.robot,servingConfiguration)
    #time.sleep(3)
    #prpy.rave.save_trajectory(traj_servingConfiguration, "traj_servingConfiguration.xml")
    self.robot.ExecuteTrajectory(self.traj_serving)
    #self.manip.PlanToConfiguration(servingConfiguration)


    time.sleep(5)
    #print str(self.tasklist)
    if(self.waterServing_task.is_complete()):
      self.addWaterServingTask()
      self.pub.publish(str(self.tasklist))
    self.robot.planner = prpy.planning.Sequence(self.robot.cbirrt_planner) 
    self.ROBOT_STATE = "LOOKING_AT_FACE"

  
if __name__ == "__main__":
  adaManipulationTester = ADAmanipulationTester()
  while not rospy.is_shutdown():
    if(adaManipulationTester.ROBOT_STATE == "INITIAL"):
      adaManipulationTester.initSimple()
    elif(adaManipulationTester.ROBOT_STATE == "LOOKING_AT_FACE"):
      adaManipulationTester.lookingAtFace()
    elif(adaManipulationTester.ROBOT_STATE == "LOOKING_AT_PLATE"):
      adaManipulationTester.lookingAtPlate()
    elif(adaManipulationTester.ROBOT_STATE == "EXECUTING_TRAJECTORY"):
      adaManipulationTester.executeTrajectory()
    else:
      print "Error: Unknown ROBOT_STATE"
  adaManipulationTester.statePub.publish(adaManipulationTester.ROBOT_STATE)
  adaManipulationTester.rospyRate.sleep()
