#!/usr/bin/env python
import Tkinter as tk

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

from twistHandler import Supervisor

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

readyToGraspConfiguration = np.asarray([ 1.59678399, -1.0000351 ,  0.14533722,  0.9793679 , -1.22807706,
        1.55532642])

readyToServePosition =  np.asarray([ 0.59867355, -1.39080925, -0.65546094,  0.9805578 , -1.22807706,
        1.55532642])


servingPosition = np.asarray([ 0.56979876, -1.40909664, -0.68818586,  0.9805578 , -1.22807706,
        0.58785866])

slowVelocityLimits = np.asarray([ 0.3,0.3,0.3,0.3,0.3,0.3,0.78,0.78])


class ADAmanipulationTester:


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
      self.env, self.robot = adapy.initialize(attach_viewer='rviz', sim=False, env_path = env_path)
      #embed()

      self.manip = self.robot.arm
      inds, pos = self.robot.configurations.get_configuration('home')
      pos[1] = -1.57;
      pos[2] = 0;
      #self.robot.SetDOFValues(pos, inds, openravepy.KinBody.CheckLimitsAction.Nothing)
      self.robot.SetDOFValues(self.robot.GetDOFValues())

      # find the ordata
      rospack = rospkg.RosPack()
      file_root = rospack.get_path('pr_ordata')
      #file_root = rospack.get_path('ada_description')

      # self.glass.SetTransform(glass_pose)
      self.table = self.env.GetKinBody('table')

      robot_pose = numpy.array([[1, 0, 0, 0.409],[0, 1, 0, 0.338],[0, 0, 1, 0.795],[0, 0, 0, 1]])
      self.robot.SetTransform(robot_pose)

      ViewSide1Obj = self.env.GetKinBody('ViewSide1')
      ViewSide1Trans = ViewSide1Obj.GetTransform()

      ViewSide2Obj = self.env.GetKinBody('ViewSide2')
      ViewSide2Trans = ViewSide2Obj.GetTransform()

      ViewTopObj = self.env.GetKinBody('ViewTop')
      ViewTopTrans = ViewTopObj.GetTransform()

      #prpy.rave.disable_padding(self.glass, False)
      #self.glass.GetLink('padding_glass').SetVisible(False)

      self.numOfGrasps = 0
      self.numOfUpdateCalls = 100


      viewer = self.env.GetViewer()
      #viewer.SetCamera(ViewTopTrans)

      self.trajectoryPlanned = True


      # tf listener for querying transforms
      self.tfListener = tf.TransformListener()

      self.UPDATING_POSE = 1
      self.GRASPING = 2
      self.RUNNING = 3
      self.MODE = self.UPDATING_POSE
 
      self.manip = self.robot.arm
      #self.robot.SetActiveManipulator(manip)
      activedofs = [i for i in range(6)]
      self.robot.SetActiveDOFs(activedofs)
      self.robot.planner = prpy.planning.Sequence(self.robot.cbirrt_planner)      

      #self.manip.PlanToConfiguration(startConfiguration)
      #time.sleep(5)
      self.Initialized = True

      self.pub = rospy.Publisher('ada_tasks',String, queue_size=10)
      #embed()

      self.tasklist = TaskLogger()  
      self.addWaterServingTask()

      #rospy.spin()   
 

  def planAndExecuteTrajectorySimple(self):    
          #res =  openravepy.planningutils.SmoothTrajectory(traj,1, 1, 'ParabolicSmoother', '')
          #self.controller.SetPath(traj)

          defaultVelocityLimits = self.robot.GetDOFVelocityLimits()


          self.robot.planner = prpy.planning.Sequence(self.robot.cbirrt_planner) 
            
          self.manip.PlanToConfiguration(startConfiguration)
          #embed()
          time.sleep(3)
          self.manip.PlanToConfiguration(readyToGraspConfiguration)
          time.sleep(3)
          #embed()
 
          self.robot.SetDOFVelocityLimits(slowVelocityLimits)

          self.manip.PlanToEndEffectorOffset([0,0,-1],0.16)
          time.sleep(5)
          self.manip.PlanToEndEffectorOffset([0,0,1],0.16)
          time.sleep(4)
          self.robot.SetDOFVelocityLimits(defaultVelocityLimits)
          self.manip.PlanToConfiguration(readyToServePosition)
          time.sleep(3)
          self.robot.SetDOFVelocityLimits(slowVelocityLimits)
          self.manip.PlanToConfiguration(servingPosition)
          time.sleep(8)
          self.manip.PlanToConfiguration(readyToServePosition)
          time.sleep(3)
          self.robot.SetDOFVelocityLimits(defaultVelocityLimits)
          self.manip.PlanToConfiguration(readyToGraspConfiguration)
          time.sleep(3)
  
          print str(self.tasklist)
          if(self.waterServing_task.is_complete()):
            self.addWaterServingTask()
            self.pub.publish(str(self.tasklist))

  
if __name__ == "__main__":
    adaManipulationTester = ADAmanipulationTester()
    adaManipulationTester.initSimple()
    adaManipulationTester.planAndExecuteTrajectorySimple()
    adaManipulationTester.robot.SetActiveDOFs([0,1,2,3,4,5,6,7]) #have to do that, otherwise get an error from roscontroller
    
    
    embed()
    #adaManipulationTester.initSimple() 
    #adaManipulationTester.planAndExecuteTrajectorySimple()