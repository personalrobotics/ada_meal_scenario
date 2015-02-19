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
from adapy.tsr import glass as glassUtils

from tasklogger import TaskLogger

 
#some hard-coded configurations   


#liftConfiguration = np.asarray([  2.20700740e+00,  -9.48060314e-01,  -5.36111239e-01,      6.41408500e-01,  -1.46131657e+00,   2.29788458e+00])
startConfiguration = np.asarray([ 2.54388183, -0.46103633,  0.22329953,  1.94207614, -1.07575742, 2.2014946])

liftConfiguration1 = np.asarray([ 1.94135815, -0.63236094, -0.36959909,  1.04243795, -1.05195742, 2.3978442])
liftConfiguration2 = np.asarray([-1.43604663, -0.69011079, -0.37537415,  1.04243795, -1.05314745, 2.3978442])

drinkConfiguration = np.asarray([ -1.43604663e+00,  -2.74311718e-01,  -7.22835782e-01, 1.04243795e+00,  -1.05314745e+00,   2.39784420e+00])

#placeConfiguration = np.asarray([ 2.77103083, -0.17324981,  0.71128585,  1.18880715, -0.26060922, 1.31018697])
placeConfiguration = np.asarray([2.77103083, -0.55439877,  0.26661185,  1.18880741, -0.26060922,  1.31018697])
#placeConfiguration = np.asarray([-1.43604663, -0.69011079, -0.37537415,  1.04243795, -1.05314745, 2.3978442])

#dropConfiguration = np.asarray([  2.77584370e+00,   3.17624571e-02,   8.38335681e-01, 1.18880715e+00,  -2.60609221e-01,   1.31018697e+00])
#dropConfiguration = np.asarray([  2.77584370e+00,   3.17624571e-02,   0.95, 1.18880715e+00,  -2.60609221e-01,   1.31018697e+00])
#dropConfiguration = np.asarray([ 1.29840954, -0.37056156,  0.24447446,  1.18880741, -0.26060922, 1.31018684])
dropConfiguration1 = np.asarray([ 2.16754521, -0.39173662, -0.24543693,  1.04005815, -1.05314745,
        2.3978442])

#dropConfiguration2 = np.asarray([ 2.16754521, -0.39173662, -0.24543693,  1.04005815, -1.05314745,
#        2.3978442 ])
dropConfiguration2 = np.asarray([ 2.16754521,  0.127748 ,  0.10202477,  1.04005815, -1.05314745,   2.3978442])



#ReleasedConfiguration = numpy.asarray([  3.3e+00,   3.17624571e-02,   8.38335681e-01,  1.18880715e+00,  -2.60609221e-01,   1.31018697e+00])
#releasedConfiguration = numpy.asarray([  1.3e+00,   3.17624571e-02,   8.38335681e-01,
#          1.18880741e+00,  -2.60609221e-01,   1.31018684e+00])


slowVelocityLimits = np.asarray([ 0.3,0.3,0.3,0.3,0.3,0.3,0.78,0.78])

#startConfiguration = np.asarray([ 2.17332001, -0.06063751,  1.04142249,  1.2602073 , -0.67115841, 1.31018697])
  
class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

def publishTf(tf_broadcaster, robot, frame_name):  
       transform = robot.GetTransform()
       pose = openravepy.poseFromMatrix(transform)
       quat = pose[0:4]
       w = quat[0]
       x = quat[1]
       y = quat[2]
       z = quat[3]
       trans = pose[4:7]
       #ros_quat = numpy.array([x,y,z,w])
       
       tf_broadcaster.sendTransform((trans[0], trans[1], trans[2]),
                        #tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                        (x,y,z,w),
                        rospy.Time.now(),
                        frame_name,
                        "/map")
       #print "published transform"

class ADAmanipulationTester:

  def object_pose_callback(self, msg):
    if(len(msg.markers)>0):
       
                marker = msg.markers[0]
                position_data = marker.pose.position
                orientation_data = marker.pose.orientation

                object_pose = numpy.matrix(quaternion_matrix([orientation_data.x,
                                                             orientation_data.y,
                                                             orientation_data.z,
                                                             orientation_data.w]))
                object_pose[0,3] = position_data.x 
                object_pose[1,3] = position_data.y
                object_pose[2,3] = position_data.z

                world_camera = self.robot.GetLinks()[7].GetTransform()

                object_world_pose = np.dot(world_camera,object_pose)

  
                if(self.MODE == self.UPDATING_POSE):
                        print 'detected ', marker.ns
                        self.numOfUpdateCalls = self.numOfUpdateCalls + 1
                        object_world_pose[2,3] = object_world_pose[2,3] + 0.02
                        object_world_pose[0,3] = object_world_pose[0,3] + 0.01 #make the glass a little closer so that it succeeds
                        self.glass.SetTransform(np.asarray(object_world_pose))
                        #embed()
                        #exit()
                        h  = openravepy.misc.DrawAxes(self.env,object_world_pose)

                        if(self.numOfUpdateCalls > self.NUM_UPDATE_CALLS):
                           self.MODE = self.GRASPING 
                           self.findGlass_subtask.succeed()
                           self.pub.publish(str(self.tasklist))
                           #self.NUM_UPDATE_CALLS = 20
                elif self.MODE == self.GRASPING and self.Initialized == True:

                        self.MODE = self.RUNNING
                        self.planAndExecuteTrajectorySimple()
                        time.sleep(6)
                        print "*********************Done grasping!***************************"
                        self.numOfGrasps = self.numOfGrasps + 1
                        self.numOfUpdateCalls = 0
                        self.MODE = self.UPDATING_POSE
                        glass_start_pose= numpy.eye(4)
                        self.glass.SetTransform(glass_start_pose)

            #time.sleep(10.0)

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

      rospy.init_node('ada_meal_scenario', anonymous = True)
      env_path = '/environments/tablewithglass.env.xml'

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

      #embed()
      #exit()

      # find the ordata
      rospack = rospkg.RosPack()
      file_root = rospack.get_path('pr_ordata')
      #file_root = rospack.get_path('ada_description')

      # self.glass.SetTransform(glass_pose)
      self.table = self.env.GetKinBody('table')

      self.glass = self.env.GetKinBody('glass')
      glass_start_pose= numpy.eye(4)
      self.glass.SetTransform(glass_start_pose)

      robot_pose = numpy.array([[1, 0, 0, 0.409],[0, 1, 0, 0.338],[0, 0, 1, 0.795],[0, 0, 0, 1]])
      self.robot.SetTransform(robot_pose)

      ViewSide1Obj = self.env.GetKinBody('ViewSide1')
      ViewSide1Trans = ViewSide1Obj.GetTransform()

      ViewSide2Obj = self.env.GetKinBody('ViewSide2')
      ViewSide2Trans = ViewSide2Obj.GetTransform()

      ViewTopObj = self.env.GetKinBody('ViewTop')
      ViewTopTrans = ViewTopObj.GetTransform()

      prpy.rave.disable_padding(self.glass, False)
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


      self.robot.arm.hand.OpenHand()
      time.sleep(6)
      self.manip.PlanToConfiguration(startConfiguration)
      time.sleep(6)
      self.Initialized = True
      #embed()
      #exit()
      self.sub = rospy.Subscriber('object_poses_array', MarkerArray, self.object_pose_callback)
      self.pub = rospy.Publisher('ada_tasks',String, queue_size=10)

      self.tasklist = TaskLogger()  
      self.addWaterServingTask()


      #new thread for tf global transform
      br = tf.TransformBroadcaster()
      t = RepeatedTimer(0.5, publishTf, br, self.robot, "/base_link")
      t.start() 
      rospy.spin()    

  def getVelocitiesTrajectory(self, cartesian_vels):
      jac = self.robot.arm.CalculateJacobian()
      jointVels = numpy.dot(numpy.transpose(jac),numpy.array(numpy.transpose(cartesian_vels)))
      traj = openravepy.RaveCreateTrajectory(self.env, '')
      self.robot.SetActiveDOFs([0,1,2,3,4,5,6,7]) #have to do that, otherwise get an error from roscontroller
      velocitySpecification = openravepy.ConfigurationSpecification.ConvertToVelocitySpecification(self.robot.GetActiveConfigurationSpecification())
      traj.Init(velocitySpecification)
      velValues = np.zeros([8])
      velValues[0:6] = jointVels
      traj.Insert(0,velValues)
      self.robot.SetActiveDOFs([0,1,2,3,4,5])
      return traj

  def planAndExecuteTrajectorySimple(self):    
      #embed()
      try:
          #res =  openravepy.planningutils.SmoothTrajectory(traj,1, 1, 'ParabolicSmoother', '')
          #self.controller.SetPath(traj)
          defaultVelocityLimits = self.robot.GetDOFVelocityLimits()


          prpy.rave.disable_padding(self.glass, False)

        
          grasp_chains = glassUtils.glass_grasp(self.robot, self.glass, self.manip)

          self.robot.planner = prpy.planning.Sequence(self.robot.cbirrt_planner) 


          self.robot.PlanToTSR(self.robot.tsrlibrary(self.glass, 'grasp'))

          #self.robot.PlanToTSR(grasp_chains)

          reached = False
          distThres = 0.4
          while(reached == False):
             glassPose = self.glass.GetTransform()
             glassPos = numpy.array([glassPose[0,3],glassPose[1,3],glassPose[2,3]])
             robotPose = self.manip.GetEndEffectorTransform()
             robotPos = numpy.array([robotPose[0,3],robotPose[1,3],robotPose[2,3]])
             if(numpy.linalg.norm(robotPos-glassPos) < distThres):
                  print "*******target reached!************"
                  reached = True
                  time.sleep(3.0)
                  self.robot.arm.hand.CloseHandTight()
                  self.robot.Grab(self.glass)
                  self.graspGlass_subtask.succeed()
                  self.pub.publish(str(self.tasklist))


                  #self.robot.Grab(self.glass)
                  #time.sleep(4.0)

          reached = False
          activedofs = [i for i in range(8)]
          self.robot.SetActiveDOFs(activedofs)
          while(reached == False):
            values = self.robot.GetActiveDOFValues()
            target_finger_pos = numpy.array([0.8,0.8])
            current_pos = numpy.array([values[6], values[7]])
            #embed()
            if(numpy.linalg.norm(current_pos - target_finger_pos)<0.1):
                time.sleep(1.5)
                #with prpy.rave.Disabled(self.glass), prpy.rave.Disabled(self.robot):
                #with prpy.rave.Disabled(self.glass):
                currentConfiguration = self.manip.GetDOFValues()

                time.sleep(0.5)

                #jointVelsUp = self.getVelocitiesTrajectory([0,0,0.5])
                #self.manip.controller.SetPath(jointVelsUp)
                #time.sleep(2)
                #jointVelsZero = self.getVelocitiesTrajectory([0,0,0])
                #self.manip.controller.SetPath(jointVelsZero)
                time.sleep(1)
                #embed()
                #exit()
                with prpy.rave.Disabled(self.table):
                    time.sleep(1)
                    
                    print 'starting to plan configuration'
                    self.manip.PlanToConfiguration(liftConfiguration1)
                    time.sleep(3)
                    reached = True


                self.manip.PlanToConfiguration(liftConfiguration2)
                #time.sleep(1)
                #self.manip.PlanToConfiguration(placeConfiguration)
                self.liftGlass_subtask.succeed()
                self.pub.publish(str(self.tasklist))
                time.sleep(8)
                self.robot.SetDOFVelocityLimits(slowVelocityLimits)       
                self.manip.PlanToConfiguration(drinkConfiguration)
                time.sleep(6)
                self.drinkGlass_subtask.succeed()
                self.pub.publish(str(self.tasklist))
                self.manip.PlanToConfiguration(liftConfiguration2)
                time.sleep(3)
                self.robot.SetDOFVelocityLimits(defaultVelocityLimits)
                #embed()
                print '11111111111111111111111111111111111111111'
                self.manip.PlanToConfiguration(liftConfiguration1)
                self.returnGlass_subtask.succeed()
                self.pub.publish(str(self.tasklist))
                time.sleep(6)
                print('222222222222222222222222222222222222')
                self.robot.SetDOFVelocityLimits(slowVelocityLimits)
                #embed()
                with prpy.rave.Disabled(self.glass):
                  self.manip.PlanToConfiguration(dropConfiguration2)
                  time.sleep(7)
                  #$self.manip.PlanToConfiguration(ReleasedConfiguration)
                  self.robot.arm.hand.CloseHand()
                  time.sleep(4.0)
                  self.robot.arm.hand.OpenHand()
                  time.sleep(3.0)
                  self.robot.Release(self.glass)
                  self.placeGlass_subtask.succeed()
                  self.pub.publish(str(self.tasklist))
                  self.robot.SetDOFVelocityLimits(defaultVelocityLimits)

                  #time.sleep(3.0)
                  self.manip.PlanToConfiguration(startConfiguration)
                  self.waterServing_task.succeed()
                  self.pub.publish(str(self.tasklist))
                  time.sleep(3.0)
                  #exit()
                     

                
          # with prpy.rave.Disabled(self.glass):
          #   defaultVelocityLimits = self.robot.GetDOFVelocityLimits()
          #   self.robot.SetDOFVelocityLimits(slowVelocityLimits)       
          #   self.manip.PlanToConfiguration(dropConfiguration)
          #   #embed()
          #   #exit()
          #   time.sleep(4)
          #   self.robot.SetDOFVelocityLimits(defaultVelocityLimits)
          #   self.robot.arm.hand.OpenHand()
          #   self.robot.Release(self.glass)
          #   time.sleep(5)
          #   self.manip.PlanToConfiguration(ReleasedConfiguration)

                    #jointVelsDown = self.getVelocitiesTrajectory([0,0,-0.6])
                    #self.manip.controller.SetPath(jointVelsDown)
                    #time.sleep(4)
                    #self.manip.controller.SetPath(jointVelsZero)
                #self.robot.SetDOFVelocityLimits(defaultVelocityLimits)
         #time.sleep(3.0)

          #with prpy.rave.Disabled(self.glass), prpy.rave.Disabled(self.robot):
          
          #with prpy.rave.Disabled(self.glass):
          #time.sleep(1)

          #add cylinder around glass, which has a thicker geometry. I need to replace this in the future with different geometry groups
          #embed()
          prpy.rave.disable_padding(self.glass, True)
          glass_pose_curr = self.glass.GetTransform()
          glass_pose = numpy.eye(4)
          # glass_pose[0,3] = glass_pose_curr[0,3]
          # glass_pose[1,3] = glass_pose_curr[1,3] 
          # glass_pose[2,3] = glass_pose_curr[2,3]
          self.glass.SetTransform(glass_pose)
          print str(self.tasklist)
          if(self.waterServing_task.is_complete()):
            self.addWaterServingTask()
            self.pub.publish(str(self.tasklist))

      except:
          self.numOfUpdateCalls = 0
          self.manip.PlanToConfiguration(startConfiguration)
          time.sleep(2)
          self.robot.arm.hand.OpenHand()
          self.MODE = self.UPDATING_POSE
          glass_start_pose= numpy.eye(4)
          self.glass.SetTransform(glass_start_pose)

    
if __name__ == "__main__":
    adaManipulationTester = ADAmanipulationTester()
    adaManipulationTester.initSimple() 
