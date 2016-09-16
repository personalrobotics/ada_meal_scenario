import rospy
from bypassable_action import BypassableAction
from trajectory_actions import LookAtFace, LookAtPlate, Serve
from detect_morsal import DetectMorsal
from get_morsal import GetMorsal
from std_msgs.msg import String
import time


from ada_teleoperation.DataRecordingUtils import *

import logging
logger = logging.getLogger('ada_meal_scenario')

from direct_teleop_action import DirectTeleopAction

class BiteServing(BypassableAction):

    def __init__(self, bypass = False):
        BypassableAction.__init__(self, 'BiteServing', bypass=bypass)

    def execute(self, manip, env, method, ui_device, detection_sim=False, record_trial=False):
        
        # TODO: Does this need to be publishing regularly
        state_pub = rospy.Publisher('ada_tasks',String, queue_size=10)

        # Move to look at face
        #action = LookAtFace(bypass = self.bypass)
        #state_pub.publish(action.name)
        #action.execute(manip)

        if record_trial:
          file_directory = rospkg.RosPack().get_path('ada_meal_scenario') + '/trajectory_data'
          rosbag_topic_names = ['/hydra_calib', '/ada/joy']
          rosbag_process = start_rosbag(rosbag_topic_names, file_directory=file_directory)




        #if direct teleop, skip sequence
        if method == 'direct':
          direct_teleop_action = DirectTeleopAction(bypass=self.bypass)
          direct_teleop_action.execute(manip, ui_device, record_trial=record_trial)

          #make sure we end at serving
          manip.PlanToNamedConfiguration('ada_meal_scenario_servingConfiguration', execute=True)
        else:

          # Move to look at plate
          action = LookAtPlate(bypass = self.bypass)
          state_pub.publish(action.name)
          action.execute(manip)

          # Detect morsal
          if self.bypass:
              detection_sim = True
          action = DetectMorsal(bypass = detection_sim)
          state_pub.publish(action.name)
          action.execute(manip.GetRobot())
                      
          # Move to get object
          action = GetMorsal(bypass = self.bypass)
          state_pub.publish(action.name)
          action.execute(manip, method, ui_device, record_trial=record_trial)

    
          # Serve the morsal
          action = Serve(bypass = self.bypass)
          state_pub.publish(action.name)
          action.execute(manip)

        if record_trial:
          stop_rosbag(rosbag_process)
