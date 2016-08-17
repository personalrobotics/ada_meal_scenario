import rospy
from bypassable_action import BypassableAction
from trajectory_actions import LookAtFace, LookAtPlate, Serve
from detect_morsal import DetectMorsal
from get_morsal import GetMorsal
from std_msgs.msg import String
import time

import logging
logger = logging.getLogger('ada_meal_scenario')

class BiteServing(BypassableAction):

    def __init__(self, bypass = False):
        BypassableAction.__init__(self, 'BiteServing', bypass=bypass)

    def execute(self, manip, env, method, ui_device, detection_sim=False):
        
        # TODO: Does this need to be publishing regularly
        state_pub = rospy.Publisher('ada_tasks',String, queue_size=10)

        # Move to look at face
        #action = LookAtFace(bypass = self.bypass)
        #state_pub.publish(action.name)
        #action.execute(manip)


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
        action.execute(manip, method, ui_device)

        # Serve the morsal
        action = Serve(bypass = self.bypass)
        state_pub.publish(action.name)
        action.execute(manip)
