from bypassable_action import BypassableAction
from trajectory_actions import LookAtFace
from detect_bite import DetectBite
from get_bite import GetBite

import logging
logger = logging.getLogger('ada_meal_scenario')

class BiteServing(BypassableAction):

    def __init__(self, bypass = False):
        BypassableAction.__init__(self, 'BiteServing', bypass=bypass)

    def execute(self, robot, env):
        
        # Move to look at face
        action = LookAtFace(bypass = self.bypass)
        action.execute(robot)
        
        # Detect bite
        action = DetectBite(bypass = self.bypass)
        action.execute(robot)
        
        # Move to get object
        action = GetBite(bypass = self.bypass)
        action.execute(robot)

        # Serve the morsal
        action = Serve(bypass = self.bypass)
        action.execute(robot)
