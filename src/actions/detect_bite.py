from bypassable_action import BypassableAction
from trajectory_actions import LookAtPlate

class DetectBite(BypassableAction):

    def __init__(self, bypass=False):
        BypassableAction.__init__(self, 'DetectBite', bypass=bypass)

    def _run(self, robot):
        
        # Move to look at plate
        action = LookAtPlate(bypass = self.bypass)
        action.execute(robot)

        # Now wait for the morsal to be detected
        while not env.GetKinBody('morsal')
            time.sleep(1)

    def _bypass(self, robot):

        # TODO: Here we want to place the kinbody
        #  somewhere in the environment
        pass
