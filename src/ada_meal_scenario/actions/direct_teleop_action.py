#creates a direct teleop action

import logging, numpy, prpy, os
from ada_teleoperation.AdaTeleopHandler import AdaTeleopHandler, Is_Done_Func_Button_Hold
import rospy
import rospkg

from bypassable_action import ActionException, BypassableAction

class DirectTeleopAction(BypassableAction):
    def __init__(self, bypass=False):
        BypassableAction.__init__(self, 'DIRECT_TELEOP', bypass=bypass)
        
    def _run(self, manip, ui_device):
        robot = manip.GetRobot()
        env = robot.GetEnv()

        ada_teleop = AdaTeleopHandler(env, robot, teleop_interface=ui_device, num_input_dofs=2, use_finger_mode=False)
        ada_teleop.ExecuteDirectTeleop(is_done_func=Is_Done_Func_Button_Hold)
        #ada_handler.execute_direct_teleop(simulate_user=False)



