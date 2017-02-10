#creates a direct teleop action

from bypassable_action import ActionException, BypassableAction
from ada_teleoperation.AdaTeleopHandler import AdaTeleopHandler, Is_Done_Func_Button_Hold
from ada_teleoperation.DataRecordingUtils import TrajectoryData

class DirectTeleopAction(BypassableAction):
    def __init__(self, bypass=False):
        BypassableAction.__init__(self, 'DIRECT_TELEOP', bypass=bypass)
        """ Create a direct teleop action, where user controls the arm
        directly through their input device


        @param bypass: flag for bypassable action
        """
        
    def _run(self, manip, ui_device, filename_trajdata=None):
        """ Execute the direct teleop action

        @param manip: pointer to robot manipulator
        @param ui_device: input device for teleoperation/shared autonomy
        @type  ui_device: string
        @param filename_trajdata: filename for recording trial
        @type  filename_trajdata: string
        """
        robot = manip.GetRobot()
        env = robot.GetEnv()

        if filename_trajdata:
          traj_data_recording = TrajectoryData(filename_trajdata)
        else:
          traj_data_recording = None

        ada_teleop = AdaTeleopHandler(env, robot, teleop_interface=ui_device, 
                                      num_input_dofs=2, use_finger_mode=False)
        ada_teleop.ExecuteDirectTeleop(is_done_func=Is_Done_Func_Button_Hold,
                                       traj_data_recording=traj_data_recording)



