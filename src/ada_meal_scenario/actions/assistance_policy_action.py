#creates a shared autonomy policy that runs essentially as an action

from bypassable_action import ActionException, BypassableAction
from ada_assistance_policy.AdaHandler import AdaHandler
from ada_assistance_policy.Goal import Goal
from ada_teleoperation.DataRecordingUtils import TrajectoryData


class AssistancePolicyAction(BypassableAction):
    def __init__(self, bypass=False):
        BypassableAction.__init__(self, 'ASSISTANCE_POLICY', bypass=bypass)
        """ Create a shared autonomy action using ada_assistance_policy
        Action takes in a list of possible user goals and target poses, and
        runs our shared autonomy code to simultaneously predict and assist

        @param bypass: flag for bypassable action
        """
        
    def _run(
          self, 
          manip,
          objects,
          desired_ee_poses,
          ui_device,
          fix_magnitude_user_command=False,
          blend_only=False,
          filename_trajdata=None):
        """ Execute the shared autonomy action

        @param manip: pointer to robot manipulator
        @param objects: kinbody objects that are possible user goals
        @param desired_ee_poses: target ee pose for each object
        @param ui_device: input device for teleoperation/shared autonomy
        @type  ui_device: string
        @param fix_magnitude_user_command: flag for constraining total magnitude
               of user command to be the magnitude of user input
        @type  fix_magnitude_user_command: bool
        @param blend_only: flag for blending instead of full policy
        @type  blend_only: bool
        @param filename_trajdata: filename for recording trial
        @type  filename_trajdata: string
        """
        robot = manip.GetRobot()
        env = robot.GetEnv()

        if filename_trajdata:
          traj_data_recording = TrajectoryData(filename_trajdata)
        else:
          traj_data_recording = None

        all_goals = [Goal(obj.GetTransform(), [desired_ee_pose])
                      for obj, desired_ee_pose in zip(objects, desired_ee_poses)]
        ada_handler = AdaHandler(env, robot, all_goals, objects,
                                input_interface_name=ui_device,
                                num_input_dofs=2, use_finger_mode=False)
        ada_handler.execute_policy(simulate_user=False, blend_only=blend_only,
                                  fix_magnitude_user_command=fix_magnitude_user_command, 
                                  traj_data_recording=traj_data_recording)



