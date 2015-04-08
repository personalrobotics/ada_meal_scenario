import numpy, os, rospy, time
from bypassable_action import BypassableAction
from trajectory_actions import LookAtPlate
from std_msgs.msg import String
from catkin.find_in_workspaces import find_in_workspaces

class DetectMorsal(BypassableAction):

    def __init__(self, bypass=False):
        BypassableAction.__init__(self, 'DetectBite', bypass=bypass)

    def _run(self, robot):
        
        # Move to look at plate
        action = LookAtPlate(bypass = self.bypass)
        action.execute(robot)

        m_detector = MorsalDetector(robot)
        m_detector.start()

        # Now wait for the morsal to be detected
        env = robot.GetEnv()
        while not env.GetKinBody('morsal'):
            time.sleep(1)

        m_detector.stop()

    def _bypass(self, robot):

        # Here we want to place the kinbody
        #  somewhere in the environment
        morsal_in_camera = numpy.eye(4)
        morsal_in_camera[:3,3] = [0.1, 0., -0.25]
                
        m_detector = MorsalDetector(robot)
        m_detector.add_morsal(morsal_in_camera)

        import IPython; IPython.embed()

class MorsalDetector(object):
    
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot
        self.sub = None

    def start(self):
        self.sub = rospy.Subscriber("/perception/morsel_detection", 
                                    String, 
                                    self._callback, 
                                    queue_size=1)
    
    def stop(self):
        self.sub = None #unsubscribe

    def add_morsal(self, morsal_in_camera):
        camera_in_world = self.robot.GetLinks()[7].GetTransform()
        morsal_in_world = numpy.dot(camera_in_world, morsal_in_camera)

        object_base_path = find_in_workspaces(
            search_dirs=['share'],
            project='ada_meal_scenario',
            path='data',
            first_match_only=True)[0]
        ball_path = os.path.join(object_base_path, 'objects', 'smallsphere.kinbody.xml')
        morsal = self.env.ReadKinBodyURI(ball_path)
        morsal.SetName('morsal')
        morsal.SetTransform(morsal_in_world)
        self.env.Add(morsal)

        
    def _callback(self, msg):
        obj =  json.loads(msg.data)
        pts_arr = obj['pts3d']
        morsal_pos = np.asarray(pts_arr)
        if(morsal_pos is None) or(len(morsal_pos)==0):
            return

        morsal_in_camera = numpy.eye(4)
        morsal_in_camera[:3,3] = morsal_pos
        
        self.add_morsal(morsal_in_camera)
        
