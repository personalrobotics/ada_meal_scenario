import numpy, os, rospy, time, json
from bypassable_action import ActionException, BypassableAction
from std_msgs.msg import String, Int32
from catkin.find_in_workspaces import find_in_workspaces

import logging
logger = logging.getLogger('ada_meal_scenario')

class DetectMorsal(BypassableAction):

    def __init__(self, bypass=False):
        BypassableAction.__init__(self, 'DetectBite', bypass=bypass)

    def _run(self, robot, timeout=None):
        
        m_detector = MorsalDetector(robot)
        m_detector.start()

        # Now wait for the morsal to be detected
        env = robot.GetEnv()
        logger.info('Waiting to detect morsal')
        start_time = time.time()
        while not env.GetKinBody('morsal') and (timeout is None or time.time() - start_time < timeout):
            time.sleep(1.0)

        m_detector.stop()

        if not env.GetKinBody('morsal'):
            raise ActionException(self, 'Failed to detect morsal.')

    def _bypass(self, robot):

        # Here we want to place the kinbody
        #  somewhere in the environment
        morsal_in_camera = numpy.eye(4)
        morsal_in_camera[:3,3] = [0.1, 0., 0.25]
                
        m_detector = MorsalDetector(robot)
        m_detector.add_morsal(morsal_in_camera)

class MorsalDetector(object):
    
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot
        self.detection_sub = None
        self.selection_sub = None
        self.morsal_pos = None

    def start(self):
        logger.info('Subscribing to morsal detection and selection')
        self.detection_sub = rospy.Subscriber("/perception/morsel_detection", 
		                                      String, 
		                                      self._detection_callback, 
		                                      queue_size=1)
        self.selection_sub = rospy.Subscriber("/interface/morsel_selection",
        									  Int32,
        									  self._selection_callback,
        									  queue_size=1)
    
    def stop(self):
        logger.info('Unsubscribing from morsal detection and selection')
        self.detection_sub.unregister() # unsubscribe
        self.selection_sub.unregister() # unsubscribe
        self.detection_sub = None
        self.selection_sub = None
        self.morsal_pos = None

    def add_morsal(self, morsal_in_camera):
        camera_in_world = self.robot.GetLink('Camera_RGB_Frame').GetTransform()
        morsal_in_world = numpy.dot(camera_in_world, morsal_in_camera)
        import openravepy
        h1 = openravepy.misc.DrawAxes(self.env, camera_in_world)
        h2 = openravepy.misc.DrawAxes(self.env, morsal_in_world)

        
        object_base_path = find_in_workspaces(
            search_dirs=['share'],
            project='ada_meal_scenario',
            path='data',
            first_match_only=True)[0]
        ball_path = os.path.join(object_base_path, 'objects', 'smallsphere.kinbody.xml')
        if self.env.GetKinBody('morsal') is None:
           morsal = self.env.ReadKinBodyURI(ball_path)
           morsal.SetName('morsal')
           self.env.Add(morsal)
        else:
           morsal = self.env.GetKinBody('morsal')
        morsal.SetTransform(morsal_in_world)

        
    def _detection_callback(self, msg):
        logger.debug('Received detection')
        obj =  json.loads(msg.data)
        pts_arr = obj['pts3d']
        self.morsal_pos = numpy.asarray(pts_arr)
        logger.debug('Waiting for selection...')

        # if(self.morsal_pos is None) or (len(self.morsal_pos) <= 2):
        #     return

        # morsal_in_camera = numpy.eye(4)
        # morsal_in_camera[:3,3] = self.morsal_pos[2]

        # #check 
        # self.add_morsal(morsal_in_camera)

    def _selection_callback(self, msg):
    	logger.debug('Received selection')
    	index = msg.data

    	if(self.morsal_pos is None) or (len(self.morsal_pos) <= index):
            return

        morsal_in_camera = numpy.eye(4)
        morsal_in_camera[:3,3] = self.morsal_pos[index]

        #check 
        self.add_morsal(morsal_in_camera)
