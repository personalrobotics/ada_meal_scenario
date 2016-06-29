import numpy, os, rospy, time, json
from bypassable_action import ActionException, BypassableAction
from std_msgs.msg import String
from catkin.find_in_workspaces import find_in_workspaces

import logging
logger = logging.getLogger('ada_meal_scenario')

#name which we will add indices to
morsal_base_name = 'morsal'
def morsal_index_to_name(ind):
    return morsal_base_name + str(ind)

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
        while not env.GetKinBody(morsal_index_to_name(0)) and (timeout is None or time.time() - start_time < timeout):
            time.sleep(1.0)

        m_detector.stop()

        if not env.GetKinBody(morsal_index_to_name(0)):
            raise ActionException(self, 'Failed to detect any morsals.')

    def _bypass(self, robot, num_morsals=2):

        for i in range(num_morsals):
            # Here we want to place the kinbody
            #  somewhere in the environment
            morsal_in_camera = numpy.eye(4)
            #morsal_in_camera[:3,3] = [0.1, 0., 0.25]
            morsal_in_camera[:3,3] = [0.05, -0.04, 0.45]

            #add random noise
            rand_max_norm = 0.15
            morsal_in_camera[0:2, 3] += numpy.random.rand(2)*2.*rand_max_norm - rand_max_norm

            #switch to this if you want to test noise in world frame, not camera frame
#            camera_in_world = robot.GetLink('Camera_RGB_Frame').GetTransform()
#            morsal_in_world = numpy.dot(camera_in_world, morsal_in_camera)
#            morsal_in_world[0:2, 3] += numpy.random.rand(2)*2.*rand_max_norm - rand_max_norm
#            morsal_in_camera = numpy.dot(numpy.linalg.inv(camera_in_world), morsal_in_world)

            m_detector = MorsalDetector(robot)
            m_detector.add_morsal(morsal_in_camera, morsal_index_to_name(i))
        self.remove_morsals_next_indices(robot.GetEnv(), num_morsals)

    
    def remove_morsals_next_indices(self, env, start_ind):
        """ Removes the OpenRAVE kin bodies for all morsals with index at
        or greater than the start index
        Assumes that if a morsal of index i if not in the environment, then no morsals
        with index > i are in the environment

        @param env the OpenRAVE environment
        @param ind the index
        """

        ind = start_ind
        morsal_body = env.GetKinBody(morsal_index_to_name(ind))
        while morsal_body:
            env.Remove(morsal_body)
            ind+=1
            morsal_body = env.GetKinBody(morsal_index_to_name(ind))


class MorsalDetector(object):
    
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot
        self.sub = None

    def start(self):
        logger.info('Subscribing to morsal detection')
        self.sub = rospy.Subscriber("/perception/morsel_detection", 
                                    String, 
                                    self._callback, 
                                    queue_size=1)
    
    def stop(self):
        logger.info('Unsubscribing from morsal detection')
        self.sub.unregister() # unsubscribe
        self.sub = None

    def add_morsal(self, morsal_in_camera, morsal_name=None):
        camera_in_world = self.robot.GetLink('Camera_RGB_Frame').GetTransform()
        morsal_in_world = numpy.dot(camera_in_world, morsal_in_camera)
        import openravepy
        h1 = openravepy.misc.DrawAxes(self.env, camera_in_world)
        h2 = openravepy.misc.DrawAxes(self.env, morsal_in_world)
        
        if morsal_name is None:
          morsal_name = 'morsal'
        
        object_base_path = find_in_workspaces(
            search_dirs=['share'],
            project='ada_meal_scenario',
            path='data',
            first_match_only=True)[0]
        ball_path = os.path.join(object_base_path, 'objects', 'smallsphere.kinbody.xml')
        if self.env.GetKinBody(morsal_name) is None:
           morsal = self.env.ReadKinBodyURI(ball_path)
           morsal.SetName(morsal_name)
           self.env.Add(morsal)
        else:
           morsal = self.env.GetKinBody(morsal_name)
        morsal.SetTransform(morsal_in_world)


        
    # TODO update this for multiple morsals
    def _callback(self, msg):
        logger.debug('Received detection')
        obj =  json.loads(msg.data)
        pts_arr = obj['pts3d']
        morsal_pos = numpy.asarray(pts_arr)
        if(morsal_pos is None) or(len(morsal_pos)==0):
            return

        for i in range(len(morsal_pos)):
          morsal_in_camera = numpy.eye(4)
          morsal_in_camera[:3,3] = morsal_pos[i]

          #check 
          self.add_morsal(morsal_in_camera, morsal_index_to_name(i))
        

