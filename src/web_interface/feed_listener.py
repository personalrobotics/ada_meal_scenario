# rostopic echo /perception/morsel_detection
# python biteserver.py structureio_settings.json verbose.json

import sys, subprocess, json, logging, threading
import rospy, std_msgs
import time

class MorselFeedListener:
    bites = []
    detection_sub = None
    selection_pub = None

    @staticmethod
    def init():
        # MorselFeedListener.detection_sub = rospy.Subscriber("/perception/morsel_detection",
        #                                                     std_msgs.msg.String,
        #                                                     MorselFeedListener._detection_callback,
        #                                                     queue_size = 1)
        
        # MorselFeedListener.selection_pub = rospy.Publisher("/interface/morsel_selection",
        #                                                    std_msgs.msg.Int32,
        #                                                    queue_size = 1)

        # MorselFeedListener.simulate_stream()

        MorselFeedListener.subscribe_via_cmdline()

    @staticmethod
    def _detection_callback(self, data):
        logging.debug("received callback")
        try:
            raw_data = json.loads(data)
            raw_bites = raw_data["bites"]
            if raw_bites != []:
              MorselFeedListener.bites = MorselFeedListener.process_raw_bites(raw_bites)
        except ValueError:
            pass

    @staticmethod
    def process_raw_bites(raw_bites):
        new_bites = []

        for raw_bite in raw_bites:
            new_bite = {}
            new_bite["x"] = raw_bite[0][0]
            new_bite["y"] = raw_bite[0][1]
            new_bite["r"] = raw_bite[1]
            new_bites.append(new_bite)

        return new_bites

    @staticmethod
    def publish_selection(index):
        publishThread = threading.Thread(target=MorselFeedListener.publish_via_cmdline, args=(index))
        publishThread.daemon = True
        publishThread.start()
        # MorselFeedListener.selection_pub.publish(index)

    # methods for debugging
    @staticmethod
    def subscribe_via_cmdline():
        command = "rostopic echo /perception/morsel_detection"

        data_feed = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE)

        for line in data_feed.stdout:
          logging.debug("received cmdline data")
          try:
              raw_data = json.loads(line[6:])
              raw_bites = raw_data["bites"]
              if raw_bites != []:
                  MorselFeedListener.bites = MorselFeedListener.process_raw_bites(raw_bites)
          except ValueError:
              pass

    @staticmethod
    def publish_via_cmdline(index):
        logging.debug("publishing via cmdline...")
        command = "rostopic pub /interface/morsel_selection std_msgs/Int32 " + index
        publish_process = subprocess.Popen(command, shell=True)
        time.sleep(0.5)
        publish_process.kill()
    
    @staticmethod
    def simulate_stream():
        count = 0
        
        while True:
            c = count / 5.0

            raw_bites = [[[16.9824+c,10.2672+c],4.77898+c,9.435234],\
                         [[22.4563+2*c,45.098-c],8.7098-c,9.7234],\
                         [[43.9824-c,30.2682+c],6.345+c,8.439]]

            time.sleep(1)
            logging.debug("LISTENING")
            MorselFeedListener.bites = MorselFeedListener.process_raw_bites(raw_bites)
            count = ((count + 1) % 3)