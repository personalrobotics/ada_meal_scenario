#!/usr/bin/env python

# To build the guis
from Tkinter import *
from PIL import Image as tkImage
from PIL import ImageTk
import ttk
import tkMessageBox

# Ros stuff for finding image paths
import rospkg
import rospy

# For converting ros image message to PIL Image
from sensor_msgs.msg import Image as rosImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import IPython

class FeedingApp:
    def __init__(self, master, image_topic=None):
        
        self.pkgpath = rospkg.RosPack().get_path('ada_meal_scenario')
        no_img_path = self.pkgpath + '/media/test.jpg'
        img = ImageTk.PhotoImage(tkImage.open(no_img_path))
        
        if image_topic:
            rospy.Subscriber(image_topic, rosImage, self.image_listener)
        
        # Set the geometry manager to be grid
        Grid.columnconfigure(master, 0, weight=1)
        
        # Set up image display frame
        self.image_frame = LabelFrame(master,text='Image Feed', padx=5, pady=5)
        self.image_frame.grid(padx=10, pady=10, row=0, column=0)
        self.image_label = Label(self.image_frame, image=img)
        self.image_label.image = img
        self.image_label.grid(padx=10, pady=10, row=0, rowspan=2, column=0, sticky='N')
        
        # Set up control buttons
        control_frame = LabelFrame(master, text="Controls", padx=5, pady=5)
        control_frame.grid(padx=10, pady=10, row=0, column=1)
        
        refresh_button = Button(control_frame, text="Refresh Image", command=self.refresh_image)
        refresh_button.grid(padx=5, pady=5, row=0, column=0, sticky='N')
        skewer_button = Button(control_frame, text="Skewer", command=self.skewer)
        skewer_button.grid(padx=5, pady=5, row=1, column=0, sticky='N')
        scoop_button = Button(control_frame, text="Scoop", command=self.scoop)
        scoop_button.grid(padx=5, pady=5, row=2, column=0, sticky='N')
        
        # Set up cv bridge
        self.bridge = CvBridge()
        
    def image_listener(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        
        np_img = np.array(cv_img, dtype=np.float32)
        np_img = np.nan_to_num(np_img)
        np_img = cv2.normalize(np_img, np_img, 0, 1, cv2.NORM_MINMAX)*255.0
        tk_img = tkImage.fromarray(np_img, mode='F')
        self.img = ImageTk.PhotoImage(tk_img)
        
    
    def refresh_image(self):
        self.copy_img = self.img
        self.image_label.configure(image=self.copy_img)
        
    def skewer(self):
        pass
    
    def scoop(self):
        pass
        
        
if __name__ == '__main__':
    
    root = Tk()
    root.title('Feeding App')
    rospy.init_node('feeding_app', anonymous=True)
    
    app = FeedingApp(root, image_topic='/camera/depth/image_rect')
    root.mainloop()
