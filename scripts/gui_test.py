#!/usr/bin/env python

# To build the guis
from Tkinter import *
from PIL import Image as tkImage
from PIL import ImageTk, ImageDraw
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

# For robot
import adapy, argparse, logging, numpy, os, openravepy, prpy, rospy
from catkin.find_in_workspaces import find_in_workspaces
from ada_meal_scenario.actions.trajectory_actions import LookAtFace, LookAtPlate, Serve
from ada_meal_scenario.actions.get_morsal import GetMorsal

class FeedingApp:
    def __init__(self, master, image_topic=None, sim=False):
        
        self.sim = sim
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
        
        look_button = Button(control_frame, text="Look at Plate", command=self.look)
        look_button.grid(padx=5, pady=5, row=3, column=0, sticky='N')
        present_button = Button(control_frame, text="Present to User", command=self.present)
        present_button.grid(padx=5, pady=5, row=4, column=0, sticky='N')
        
        # Set up cv bridge
        self.bridge = CvBridge()
        
        # Set up event handlers
        master.bind('<Button-1>', self.clicked)
        
        self.cursor = None
        
        
    def image_listener(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        
        np_img = np.array(self.cv_img, dtype=np.float32)
        np_img = np.nan_to_num(np_img)
        np_img = cv2.normalize(np_img, np_img, 0, 1, cv2.NORM_MINMAX)*255.0
        tk_img = tkImage.fromarray(np_img, mode='F')
        self.img = tk_img
        
    def clicked(self, event):
        x, y = event.x, event.y
        w = event.widget
        if w.winfo_class() == 'Label':
            self.cursor = [x, y]
            self.clicked_img = self.draw_cursor(self.captured_img)
            self.clicked_photo = ImageTk.PhotoImage(self.clicked_img)
            self.image_label.configure(image=self.clicked_photo)
        
    
    def refresh_image(self):
        self.captured_img = self.img
        if self.cursor:
            self.clicked_img = self.draw_cursor(self.captured_img)
            self.clicked_photo = ImageTk.PhotoImage(self.clicked_img)
            self.image_label.configure(image=self.clicked_photo)
        else:
            self.captured_photo = ImageTk.PhotoImage(self.captured_img)
            self.image_label.configure(image=self.captured_photo)
    
    def draw_cursor(self,input_img):
        in_img = input_img.copy()
        draw = ImageDraw.Draw(in_img)
        draw.line((self.cursor[0]-10, self.cursor[1], self.cursor[0]+10, self.cursor[1]), fill=0)
        draw.line((self.cursor[0], self.cursor[1]-10, self.cursor[0], self.cursor[1]+10), fill=0)
        return in_img     
    
    def look(self):
        action = LookAtPlate()
        action.execute(self.robot.GetActiveManipulator())
        
    def present(self):
        action = Serve()
        action.execute(self.robot.GetActiveManipulator())
    
    def get_morsal(self, pt):
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
        camera_in_world = self.robot.GetLink('Camera_RGB_Frame').GetTransform()
        morsal_in_camera = numpy.eye(4)
        morsal_pose = [0, 0, 0] # Need to change this!
        morsal_in_camera[:3,3] = morsal_pose
        morsal_in_world = numpy.dot(camera_in_world, morsal_in_camera)
        morsal.SetTransform(morsal_in_world)
        
    def skewer(self):
        depth = self.cv_img[self.cursor[0], self.cursor[1]]
        if np.isnan(depth):
            print('Got a Nan depth! Please select another point.')
            return
        print 'Depth: %f' % (depth)
        self.get_morsal([self.cursor[1], self.cursor[0], depth])
        action = GetMorsal()
        action.execute(self.robot.GetActiveManipulator())
        
    def scoop(self):
        pass


def setup_robot(sim=False, viewer=None, debug=True):
    data_base_path = find_in_workspaces(
        search_dirs=['share'],
        project='ada_meal_scenario',
        path='data',
        first_match_only=True)
    if len(data_base_path) == 0:
        raise Exception('Unable to find environment path. Did you source devel/setup.bash?')

    env_path = os.path.join(data_base_path[0], 'environments', 'table.env.xml')
    
    # Initialize logging
    if debug:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
    else:
        openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()
    prpy.logger.initialize_logging()

    # Load the environment
    env, robot = adapy.initialize(attach_viewer=viewer, sim=sim, env_path=env_path)

    # Set the active manipulator on the robot
    robot.arm.SetActive()

    # Set initial robot position (looking at the plate)
    robot.arm.PlanToNamedConfiguration('ada_meal_scenario_lookingAtPlateConfiguration')

    # Put robot on the table in the right place
    robot_pose = numpy.array([[1., 0., 0., 0.409],
                              [0., 1., 0., 0.338],
                              [0., 0., 1., 0.795],
                              [0., 0., 0., 1.]])
    with env:
        robot.SetTransform(robot_pose)

    # Manually set iksolver for now.
    iksolver = openravepy.RaveCreateIkSolver(env,"NloptIK")
    robot.arm.SetIKSolver(iksolver)

    # Load the tool into the robot's hand
    tool = env.ReadKinBodyURI('objects/kinova_tool.kinbody.xml')
    env.Add(tool)
    ee_in_world = robot.arm.GetEndEffectorTransform()
    tool_in_ee = numpy.array([[ -1., 0.,  0., 0.],
                              [ 0.,  1., 0., -0.002],
                              [ 0.,  0.,  -1., -0.118],
                              [ 0.,  0.,  0., 1.]])
    tool_in_world = numpy.dot(ee_in_world, tool_in_ee)
    tool.SetTransform(tool_in_world)
    robot.Grab(tool)
    
    # Load the fork into the toolholder
    fork = env.ReadKinBodyURI('objects/fork.kinbody.xml')
    env.Add(fork)
    fork_in_hole = numpy.array([[1.,0.,0.,0.],
                                [0.,1.,0.,0.],
                                [0.,0.,1.,-0.03],
                                [0.,0.,0.,1.]])
    hole_in_tool = numpy.array([[0.,0.,1.,0.],
                                [0.,1.,0.,-0.0225],
                                [-1.,0.,0.,0.0408],
                                [0.,0.,0.,1.]])
    fork_in_tool = numpy.dot(hole_in_tool, fork_in_hole)                           
    fork_in_ee = numpy.dot(tool_in_ee, fork_in_tool)
    fork_in_world = numpy.dot(ee_in_world, fork_in_ee)
    fork.SetTransform(fork_in_world)
    robot.Grab(fork)
    
    return env, robot


        
        
if __name__ == '__main__':

    root = Tk()
    root.title('Feeding App')
    rospy.init_node('feeding_app', anonymous=True)
    
    parser = argparse.ArgumentParser('Ada meal scenario')
    parser.add_argument("--debug", action="store_true", help="Run with debug")
    parser.add_argument("--sim", action="store_true", help="Run with the robot in simulation")
    parser.add_argument("--viewer", type=str, default='InteractiveMarker', help="The viewer to load")
    parser.add_argument("--camera-sim", action="store_true", help="Simulate the camera")
    args = parser.parse_args(rospy.myargv()[1:]) # exclude roslaunch args

    env, robot = setup_robot(sim=args.sim, viewer=args.viewer, debug=args.debug)

    app = FeedingApp(root, image_topic='/camera/depth/image_rect', sim=args.sim)
    app.robot = robot
    app.env = env
    root.mainloop()
