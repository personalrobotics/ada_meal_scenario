# gui for interacting with the ada_meal_scenario
# used to select the method and user input device

import Tkinter
import tkFont
from functools import partial


class GuiHandler(object):
    def __init__(self):
        self.master = Tkinter.Tk()
        self.method = None
        self.ui_device = None

        self.default_font = tkFont.nametofont("TkDefaultFont")
        self.default_font.configure(size=14)
        self.master.option_add("*Font", self.default_font)

        #Tkinter.Grid.rowconfigure(self.master, 0, weight=10)
        #Tkinter.Grid.columnconfigure(self.master, 0, weight=10)

        self.method_frame = Tkinter.Frame(self.master)
        self.method_frame.grid(row=0, column=0, sticky=Tkinter.W+Tkinter.E+Tkinter.N+Tkinter.S)
        self.init_method_buttons(self.method_frame)

        empty_frame = Tkinter.Frame(self.master)
        empty_frame.grid(row=0, column=1, sticky=Tkinter.W+Tkinter.E+Tkinter.N+Tkinter.S)
        empty_label = Tkinter.Label(empty_frame, text="     ")
        empty_label.grid(sticky=Tkinter.W+Tkinter.E)

        self.UI_frame = Tkinter.Frame(self.master)
        self.UI_frame.grid(row=0, column=2, sticky=Tkinter.W+Tkinter.E+Tkinter.N+Tkinter.S)
        self.init_ui_device_buttons(self.UI_frame)


    def init_method_buttons(self, frame):
        label_font = self.default_font.copy()
        label_font.configure(weight='bold')
        self.method_label = Tkinter.Label(frame, text="Method: \n", font=label_font)
        self.method_label.grid(sticky=Tkinter.W+Tkinter.E)
        
        self.button_full_auton = self.init_assistance_method_button('autonomous', 'Fully Autonomous', frame)
        self.button_direct_teleop = self.init_assistance_method_button('direct', 'Direct Teleop', frame)
        self.button_shared_auton_1 = self.init_assistance_method_button('shared_auton_always', 'Shared Auton Always On', frame)
        self.button_shared_auton_2 = self.init_assistance_method_button('shared_auton_prop', 'Shared Auton Proportional', frame)
        self.button_blend = self.init_assistance_method_button('blend', 'Blend', frame)


    def init_assistance_method_button(self, method, label, frame):
        callback = partial(self.select_assistance_method, method)
        b = Tkinter.Button(frame, text=label, command=callback)
        b.grid(sticky=Tkinter.W+Tkinter.E)

        return b

    def select_assistance_method(self, method):
        self.method = method
        print method



    def init_ui_device_buttons(self, frame):
        label_font = self.default_font.copy()
        label_font.configure(weight='bold')
        self.method_label = Tkinter.Label(frame, text="UI Device \n", font=label_font)
        self.method_label.grid(sticky=Tkinter.W+Tkinter.E)
        
        self.button_mouse = self.init_ui_device_button('mouse', 'Mouse', frame)
        self.button_razer = self.init_ui_device_button('razer', 'Razer Hydra', frame)
        self.button_kinova = self.init_ui_device_button('kinova', 'Kinova USB', frame)


    def init_ui_device_button(self, ui_device, label, frame):
        callback = partial(self.select_ui_device, ui_device)
        b = Tkinter.Button(frame, text=label, command=callback)
        b.grid(sticky=Tkinter.W+Tkinter.E)

        return b

    def select_ui_device(self, ui_device):
        self.ui_device = ui_device
        print ui_device

    




    


