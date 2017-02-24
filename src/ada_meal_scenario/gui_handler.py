# gui for interacting with the ada_meal_scenario
# used to select the method and user input device

import Tkinter
import tkFont
from functools import partial

from threading import Lock
import multiprocessing as mp
from Queue import Empty
import time
import signal 
import sys

default_bg_color = None

class GuiHandler(object):
    def __init__(self, get_gui_state_event, trial_starting_event, return_queue):
        """
        Initialize class to handle Gui
        Gui is intended to run as a separate process
        Subscripts to topic of morsel detection, weights for returns of 
        detected morsels, converts to robot frame, and adds to environment

        @param get_gui_state_event multiprocessing event that, when triggered, gui will add 
            the current selected options to the return queue
        @type get_gui_state_event multiprocessing.Event
        @param trial_starting_event multiprocessing event that, when triggered, signals to the
            gui that a trial started. Used to reset gui
        @type trial_starting_event multiprocessing.Event
        @param return_queue multiprocessing queue for gui to return its state
        @type return_queue multiprocessing.Queue
        """

        self.master = Tkinter.Tk()
        #self.method = None
        #self.ui_device = None
        self.all_buttons = dict()
        self.record_next_trial = False
        self.start_next_trial = False
        self.quit = False

        self.get_gui_state_event = get_gui_state_event
        self.trial_starting_event = trial_starting_event
        self.return_queue = return_queue

        self.default_font = tkFont.nametofont("TkDefaultFont")
        self.default_font.configure(size=14)
        self.master.option_add("*Font", self.default_font)

        global default_bg_color
        default_bg_color = self.master.cget("bg")

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

        self.start_frame = Tkinter.Frame(self.master)
        self.start_frame.grid(row=1, column=2, sticky=Tkinter.W+Tkinter.E+Tkinter.N+Tkinter.S)

        self.record_button = Tkinter.Button(self.start_frame, text="Record Next Trial", 
                                            command=self.record_button_callback)
        self.record_button.grid(sticky=Tkinter.W+Tkinter.E)
        self.start_button = Tkinter.Button(self.start_frame, text="Start Next Trial", 
                                            command=self.start_button_callback)
        self.start_button.grid(sticky=Tkinter.W+Tkinter.E)


        self.start_frame = Tkinter.Frame(self.master)
        self.start_frame.grid(row=1, column=0, sticky=Tkinter.W+Tkinter.E+Tkinter.S)
        self.quit_button = Tkinter.Button(self.start_frame, text="Quit", command=self.quit_button_callback)
        self.quit_button.grid(sticky=Tkinter.W+Tkinter.E)

        #self.all_buttons[arg] = b

        self.method='autonomous'
        self.ui_device='kinova'
        self.color_buttons()

    def mainloop(self):
        """
        Infinite loop for the Gui process
        """

        while True:
            self.master.update_idletasks()
            self.master.update()

            if self.get_gui_state_event.is_set():
                self.add_return_to_queue()
                self.get_gui_state_event.clear()

            if self.trial_starting_event.is_set():
                self.start_next_trial = False
                configure_button_not_selected(self.start_button)
                self.trial_starting_event.clear()
           
            time.sleep(0.01)


    def init_method_buttons(self, frame):
        """
        Add all buttons for all possible methods we may run

        @param frame tkinter frame to add the buttons to
        """
        label_font = self.default_font.copy()
        label_font.configure(weight='bold')
        self.method_label = Tkinter.Label(frame, text="Method: \n", font=label_font)
        self.method_label.grid(sticky=Tkinter.W+Tkinter.E)
        
        self.button_full_auton = self.init_button_with_callback(
                                    self.select_assistance_method,
                                    'autonomous',
                                    'Fully Autonomous',
                                    frame)
        self.button_direct_teleop =  self.init_button_with_callback(
                                        self.select_assistance_method,
                                        'direct', 
                                        'Direct Teleop', 
                                        frame)
        self.button_shared_auton_1 = self.init_button_with_callback(
                                        self.select_assistance_method,
                                        'shared_auton_always', 
                                        'Shared Auton Always On', 
                                        frame)
        self.button_shared_auton_2 = self.init_button_with_callback(
                                        self.select_assistance_method, 
                                        'shared_auton_prop', 
                                        'Shared Auton Proportional',
                                        frame)
        self.button_blend = self.init_button_with_callback(
                                        self.select_assistance_method, 
                                        'blend', 
                                        'Blend', 
                                        frame)


    def init_ui_device_buttons(self, frame):
        """
        Add all buttons for all possible ui devices available

        @param frame tkinter frame to add the buttons to
        """
        label_font = self.default_font.copy()
        label_font.configure(weight='bold')
        self.method_label = Tkinter.Label(frame, text="UI Device \n", font=label_font)
        self.method_label.grid(sticky=Tkinter.W+Tkinter.E)
        
        self.button_mouse = self.init_button_with_callback(
                                self.select_ui_device, 
                                'mouse', 
                                'Mouse',
                                frame)
        self.button_razer = self.init_button_with_callback(
                                self.select_ui_device, 
                                'hydra', 
                                'Razer Hydra',
                                frame)
        self.button_kinova = self.init_button_with_callback(
                                self.select_ui_device, 
                                'kinova', 
                                'Kinova USB',
                                frame)


    def init_button_with_callback(self, func, arg, label, frame):
        """
        Initialize a button with the specified callback function

        @param func callback function to call when button is pressed
        @param arg argument to sent to callback function when pressed
        @param label label for the button
        @type label string
        @param frame tkinter frake to add button to
        """
        callback = partial(func, arg)
        b = Tkinter.Button(frame, text=label, command=callback)
        b.grid(sticky=Tkinter.W+Tkinter.E)
        self.all_buttons[arg] = b

        return b

    def select_assistance_method(self, method):
        """
        Callback function to use for all assistance methods
        Sets the method and colors buttons

        @param method the method selected using the gui
        """
        self.method = method
        self.color_buttons()

    def select_ui_device(self, ui_device):
        """
        Callback function to use for all ui devices
        Sets the device and colors buttons

        @param ui_device the device selected using the gui
        """
        self.ui_device = ui_device
        self.color_buttons()

    def start_button_callback(self):
        """
        Callback function for the start trial button
        """
        self.start_next_trial = toggle_trial_button_callback(self.start_button, self.start_next_trial)

    def record_button_callback(self):
        """
        Callback function for the record trial button
        """
        self.record_next_trial = toggle_trial_button_callback(self.record_button, self.record_next_trial)

    def quit_button_callback(self):
        """
        Callback function for the quit button
        """
        self.quit = toggle_trial_button_callback(self.quit_button, self.quit)


    def add_return_to_queue(self):
        """
        Function for communicated with the main process. 
        Add all of the currently selected options to the return queue
        """
        curr_selected = self.get_selected_options()
        self.return_queue.put(curr_selected)


    def get_selected_options(self):
        """
        Create and return dictionary of all currently selected options
        """
        to_ret = dict()
        to_ret['start'] = self.start_next_trial
        to_ret['quit'] = self.quit
        to_ret['method'] = self.method
        to_ret['ui_device'] = self.ui_device
        to_ret['record'] = self.record_next_trial
        return to_ret

    def color_buttons(self):
        """
        Color all buttons correctly for the currently selected options
        """
        #reset all
        for key,b in self.all_buttons.iteritems():
            configure_button_not_selected(b)
        #set the selected items
        configure_button_selected(self.all_buttons[self.ui_device])
        configure_button_selected(self.all_buttons[self.method])


def toggle_trial_button_callback(button, curr_val):
    """
    Helper function for buttons that we want to change option when clicked again
    Used for start trial, record, and quit buttons
    """
    to_ret = not curr_val
    if to_ret:
        configure_button_selected(button)
    else:
        configure_button_not_selected(button)
    return to_ret

def configure_button_selected(button):
    """
    Color button to specify that is is a selected option
    """
    button.configure(bg="#cc0000", activebackground="#ff0000")

def configure_button_not_selected(button):
    """
    Color button to specify that is not a selected option
    """
    button.configure(bg=default_bg_color, activebackground="white")


def create_gui(get_gui_state_event, trial_starting_event, data_queue):
    """
    Create a gui handler and start the main loop

    @param get_gui_state_event multiprocessing event that, when triggered, gui will add 
        the current selected options to the return queue
    @type get_gui_state_event multiprocessing.Event
    @param trial_starting_event multiprocessing event that, when triggered, signals to the
        gui that a trial started. Used to reset gui
    @type trial_starting_event multiprocessing.Event
    @param return_queue multiprocessing queue for gui to return its state
    @type return_queue multiprocessing.Queue
    """
    gui = GuiHandler(get_gui_state_event, trial_starting_event, data_queue)
    signal.signal(signal.SIGTERM, lambda signum, stack_frame: sys.exit())

    gui.mainloop()
    

def start_gui_process():
    """
    Launches the gui process. Creates necessary multiprocessing data structures for gui
    """
    get_gui_state_event = mp.Event()
    trial_starting_event = mp.Event()
    data_queue = mp.Queue()
    p = mp.Process(target=create_gui, args=(get_gui_state_event, trial_starting_event, data_queue))
    p.daemon = True
    p.start()

    return get_gui_state_event, trial_starting_event, data_queue, p
    

def empty_queue(queue):
    """
    Remove all items from the queue

    @param queue queue to remove items from
    """
    while not queue.empty():
        try:
            queue.get_nowait()
        except Empty:
            break






