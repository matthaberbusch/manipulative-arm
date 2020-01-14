#!/usr/bin/python
# -*- coding: utf-8 -*-
import Tkinter as tk
import ttk

# import roslaunch

import os
import time

# Length of the lag between sending and the machine receiving emssage
delay_len = 0

# List of options for preset values, to be associated with related values in each function
options = ['Peg', 'Bottle_Cap', 'Cutting', 'Tool', 'Task']
selected_option = 2 

class Application(tk.Frame):

    def __init__(self, master=None):

        # super().__init__(master)

        tk.Frame.__init__(self, master)
        self.master = master  # might not be needed?

        # self.pack()

        self.create_widgets()

    def create_widgets(self):

        # self.hi_there = tk.Button(self)
        # self.hi_there["text"] = "Hello World\n(click me)"
        # self.hi_there["command"] = self.say_hi
        # self.hi_there.pack(side="top")

        self.master.title('Skills GUI')
        self.tab_control = ttk.Notebook(self.master)
        self.motion_control = ttk.Frame(self.tab_control)
        self.orientation_control = ttk.Frame(self.tab_control)
        self.wiggle_control = ttk.Frame(self.tab_control)
        self.misc_commands = ttk.Frame(self.tab_control)

        self.tab_control.add(self.motion_control, text='Motion Control')
        self.tab_control.add(self.orientation_control, text='Orientation Control')
        self.tab_control.add(self.wiggle_control, text='Wiggle Control')
        self.tab_control.add(self.misc_commands, text='Misc. Commands')

    # Pack and display the tabs

        self.tab_control.pack(expand=1, fill='both')

    # Font defined to be larger
    # myFont = tk.font.Font(size=20, weight='bold')

        # Config for the button layout

        self.quit = tk.Button(self, text='QUIT', fg='red',
                              command=self.master.destroy,
                              font='Courier 20 bold')

    # self.quit['font'] = myFont

        self.quit.grid(row=0, column=0, sticky=tk.W)

        self.entry = tk.Entry(self, font='Courier 20 bold')
        self.entry.grid(row=0, column=1, sticky=tk.NSEW)

        self.clear_text = tk.Button(self, text='Clear Text', fg='blue',
                                    command=self.clear_text,
                                    font='Courier 20 bold')
        self.clear_text.grid(row=0, column=2, sticky=tk.E)

        #Parameter Dropdown Menu

        # This variable stores the value of the set of parameters needed, will send it to the ptfl
        self.parameter_set = tk.StringVar(self)
        self.parameter_set.set(options[selected_option]) # Change the number in the brackets for which one 

        # Comented out is if I want to link a function to the update of the menu
        self.drop = tk.OptionMenu(self, self.parameter_set, *options) #, font='Courier 20 bold') # command=self.updated_menu, 
        self.drop.grid(row=0,column=4,sticky=tk.W+tk.E)
        self.drop.config(font=('courier',(20)))

        self.param_label = \
            tk.Label(self,
                     text=' Preset Parameters:',
                     font='Courier 20 bold')
        self.param_label.grid(row=0, column=3,
                sticky=tk.NSEW, pady=5, padx=2)


        # Force Moment Accomodation (Present in both motion and orientation control tabs)
        # MOTION CONTROL FMA
        self.force_moment_acc_label = tk.Label(self.motion_control,
                text='Limp Mode:',
                font='Courier 20 bold')
        self.force_moment_acc_label.grid(row=1, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.force_moment_acc = tk.Button(self.motion_control,
                text='text in', fg='blue',
                command=self.force_moment_acc, font='Courier 20 bold')
        self.force_moment_acc.grid(row=1, column=1, sticky=tk.NSEW,
                                   pady=5, padx=2)

        # Preset Value 1

        self.force_moment_acc_preset_1 = tk.Button(self.motion_control,
                text='10 sec', fg='blue',
                command=self.force_moment_acc_preset_1,
                font='Courier 20 bold')
        self.force_moment_acc_preset_1.grid(row=1, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset Value 2

        self.force_moment_acc_preset_2 = tk.Button(self.motion_control,
                text='5 sec', fg='blue',
                command=self.force_moment_acc_preset_2,
                font='Courier 20 bold')
        self.force_moment_acc_preset_2.grid(row=1, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # ORIENTATION CONTROL FMA
        self.force_moment_acc_label_oc = tk.Label(self.orientation_control,
                text='Limp Mode:',
                font='Courier 20 bold')
        self.force_moment_acc_label_oc.grid(row=1, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.force_moment_acc_oc = tk.Button(self.orientation_control,
                text='text in', fg='blue',
                command=self.force_moment_acc, font='Courier 20 bold')
        self.force_moment_acc_oc.grid(row=1, column=1, sticky=tk.NSEW,
                                   pady=5, padx=2)

        # Preset Value 1
        self.force_moment_acc_preset_1_oc = tk.Button(self.orientation_control,
                text='10 sec', fg='blue',
                command=self.force_moment_acc_preset_1,
                font='Courier 20 bold')
        self.force_moment_acc_preset_1_oc.grid(row=1, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset Value 2
        self.force_moment_acc_preset_2_oc = tk.Button(self.orientation_control,
                text='5 sec', fg='blue',
                command=self.force_moment_acc_preset_2,
                font='Courier 20 bold')
        self.force_moment_acc_preset_2_oc.grid(row=1, column=2,
                sticky=tk.NSEW, pady=5, padx=2)


        # Orientation Targeting w/ Effort Limit

        # original Orientation (in the x)
        self.orientation_targ_effort_limit_label_x = \
            tk.Label(self.orientation_control,
                     text='Orientation Targeting w/ Effort Limit in x:',
                     font='Courier 20 bold')
        self.orientation_targ_effort_limit_label_x.grid(row=2, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.orientation_targ_effort_limit_x = \
            tk.Button(self.orientation_control, text='text in', fg='blue',
                      command=self.orientation_targ_effort_limit_x,
                      font='Courier 20 bold')
        self.orientation_targ_effort_limit_x.grid(row=2, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1
        self.orientation_targ_effort_limit_pre_1_x = \
            tk.Button(self.orientation_control, text='0.2 rads', fg='blue',
                      command=self.orientation_targ_effort_limit_pre_1_x,
                      font='Courier 20 bold')
        self.orientation_targ_effort_limit_pre_1_x.grid(row=2, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2
        self.orientation_targ_effort_limit_pre_2_x = \
            tk.Button(self.orientation_control, text='-0.2 rads', fg='blue',
                      command=self.orientation_targ_effort_limit_pre_2_x,
                      font='Courier 20 bold')
        self.orientation_targ_effort_limit_pre_2_x.grid(row=2, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        # Orientation Targeting w/ Effort Limit (In the y)
        self.orientation_targ_effort_limit_label_y = \
            tk.Label(self.orientation_control,
                     text='Orientation Targeting in y:',
                     font='Courier 20 bold')
        self.orientation_targ_effort_limit_label_y.grid(row=3, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.orientation_targ_effort_limit_y = \
            tk.Button(self.orientation_control, text='text in', fg='blue',
                      command=self.orientation_targ_effort_limit_y,
                      font='Courier 20 bold')
        self.orientation_targ_effort_limit_y.grid(row=3, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1
        self.orientation_targ_effort_limit_pre_1_y = \
            tk.Button(self.orientation_control, text='0.2 rads', fg='blue',
                      command=self.orientation_targ_effort_limit_pre_1_y,
                      font='Courier 20 bold')
        self.orientation_targ_effort_limit_pre_1_y.grid(row=3, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.orientation_targ_effort_limit_pre_2_y = \
            tk.Button(self.orientation_control, text='-0.2 rads', fg='blue',
                      command=self.orientation_targ_effort_limit_pre_2_y,
                      font='Courier 20 bold')
        self.orientation_targ_effort_limit_pre_2_y.grid(row=3, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        # Orientation Targeting w/ Effort Limit (In the z)
        self.orientation_targ_effort_limit_label_z = \
            tk.Label(self.orientation_control,
                     text='Orientation Targeting in z:',
                     font='Courier 20 bold')
        self.orientation_targ_effort_limit_label_z.grid(row=4, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.orientation_targ_effort_limit_z = \
            tk.Button(self.orientation_control, text='text in', fg='blue',
                      command=self.orientation_targ_effort_limit_z,
                      font='Courier 20 bold')
        self.orientation_targ_effort_limit_z.grid(row=4, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1
        self.orientation_targ_effort_limit_pre_1_z = \
            tk.Button(self.orientation_control, text='pi/2 rads', fg='blue',
                      command=self.orientation_targ_effort_limit_pre_1_z,
                      font='Courier 20 bold')
        self.orientation_targ_effort_limit_pre_1_z.grid(row=4, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.orientation_targ_effort_limit_pre_2_z = \
            tk.Button(self.orientation_control, text='-pi/2 rads', fg='blue',
                      command=self.orientation_targ_effort_limit_pre_2_z,
                      font='Courier 20 bold')
        self.orientation_targ_effort_limit_pre_2_z.grid(row=4, column=3,
                sticky=tk.NSEW, pady=5, padx=2)


        # Position Targeting w/ Effort Limit

        # PTEL x
        self.position_targ_effort_limit_label_x = \
            tk.Label(self.motion_control,
                     text='Position Targeting w/ Effort Limit in x:',
                     font='Courier 20 bold')
        self.position_targ_effort_limit_label_x.grid(row=2, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.position_targ_effort_limit_x = tk.Button(self.motion_control,
                text='text in', fg='blue',
                command=self.position_targ_effort_limit_x,
                font='Courier 20 bold')
        self.position_targ_effort_limit_x.grid(row=2, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1

        self.position_targ_effort_limit_pre_1_x = \
            tk.Button(self.motion_control, text='0.03 m', fg='blue',
                      command=self.position_targ_effort_limit_pre_1_x,
                      font='Courier 20 bold')
        self.position_targ_effort_limit_pre_1_x.grid(row=2, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.position_targ_effort_limit_pre_2_x = \
            tk.Button(self.motion_control, text='-0.03 m', fg='blue',
                      command=self.position_targ_effort_limit_pre_2_x,
                      font='Courier 20 bold')
        self.position_targ_effort_limit_pre_2_x.grid(row=2, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        # PTEL y
        self.position_targ_effort_limit_label_y = \
            tk.Label(self.motion_control,
                     text='PTEL in y:',
                     font='Courier 20 bold')
        self.position_targ_effort_limit_label_y.grid(row=3, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.position_targ_effort_limit_y = tk.Button(self.motion_control,
                text='text in', fg='blue',
                command=self.position_targ_effort_limit_y,
                font='Courier 20 bold')
        self.position_targ_effort_limit_y.grid(row=3, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1

        self.position_targ_effort_limit_pre_1_y = \
            tk.Button(self.motion_control, text='0.03 m', fg='blue',
                      command=self.position_targ_effort_limit_pre_1_y,
                      font='Courier 20 bold')
        self.position_targ_effort_limit_pre_1_y.grid(row=3, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.position_targ_effort_limit_pre_2_y = \
            tk.Button(self.motion_control, text='-0.03 m', fg='blue',
                      command=self.position_targ_effort_limit_pre_2_y,
                      font='Courier 20 bold')
        self.position_targ_effort_limit_pre_2_y.grid(row=3, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        # PTEL z
        self.position_targ_effort_limit_label_z = \
            tk.Label(self.motion_control,
                     text='PTEL in z:',
                     font='Courier 20 bold')
        self.position_targ_effort_limit_label_z.grid(row=4, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.position_targ_effort_limit_z = tk.Button(self.motion_control,
                text='text in', fg='blue',
                command=self.position_targ_effort_limit_z,
                font='Courier 20 bold')
        self.position_targ_effort_limit_z.grid(row=4, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1

        self.position_targ_effort_limit_pre_1_z = \
            tk.Button(self.motion_control, text='0.03 m', fg='blue',
                      command=self.position_targ_effort_limit_pre_1_z,
                      font='Courier 20 bold')
        self.position_targ_effort_limit_pre_1_z.grid(row=4, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.position_targ_effort_limit_pre_2_z = \
            tk.Button(self.motion_control, text='-0.03 m', fg='blue',
                      command=self.position_targ_effort_limit_pre_2_z,
                      font='Courier 20 bold')
        self.position_targ_effort_limit_pre_2_z.grid(row=4, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        # Torsional Wiggle Pull

        self.torsional_wiggle_pull_label = \
            tk.Label(self.wiggle_control, text='Torsional Wiggle Pull:'
                     , font='Courier 20 bold')
        self.torsional_wiggle_pull_label.grid(row=0, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.torsional_wiggle_pull = tk.Button(self.wiggle_control,
                text='text in', fg='blue',
                command=self.torsional_wiggle_pull,
                font='Courier 20 bold')
        self.torsional_wiggle_pull.grid(row=0, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1

        self.torsional_wiggle_pull_pre_1 = \
            tk.Button(self.wiggle_control, text='3 sec', fg='blue',
                      command=self.torsional_wiggle_pull_pre_1,
                      font='Courier 20 bold')
        self.torsional_wiggle_pull_pre_1.grid(row=0, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.torsional_wiggle_pull_pre_2 = \
            tk.Button(self.wiggle_control, text='5 sec', fg='blue',
                      command=self.torsional_wiggle_pull_pre_2,
                      font='Courier 20 bold')
        self.torsional_wiggle_pull_pre_2.grid(row=0, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        # Torsional Wiggle Push

        self.torsional_wiggle_push_label = \
            tk.Label(self.wiggle_control, text='Torsional Wiggle Push:'
                     , font='Courier 20 bold')
        self.torsional_wiggle_push_label.grid(row=1, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.torsional_wiggle_push = tk.Button(self.wiggle_control,
                text='text in', fg='blue',
                command=self.torsional_wiggle_push,
                font='Courier 20 bold')
        self.torsional_wiggle_push.grid(row=1, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1

        self.torsional_wiggle_push_pre_1 = \
            tk.Button(self.wiggle_control, text='3 sec', fg='blue',
                      command=self.torsional_wiggle_push_pre_1,
                      font='Courier 20 bold')
        self.torsional_wiggle_push_pre_1.grid(row=1, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.torsional_wiggle_push_pre_2 = \
            tk.Button(self.wiggle_control, text='5 sec', fg='blue',
                      command=self.torsional_wiggle_push_pre_2,
                      font='Courier 20 bold')
        self.torsional_wiggle_push_pre_2.grid(row=1, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        # Translational Wiggle Pull

        self.translational_wiggle_pull_label = \
            tk.Label(self.wiggle_control,
                     text='Translational Wiggle Pull:',
                     font='Courier 20 bold')
        self.translational_wiggle_pull_label.grid(row=2, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.translational_wiggle_pull = tk.Button(self.wiggle_control,
                text='text in', fg='blue',
                command=self.translational_wiggle_pull,
                font='Courier 20 bold')
        self.translational_wiggle_pull.grid(row=2, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1

        self.translational_wiggle_pull_pre_1 = \
            tk.Button(self.wiggle_control, text='3 sec', fg='blue',
                      command=self.translational_wiggle_pull_pre_1,
                      font='Courier 20 bold')
        self.translational_wiggle_pull_pre_1.grid(row=2, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.translational_wiggle_pull_pre_2 = \
            tk.Button(self.wiggle_control, text='5 sec', fg='blue',
                      command=self.translational_wiggle_pull_pre_2,
                      font='Courier 20 bold')
        self.translational_wiggle_pull_pre_2.grid(row=2, column=3,
                sticky=tk.NSEW, pady=5, padx=2)

        # Translational Wiggle Push

        self.translational_wiggle_push_label = \
            tk.Label(self.wiggle_control,
                     text='Translational Wiggle Push:',
                     font='Courier 20 bold')
        self.translational_wiggle_push_label.grid(row=3, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        self.translational_wiggle_push = tk.Button(self.wiggle_control,
                text='text in', fg='blue',
                command=self.translational_wiggle_push,
                font='Courier 20 bold')
        self.translational_wiggle_push.grid(row=3, column=1,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 1

        self.translational_wiggle_push_pre_1 = \
            tk.Button(self.wiggle_control, text='3 sec', fg='blue',
                      command=self.translational_wiggle_push_pre_1,
                      font='Courier 20 bold')
        self.translational_wiggle_push_pre_1.grid(row=3, column=2,
                sticky=tk.NSEW, pady=5, padx=2)

        # Preset 2

        self.translational_wiggle_push_pre_2 = \
            tk.Button(self.wiggle_control, text='5 sec', fg='blue',
                      command=self.translational_wiggle_push_pre_2,
                      font='Courier 20 bold')
        self.translational_wiggle_push_pre_2.grid(row=3, column=3,
                sticky=tk.NSEW, pady=5, padx=2)


        # Button for Zero Forces
        self.zero_forces = tk.Button(self.misc_commands,
                text='Zero all Forces', fg='blue',
                command=self.zero_forces, font='Courier 20 bold')
        self.zero_forces.grid(row=0, column=0,
                sticky=tk.NSEW, pady=5, padx=2)

        # Button for Setting Task Frame
        self.set_task_frame = tk.Button(self.misc_commands,
                text='Set Task Frame', fg='blue',
                command=self.set_task_frame, font='Courier 20 bold')
        self.set_task_frame.grid(row=1, column=0,
                sticky=tk.NSEW, pady=5, padx=2)


        # Show all buttons

        self.pack()

    # def say_hi(self):
    #     print 'hi there, everyone!'

    def clear_text(self):

        # print("Here, i run a command using os, skill 1")

        self.entry.delete(0, 'end')

        # os.system("rosrun irb120_accomodation_control accommodation_controller") 

    def force_moment_acc(self):

        # print("Here, i run a command using os, skill 2")

        command = \
            'rosrun behavior_algorithms force_moment_accommodation'
        run_time = self.parse_entry()
        if run_time > 0:
            command = command + ' _run_time:=%f' % run_time

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def force_moment_acc_preset_1(self):

        # print("Here, i run a command using os, skill 2")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms force_moment_accommodation _run_time:=10'
                  )  
        time.sleep(delay_len)

    def force_moment_acc_preset_2(self):

        # print("Here, i run a command using os, skill 1")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms force_moment_accommodation _run_time:=5'
                  )  
        time.sleep(delay_len)

    
    # New example 
    # rosrun behavior_algorithms orientation_targeting_effort_limiting_y _target_orientation:=0.25

    # In the x
    def orientation_targ_effort_limit_x(self):

        # print("Here, i run a command using os, skill 2")

        command = \
            'rosrun behavior_algorithms orientation_targeting_effort_limiting_x _param_set:=' + self.parameter_set.get()
        orientation = self.parse_entry()
        if orientation != 0:
            command = command + ' _target_orientation:=%f' % orientation

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def orientation_targ_effort_limit_pre_1_x(self):

        # print("Here, i run a command using os, skill 2")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms orientation_targeting_effort_limiting_x _target_orientation:=0.2 _param_set:=' + self.parameter_set.get()
                  )  
        time.sleep(delay_len)

    def orientation_targ_effort_limit_pre_2_x(self):

        # print("Here, i run a command using os, skill 2")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms orientation_targeting_effort_limiting_x _target_orientation:=-0.2 _param_set:=' + self.parameter_set.get()
                  )  
        time.sleep(delay_len)


    def orientation_targ_effort_limit_y(self):

        # print("Here, i run a command using os, skill 2")

        command = \
            'rosrun behavior_algorithms orientation_targeting_effort_limiting_y _param_set:=' + self.parameter_set.get()
        orientation = self.parse_entry()
        if orientation != 0:
            command = command + ' _target_orientation:=%f' % orientation

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def orientation_targ_effort_limit_pre_1_y(self):

        # print("Here, i run a command using os, skill 2")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms orientation_targeting_effort_limiting_y _target_orientation:=0.2 _param_set:=' + self.parameter_set.get()
                  )  
        time.sleep(delay_len)

    def orientation_targ_effort_limit_pre_2_y(self):

        # print("Here, i run a command using os, skill 2")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms orientation_targeting_effort_limiting_y _target_orientation:=-0.2 _param_set:=' + self.parameter_set.get()
                  )  
        time.sleep(delay_len)


    def orientation_targ_effort_limit_z(self):

        # print("Here, i run a command using os, skill 2")

        command = \
            'rosrun behavior_algorithms orientation_targeting_effort_limiting_z _param_set:=' + self.parameter_set.get()
        orientation = self.parse_entry()
        if orientation != 0:
            command = command + ' _target_orientation:=%f' % orientation

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def orientation_targ_effort_limit_pre_1_z(self):

        # print("Here, i run a command using os, skill 2")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms orientation_targeting_effort_limiting_z _target_orientation:=1.57 _param_set:=' + self.parameter_set.get()
                  )  
        time.sleep(delay_len)

    def orientation_targ_effort_limit_pre_2_z(self):

        # print("Here, i run a command using os, skill 2")

        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms orientation_targeting_effort_limiting_z _target_orientation:=-1.57 _param_set:=' + self.parameter_set.get()
                  )  
        time.sleep(delay_len)


    # rosrun behavior_algorithms orientation_targeting_with_torque_limiting _target_orientation:=2
    
    # PTEL in x
    # sample command: rosrun behavior_algorithms position_targeting_effort_limiting_x _target_distance:=-0.02
    def position_targ_effort_limit_x(self):

        # print("Here, i run a command using os, skill 2")

        command = \
            'rosrun behavior_algorithms position_targeting_effort_limiting_x'
        target_distance = self.parse_entry()
        if target_distance != 0:
            command = command + ' _target_distance:=%f' \
                % target_distance

        print(command)
        command = command + ' _param_set:=' + self.parameter_set.get()
        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def position_targ_effort_limit_pre_1_x(self):
        time.sleep(delay_len)
        command = 'rosrun behavior_algorithms position_targeting_effort_limiting_x _target_distance:=0.03 _param_set:=' + self.parameter_set.get()
        os.system(command)  
        time.sleep(delay_len)

    def position_targ_effort_limit_pre_2_x(self):
        time.sleep(delay_len)
        command = 'rosrun behavior_algorithms position_targeting_effort_limiting_x _target_distance:=-0.03 _param_set:=' + self.parameter_set.get()
        os.system(command)
        time.sleep(delay_len)

    # PTEL in y
    def position_targ_effort_limit_y(self):

        # print("Here, i run a command using os, skill 2")

        command = \
            'rosrun behavior_algorithms position_targeting_effort_limiting_y'
        target_distance = self.parse_entry()
        if target_distance != 0:
            command = command + ' _target_distance:=%f' \
                % target_distance

        print(command)
        command = command + ' _param_set:=' + self.parameter_set.get()
        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def position_targ_effort_limit_pre_1_y(self):
        time.sleep(delay_len)
        command = 'rosrun behavior_algorithms position_targeting_effort_limiting_y _target_distance:=0.03 _param_set:=' + self.parameter_set.get()
        os.system(command)  
        time.sleep(delay_len)

    def position_targ_effort_limit_pre_2_y(self):
        time.sleep(delay_len)
        command = 'rosrun behavior_algorithms position_targeting_effort_limiting_y _target_distance:=-0.03 _param_set:=' + self.parameter_set.get()
        os.system(command)
        time.sleep(delay_len)

    # PTEL in z
    def position_targ_effort_limit_z(self):

        # print("Here, i run a command using os, skill 2")

        command = \
            'rosrun behavior_algorithms freeze_position_targeting_effort_limiting_z'
        target_distance = self.parse_entry()
        if target_distance != 0:
            command = command + ' _target_distance:=%f' \
                % target_distance

        print(command)
        command = command + ' _param_set:=' + self.parameter_set.get()
        time.sleep(delay_len)
        # add this in before, so that the system is in compliant mode
        os.system('rosservice call /freeze_service')
        os.system(command)  
        time.sleep(delay_len)

    def position_targ_effort_limit_pre_1_z(self):
        time.sleep(delay_len)
        # add this in before, so that the system is in compliant mode
        os.system('rosservice call /freeze_service')
        command = 'rosrun behavior_algorithms freeze_position_targeting_effort_limiting_z _target_distance:=0.03 _param_set:=' + self.parameter_set.get()
        os.system(command)  
        time.sleep(delay_len)

    def position_targ_effort_limit_pre_2_z(self):
        time.sleep(delay_len)
        # add this in before, so that the system is in compliant mode
        os.system('rosservice call /freeze_service')
        command = 'rosrun behavior_algorithms freeze_position_targeting_effort_limiting_z _target_distance:=-0.03 _param_set:=' + self.parameter_set.get()
        os.system(command)
        time.sleep(delay_len)


    # rosrun behavior_algorithms torsional_wiggle_pull _wiggle_time:=6

    def torsional_wiggle_pull(self):

        # print("Here, i run a command using os, skill 2")

        command = 'rosrun behavior_algorithms torsional_wiggle_pull'
        run_time = self.parse_entry()
        if run_time > 0:
            command = command + ' _wiggle_time:=%f' % run_time

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def torsional_wiggle_pull_pre_1(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms torsional_wiggle_pull _wiggle_time:=3'
                  )  
        time.sleep(delay_len)

    def torsional_wiggle_pull_pre_2(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms torsional_wiggle_pull _wiggle_time:=5'
                  )  
        time.sleep(delay_len)

    # rosrun behavior_algorithms torsional_wiggle_push _wiggle_time:=6

    def torsional_wiggle_push(self):

        # print("Here, i run a command using os, skill 2")

        command = 'rosrun behavior_algorithms torsional_wiggle_push'
        run_time = self.parse_entry()
        if run_time > 0:
            command = command + ' _wiggle_time:=%f' % run_time

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def torsional_wiggle_push_pre_1(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms torsional_wiggle_push _wiggle_time:=3'
                  )  
        time.sleep(delay_len)

    def torsional_wiggle_push_pre_2(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms torsional_wiggle_push _wiggle_time:=5'
                  )  
        time.sleep(delay_len)

    # rosrun behavior_algorithms translational_wiggle_pull _wiggle_time:=6

    def translational_wiggle_pull(self):

        # print("Here, i run a command using os, skill 2")

        command = 'rosrun behavior_algorithms translational_wiggle_pull'
        run_time = self.parse_entry()
        if run_time > 0:
            command = command + ' _wiggle_time:=%f' % run_time

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def translational_wiggle_pull_pre_1(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms translational_wiggle_pull _wiggle_time:=3'
                  )  
        time.sleep(delay_len)

    def translational_wiggle_pull_pre_2(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms translational_wiggle_pull _wiggle_time:=5'
                  )  
        time.sleep(delay_len)

    # rosrun behavior_algorithms translational_wiggle_push _wiggle_time:=6

    def translational_wiggle_push(self):

        # print("Here, i run a command using os, skill 2")

        command = 'rosrun behavior_algorithms translational_wiggle_push'
        run_time = self.parse_entry()
        if run_time > 0:
            command = command + ' _wiggle_time:=%f' % run_time

        # print(command)

        time.sleep(delay_len)
        os.system(command)  
        time.sleep(delay_len)

    def translational_wiggle_push_pre_1(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms translational_wiggle_push _wiggle_time:=3'
                  )  
        time.sleep(delay_len)

    def translational_wiggle_push_pre_2(self):
        time.sleep(delay_len)
        os.system('rosrun behavior_algorithms translational_wiggle_push _wiggle_time:=5'
                  )  
        time.sleep(delay_len)

    def zero_forces(self):
        time.sleep(delay_len)
        os.system('rosservice call /robotiq_ft_sensor_acc "command_id: 8"')  
        time.sleep(delay_len)

    def set_task_frame(self):
        time.sleep(delay_len)
        os.system('rosservice call /task_frame_service')  
        time.sleep(delay_len)

    def parse_entry(self):
        # print("output from text box")
        # print(self.entry.get())
        # Clear text in here potentially? kept for now to show what was the last input
        try:
            return float(self.entry.get())
        except:
            return 0


def main():
    root = tk.Tk()
    app = Application(master=root)

    # os.system("rosrun robotiq_ft_sensor rq_sensor")
    # os.system("roslaunch force_reflector cybernet_testing_setup.launch ")

    app.mainloop()


if __name__ == '__main__':
    main()


            