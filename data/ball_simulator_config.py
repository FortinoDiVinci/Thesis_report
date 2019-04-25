#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 16:34:04 2019

@author: Vincent
"""

import sys
import numpy as np

#####################
# CLASSES & FUNCTIONS
#####################

class Session:
    def __init__(self, name = 'sessions', duration = None, config = None, event = None):
        if duration != None:
            self.duration = duration
        if config != None:
            self.initial_config = config
        if event != None:
            if isinstance(event, (list, tuple, np.ndarray)):
                self.nb_event = len(event)
            else:
                self.nb_event = 1
            self.event = event
            
        self.name = name
            
    def __str__(self):
        s = ("\t - %s:\n" %self.name)
        if hasattr(self, 'duration'):
            s = s + ("\t\tduration: %.2f\n" %self.duration)
        if hasattr(self, 'initial_config'):
            s = s + ("\t\tinitial_config:\n")
            s = s + str(self.initial_config) + '\n'
        if hasattr(self, 'event'):
            s = s + "\t\tevents:\n"
            if self.nb_event == 1:
                s = s + str(self.event) + '\n'
            else:
                for event in self.event:
                    s = s + str(event) + '\n'
        return s

class Config:
    def __init__(self, t_height = None, coeff = None, gravity = 9.807):
        if t_height != None:
            self.target_height = t_height
        if coeff != None:
            self.restitution_coefficient = coeff
        if gravity != 9.807:
            self.gravity = gravity
            
    def __str__(self, event = False):
        s = ''
        if event:
            t = '\t\t'
        else:
            t = ''
        if hasattr(self, 'target_height'):
            s = t + ("\t\t\ttarget_height: %.2f\n" %self.target_height)
        if hasattr(self, 'restitution_coefficient'):
            s = s + t + ("\t\t\trestitution: %.2f\n" %self.restitution_coefficient)
        if hasattr(self, 'gravity'):
            s = s + t + ("\t\t\tgravity: %.2f\n" %self.gravity)
        return s
                
class Event:
    def __init__(self, name, trigger, value, config = None):
        self.name = name
        self.trigger = trigger
        self.value = value
        if config != None:
            self.config = config
     
    def __str__(self):
        s = ("\t\t\t%s\n" %self.name)
        s = s + ("\t\t\t\ttrigger: %s\n" %self.trigger)
        s = s + ("\t\t\t\tvalue: %s\n" %self.value)
        s = s +("\t\t\t\tconfig:\n")
        if hasattr(self, 'config'):
            s = s + self.config.__str__(event=True)
        return s

        
#####################
#       MAIN
#####################

if len(sys.argv) < 2:
    raise OSError("Please specify file name for the config file as argument.")

try:
    file_name = str(sys.argv[1])
except ValueError:
    print("Please enter a valid name for the file as argument.")
else:
    if file_name.lower().endswith('.yaml'):
        pass
    else:
        file_name = file_name + '.yaml'

valid_nb = False

while(valid_nb == False):

    try:
        nb_session = int(raw_input("How many sessions do you want to create?\n"))
    except ValueError:
        print("Please provide a valid number")
    else:
        valid_nb = True

print("If a field should stay empty type anything but a number")

ses_name = str(raw_input("Please choose session base name: "))
   
session = []
     
for sess in xrange(nb_session):
    
    #################
    # Initial Config
    #################
    
    try:
        sess_dur = float(raw_input("Session %s:\nEnter duration: " %(sess+1)))
    except ValueError:
        print("No duration provided")
        
    print("Initial configuration:")
    try:
        height = float(raw_input("Enter target height: "))
    except ValueError:
        print("No target height provided")  
        height = None
    try:
        coeff = float(raw_input("Enter restitution coefficient: "))
    except ValueError:
        print("No Restitution coefficient provided")  
        coeff = None
    try:
        gravity = float(raw_input("Enter gravity: "))
    except ValueError:
        print("No gravity provided")
        gravity = 9.807
    
    init_config = Config(t_height = height, coeff = coeff, gravity = gravity)
        
    #################
    # Event
    #################     
    
    try:
        nb_event = int(raw_input("How many event are needed?\n"))
    except ValueError:
        print("No event provided")
        session.append(Session(name = ses_name + '_' + str(sess), 
                               duration = sess_dur, config = init_config))
        continue
    
    if nb_event == 0:
        session.append(Session(name = ses_name + '_' + str(sess), 
                               duration = sess_dur, config = init_config))
        continue
    
    events = []  
    for event in xrange(nb_event):
        print("Event %s configuration: " %(event+1))
        valid_trigger = False
        while(valid_trigger == False):
            trigger = raw_input("Enter input type (D for delay, B for bounce): ")
            if trigger == 'D'or trigger == 'd':
                print("Delay choosen")
                valid_trigger = True
                trigger = "DELAY"
            elif trigger == 'B' or trigger == 'b':
                print("Bounce choosen")
                valid_trigger = True
                trigger = "BOUNCE" 
            else:
                print("Please type a valid trigger type. (B, b, D or d")
        
        valid_value = False
        while(valid_value == False):
            try:
                if trigger == "BOUNCE":
                    val = int(raw_input("Enter number of bounce: "))
                else:
                    val = float(raw_input("Enter a delay (float accepted): "))
                    
            except ValueError:
                print("Please type a valid value!")  
            else:
                valid_value = True
            
        try:
            height = float(raw_input("Enter target height: "))
        except ValueError:
            print("No target height provided")  
            height = None
        try:
            coeff = float(raw_input("Enter restitution coefficient: "))
        except ValueError:
            print("No Restitution coefficient provided")  
            coeff = None
        try:
            gravity = float(raw_input("Enter gravity: "))
        except ValueError:
            print("No gravity provided")
            gravity = 9.807
        
        event_config = Config(t_height = height, coeff = coeff, gravity = gravity)
        events.append(Event(('event_' + str(event)), trigger, val, event_config))
    
    
    session.append(Session(name = ses_name + '_' + str(sess), 
                               duration = sess_dur, config = init_config, event = events))

#for sess in xrange(nb_session):
#    print(session[sess])            
    
with open(file_name, 'w') as txt_file:
    for sess in xrange(nb_session):
        txt_file.write("{}".format(session[sess]))
        
        
        
