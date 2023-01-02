# -*- coding: utf-8 -*-
"""
Created on Fri Sep  2 12:46:30 2022

@author: laubmabt

SCRIPT in order to operate the Phantom box in a local way (keypad operation)

This script is called by the autostart script after booting the system
The autostart script is located at /ETC/RC.LOCAL and is always executed with
sudo rights.
Also the manipulation of /ETC/RC.LOCAL needs sudo rights. 
reading : nano /ETC/RC.LOCAL
writing : sudo nano /ETC/RC.LOCAL


"""
from new_stepper import box_operation

box_operation()

1/0
#