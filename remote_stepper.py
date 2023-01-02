# -*- coding: utf-8 -*-
"""
Created on Fri Sep  2 12:46:30 2022

@author: laubmabt

SCRIPT in order to operate the Phantom box in a remote ssh session
(either wifi or ethernet)

"""
from new_stepper import box_operation

box_operation(remote=True)
another_round = input("Another round? if yes: type anything +enter | if no: just enter")

while another_round:
    box_operation(remote=True)
    another_round = input("Another round? if yes: type anything +enter | if no: just enter")

1/0