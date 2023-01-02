# -*- coding: utf-8 -*-
"""
Created on Fri Sep  2 15:48:53 2022

@author: laubmabt


script file that contains a function which is to be called if you want the box to work

this can either be in remote mode remote = True or 
in local mode, which means keyboard interaction as input
"""

def box_operation(remote=False):
    ## imports
    import RPi.GPIO as GPIO
    import time
    import os
    import random
    # not the best way
    global Button1, Button2, motorIN1, motorIN2, motorIN3, motorIN4
    global total_steps_per_revolution, global_sleeptime, fastest_sleeptime
    global overload_sleeptime, stepmatrix,step_pin, dir_pin, direction, time_per_revolution
    global breathing_pauses, actual_keypad, reset_flag, reset_starttime
    global pause_flag, pause_starttime, breathing_cycle, random_pos, PowerOn    
    global enable_pin, max_tumor_position, motor_position, tumor_position
    global actual_step, keypad_characters, lines, columns

    ## show the pinout 
    os.system("pinout")
    print("...")
    print("SCROLL UPWARDS IN ORDER TO SEE THE PIN NUMBERS")
    print("...")
    
    GPIO.setmode(GPIO.BCM) # call the Pins by number

    # define GPIO pins for the small bumper buttons
    Button1 = 24 # the "inner"
    Button2 = 25 # the "outer"
    GPIO.setup(Button1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
    GPIO.setup(Button2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
    
    
    ## SMALL BREATHING MOTOR
    # define GPIO pins wired to the motor driver
    motorIN1 = 14
    motorIN2 = 15
    motorIN3 = 18
    motorIN4 = 23
    
    GPIO.setup(motorIN1, GPIO.OUT)
    GPIO.setup(motorIN2, GPIO.OUT)
    GPIO.setup(motorIN3, GPIO.OUT)
    GPIO.setup(motorIN4, GPIO.OUT)


    
    # motor basics
    total_steps_per_revolution = 2048
    fastest_time_per_revolution = 4 # in seconds: given by motor specs: 4 -> means 2 times breathing in 4 seconds, without any pause
    time_per_revolution = 3.3*2 # in seconds: actual time for 2 breathings in average ##3.8 ist evtl sinnvoller
    
  
    global_sleeptime = time_per_revolution/total_steps_per_revolution # in seconds
    fastest_sleeptime = fastest_time_per_revolution/total_steps_per_revolution # in seconds
    
    overload_sleeptime = 0.00001
    
    
    # clockwise rotation
    stepmatrix = [[1,1,0,0],
                  [0,1,1,0],
                  [0,0,1,1],
                  [1,0,0,1]]
    
    
    ## BIG TUMOR MOTOR
    # define Pins
    step_pin = 21
    GPIO.setup(step_pin, GPIO.OUT)
    # by default set the step pin to low
    GPIO.output(step_pin, 0)
    
    dir_pin = 26
    GPIO.setup(dir_pin, GPIO.OUT)
    # by default set direction to 1 == clockwise turnings
    direction = True
    GPIO.output(dir_pin, direction)
    
    enable_pin = 20
    GPIO.setup(enable_pin, GPIO.OUT)
    # by default disable the driver for power savings
    GPIO.output(enable_pin, 1)
    
    
    ## SPINDEL PROPERTIES OF TUMOR MOTIONS
    # maximum range ~4 rotations
    # one rotation is 360/1.8 steps = 200
    max_tumor_position = 512
    
    
    
    ## STEP function
    motor_position = 0 # position [steps] as its symmetric % total_steps_per_rev/2
    tumor_position = 0 # position [steps] of the big tumor motion motor
    
    actual_step = 0
    
    ## Keypad
    # Keypad characters
    keypad_characters = [["1","2","3","A"],
                         ["4","5","6","B"],
                         ["7","8","9","C"],
                         ["*","0","#","D"]]
    # GPIO Pins
    line1 = 2 # grey
    line4 = 3 # 
    line7 = 4 # 
    line_star = 17 # 

    # set up the PIN properties
    lines = [line1,line4,line7,line_star]
    for line in lines:
        if line ==None:
            continue
        GPIO.setup(line, GPIO.OUT)

    # GPIO Pins
    column1 = 27 # 
    column2 = 22  # 
    column3 = 10 # 
    columnA = 9 # 

    # set up the PIN properties
    columns=[column1,column2,column3,columnA]
    for column in columns:
        GPIO.setup(column, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        

    ## Breathing pauses
    breathing_pauses = [1,6,7,8,9,10,9,8,7,6,5] # pyramid shape , in seconds


    ## INITIALIZING VARIABLES IN THE RUNNING LOOP
    

    # initialize the flag and starttime for resetbutton (3s "A")
    reset_flag = False
    reset_starttime = time.time_ns()

    # initialize the flag + starttime for breathing pause mode (3s "1")
    pause_flag = False
    pause_starttime = time.time_ns()

    # initialize breathing cycle number and random pos for alternativ 1 programm
    breathing_cycle = 1
    random_pos = 0 # unrealistic but ok, then the first cycle will probably not make breathing pause

    PowerOn = True
    
    def write_pins(array):
        global motorIN1, motorIN2, motorIN3, motorIN4
        [x1,x2,x3,x4] = array
        # writes to the 4 pins
        GPIO.output(motorIN1,x1)
        GPIO.output(motorIN2,x2)
        GPIO.output(motorIN3,x3)
        GPIO.output(motorIN4,x4)
        return

    def step(number,reset=False, feedback=False, big_only=False, small_only=False, custom_sleeptime=False):
        # does in reality 4 steps for the small one
        global motor_position, tumor_position, direction, global_sleeptime
        
        
        # check for custom sleeptime
        sleeptime = global_sleeptime
        if custom_sleeptime:
            sleeptime=custom_sleeptime
        
        
        GPIO.output(enable_pin,0) # enable the big motor driver
        
        # check to not crash and decide the running direction:
        # forward: clockwise: dir_pin=1 (default from initial setup)
        # backward: ccw: dir_pin=0
         
        if GPIO.input(Button2) == GPIO.HIGH or tumor_position>max_tumor_position:
            if GPIO.input(Button2) == GPIO.HIGH:
                print("outer limit Button was pushed!")
            
            if tumor_position>max_tumor_position:
                print("logical limit of variable maxtoumorposition reached")
            direction = False
            
        
        
        if GPIO.input(Button1) == GPIO.HIGH or tumor_position<0:
            print("inner limit Button was pushed!")
            direction = True
            tumor_position=0

        
        # change the sign of the number in accordance to the direction
        if direction:
            number = abs(number)
        else:
            number = -abs(number)
        
        # seal the direction of the big motor
        GPIO.output(dir_pin,direction) 
        
        
            
        if reset:
            reset_steps = motor_position//4
            for i in range(int(reset_steps)):
                for line in [stepmatrix[2],
                             stepmatrix[1],
                             stepmatrix[0],
                             stepmatrix[3]]:
                    write_pins(line) # small motor
                    time.sleep(sleeptime) # in seconds
            number=0
            motor_position=0
            write_pins([0,0,0,0]) 
            print("reset of smol motor done")
            
            while not GPIO.input(Button1) == GPIO.HIGH:
                GPIO.output(dir_pin,False)
                GPIO.output(step_pin,1) # big motor
                time.sleep(sleeptime/2) # in seconds
                GPIO.output(step_pin,0)
                time.sleep(sleeptime/2) # in seconds
            tumor_position=0
            direction=True
            GPIO.output(dir_pin, direction)
            GPIO.output(enable_pin,1) #energy saving
            print("reset of big motor done")
            
        if feedback:
            print("feedback")
            #do steps and back
            for i in range(10):
                for line in stepmatrix:
                    write_pins(line) # small motor
                    GPIO.output(step_pin,1) # big motor
                    time.sleep(sleeptime) # in seconds
                    GPIO.output(step_pin,0)
                    time.sleep(sleeptime) # in seconds
                    
            GPIO.output(dir_pin,not direction)
            for i in range(10):
                for line in [stepmatrix[2],
                             stepmatrix[1],
                             stepmatrix[0],
                             stepmatrix[3]]:
                    write_pins(line)
                    GPIO.output(step_pin,1) # big motor
                    time.sleep(sleeptime) # in seconds
                    GPIO.output(step_pin,0)
                    time.sleep(sleeptime) # in seconds
            GPIO.output(dir_pin,direction) 
            number=0
            
            # energy saving
            GPIO.output(enable_pin,1) 
            write_pins([0,0,0,0]) 
            
        
        if big_only:
            for i in range(abs(number)):
                # only the big motor
                GPIO.output(step_pin,1) # big motor
                time.sleep(sleeptime/2) # in seconds
                GPIO.output(step_pin,0)
                time.sleep(sleeptime/2) # in seconds
                
                if direction:
                    tumor_position += abs(number)
                else:
                    tumor_position -= abs(number)
            
            number=0 # no confusion afterwards
        
        if small_only:
            GPIO.output(enable_pin, 1) #disable the big one
                
            if number > 0:
                for i in range(number):
                    for line in stepmatrix:
                        write_pins(line) # small motor
                        time.sleep(sleeptime) # in seconds
                        
                    motor_position += 4 # done 4 steps
                    
            if number < 0:
                for i in range(int(abs(number))):
                    for line in [stepmatrix[2],
                                 stepmatrix[1],
                                 stepmatrix[0],
                                 stepmatrix[3]]:
                        write_pins(line)
                        time.sleep(sleeptime) # in seconds
                        
                    motor_position -= 4
            number=0 # no confusion afterwards, all steps are made in this case
            
            
        # normal procedure     
        if number > 0 or direction:
            for i in range(number):
                for line in stepmatrix:
                    write_pins(line) # small motor
                    GPIO.output(step_pin,1) # big motor
                    time.sleep(sleeptime/2) # in seconds
                    GPIO.output(step_pin,0)
                    time.sleep(sleeptime/2) # in seconds
                    
                motor_position += 4 # done 4 steps
                
        if number < 0 or not direction:
            for i in range(int(abs(number))):
                for line in [stepmatrix[2],
                             stepmatrix[1],
                             stepmatrix[0],
                             stepmatrix[3]]:
                    write_pins(line)
                    GPIO.output(step_pin,1) # big motor
                    time.sleep(sleeptime/2) # in seconds
                    GPIO.output(step_pin,0)
                    time.sleep(sleeptime/2) # in seconds
                    
                motor_position -= 4
        
        
       # update the positions
        motor_position = motor_position % (total_steps_per_revolution/2) #this is just the position of the smol motor
        if direction:
            tumor_position += 4*abs(number)
        else:
            tumor_position -= 4*abs(number)
        
        # comment for more holding torque:
        # uncomment for energy saving / less currents in the smol motor:
        
        #write_pins([0,0,0,0]) 
        #GPIO.output(enable_pin,1) #energy saving
        
        return


    ## OVERLAP HEARTBEAT to the usual movement 
    def heartbeat_residue(heart_rate=60, # in BPM
                          breathing_time=(time_per_revolution/2), # in s
                          fastest_time_per_revolution=fastest_time_per_revolution): 
        """
        overlap for :number: of breathing cycles at average breathing time :breathing_time: with
        heartbeat movement at :heart_rate:, in a slower-faster way of stepduration
        todo: problem: this function can only be stopped after a full revolution/residue 
        -> you have to wait for the residue be done and press all the time another button like 3
        """
        global total_steps_per_revolution, stepmatrix, actual_step, motor_position
        
        # first: reset to parking position
        step(1,reset=True, custom_sleeptime=fastest_sleeptime)
        GPIO.output(enable_pin,0) # enable after reset
        
        # calculate desired waiting times and quantities
        average_waiting_time = breathing_time*2 / total_steps_per_revolution
        fastest_waiting_time = fastest_time_per_revolution / total_steps_per_revolution
        time_per_heartbeat = heart_rate/60 # in seconds
        average_number_of_steps_per_heart_residue = time_per_heartbeat/average_waiting_time
        
        # now for the first issue, i want to do one quarter (1/4 sec) of a heartbeatresidues
        # faster and the rest slower
        number_of_fast_steps = 1/4 * time_per_heartbeat /fastest_waiting_time
        number_of_slow_steps = average_number_of_steps_per_heart_residue - number_of_fast_steps
        sleeptime_for_slowsteps = 3/4 * time_per_heartbeat /number_of_slow_steps
        
        
        start_time = time.time()
        
        # do one residue
        for i in range(total_steps_per_revolution//2 //4):
            
            
                
            for line in stepmatrix:
                # Decide the actual speed == waiting time == sleeptime 
                if (actual_step%average_number_of_steps_per_heart_residue)<number_of_fast_steps:
                    my_sleeptime = fastest_waiting_time # fast
                    
                
                else:
                    my_sleeptime = sleeptime_for_slowsteps # slow
                                        
                
                write_pins(line) # small motor
                motor_position += 1 #did one step with the small motor
                motor_position = motor_position % (total_steps_per_revolution/2) #this is just the position of the smol motor
                step(1,big_only=True,custom_sleeptime=my_sleeptime) #using step function because of position and direction tracking
                
                #did one step at a certain speed
                actual_step += 1 # for the hearbeat speed calculation
                
                
                
        print("residue done, done %s steps in %s seconds"%(actual_step, time.time()-start_time))
        return    

    def keypad():
        """
        reads out the keypad on the box
        return: 
            The string of the keypad character (as defined above) 
            or "NOTHING" as string
        """    
        
        # readout physical keybad on the box
        for line_index, line in enumerate(lines):
            if line==None:
                continue
            else:
                # set the current high for each line
                GPIO.output(line,GPIO.HIGH) 
                # check the columns for signal
                for column_index, column in enumerate(columns):
                    #print(line_index,column_index,GPIO.input(column))
                    if GPIO.input(column) == 1:
                        GPIO.output(line,GPIO.LOW) # this is very important
                        return keypad_characters[line_index][column_index]
                GPIO.output(line,GPIO.LOW) 
        
        # if no input, return "NOTHING"
        return "NOTHING"

    # initialize the keypad variable 
    actual_keypad = keypad() # should be Nothing

    if remote:
        actual_keypad=input("type one of the following strings and press enter for the programm (1 ,2, 3, 4, 5, 6, 7, 8, A, B, 1_3s):")


    ## START OF THE MAIN LOOP
    print("runnning..  press any button on the keypad for operation or press Ctrl +C for kill")

    step(1,reset=True)
    #step(1,feedback=True)

     
    
        
    
    try:    
        ## LOOP WHICH RUNS ALL THE TIME
        while True:
            if actual_keypad == "B":
                ## turn off button was hit    (B)
                # return to parking position
                if motor_position or tumor_position:
                    print("going backwards %s, %s steps" % (motor_position,tumor_position))    
                    step(1,reset=True,custom_sleeptime=fastest_sleeptime)
                
                # turn off currents
                write_pins([0,0,0,0])
                GPIO.output(enable_pin,1) #energy saving
                
                for line in lines:
                    if line==None:
                        continue
                    GPIO.output(line,GPIO.LOW)    
                
                # mandatory cleanup of GPIO assignments - prevents warnings in next restart
                GPIO.cleanup()
                print("killed..  by button B")    
                return            
            
                
            # STOP SIGNAL AND IF PRESSED >3s: RESET OF POSITION
            elif actual_keypad == "A": 
                # turn off currents
                write_pins([0,0,0,0])
                

                GPIO.output(enable_pin,1) #energy saving
                
                ## RESET OF POSITION
                
                # check if it is the same pressing of the button 
                # (nothing else was pressed in the meantime)
                if reset_flag:
                    # check if the timer was already running > 3s
                    if (time.time_ns() - reset_starttime) > 3*1e9:
                        # now reset the position to 0
                        motor_position = 0
                        while not GPIO.input(Button1) == GPIO.HIGH:
                            step(1,big_only=True)
                        #tumor_position = 0
                        
                        # and restart timer
                        reset_starttime = time.time_ns()
                        
                        # give feedback that reset was successful
                        step(1, feedback=True)
                        print("reset of parking postition successful")
                    
                reset_flag = True
               
                
            # fast and if pressed >3s: breathing pauses included
            elif actual_keypad == "1":
                # normal procedure if pressed "1"
                step(1)
                
                ## check if 3s pressed
                if pause_flag:
                    # check timer
                    if (time.time_ns() - pause_starttime) > 3*1e9:
                        # activate new mode
                        actual_keypad = "1_3s" # alternative programm
                        
                        # reset timer
                        #pause_starttime = time.time_ns()
                        
                        # give feedback that the reset was succesful
                        time.sleep(1)
                        step(1,feedback=True)
                        print("alternative mode for '1' successful activated")
                        
                        
                pause_flag = True     
                
            
            # alternative 1: breathing pauses
            elif actual_keypad == "1_3s":
                # alternative procedure if pressed 1 for >3s
                step(1) 
                
                # start a counter and every xx turns, a random breathing 
                # pause will be implemented into the "normal" breathing
                number_of_cycles = 5 #every x breathing cycle will happening sth
                
                # count the breathing cycles
                if abs(motor_position) < 4:
                    breathing_cycle +=1
                    
                    # step position can be betwenn 0-1024 in 4units steps
                    # eg [0,4,8,12,16,...,1020,1024[
                    
                    #choose random pos for this specific cycle
                    random_pos = random.randint(0,total_steps_per_revolution//2 //4)*4
                    
                    
                    
                    print("breathing cycle: actual/frequency of pauses %s / %s\n actual random breathing position %s" %(breathing_cycle,number_of_cycles,random_pos))
                    
                
                # if breathing cycle matches the predefined number: action
                if (breathing_cycle % number_of_cycles)==0:
                    # now breathing pause at specific phase
                    if motor_position==random_pos:
                        # pause time according to the pyramid shape pausing times
                        pausing_cycle = breathing_cycle//number_of_cycles
                        pause_time = breathing_pauses[pausing_cycle%len(breathing_pauses)]
                        print("make a break of %s s"%pause_time)
                        # one breathing duration pause
                        time.sleep(pause_time)
                        
            
            # slower
            elif actual_keypad == "2":
                step(1,custom_sleeptime=2*global_sleeptime)
                
                
            # reset to parking position
            elif actual_keypad == "3":
                if motor_position or tumor_position:
                    print("going backwards %s, %s steps" % (motor_position,tumor_position))
                    step(1,reset=True,custom_sleeptime=fastest_sleeptime)            
                
            # heartbeat residue
            elif actual_keypad == "4":
                heartbeat_residue()
                #write_pins([1,1,0,0])
                
            
            # slow movement of the smol tuna    
            elif actual_keypad == "5":
                step(1,small_only=True,custom_sleeptime=2*global_sleeptime)
                
                
            # backwards of the big tuna
            elif actual_keypad == "6":
                direction = not direction
                step(1,big_only=True,custom_sleeptime=2*global_sleeptime)
                direction = not direction
                
                
            # asymmetric movement
            elif actual_keypad == "7":
                if direction:
                    step(1)
                else:
                    #double the sleeptime
                    step(1,custom_sleeptime=2*global_sleeptime)
            
                
            # go to maximum position
            elif actual_keypad == "8":
                # return to parking position first
                if motor_position or tumor_position:
                    print("going backwards %s, %s steps" % (motor_position,tumor_position))    
                    step(1,reset=True,custom_sleeptime=fastest_sleeptime)

                # go to the maximum position
                # small one first
                step(total_steps_per_revolution//16,small_only=True) #small one
                # big one afterwards
                while direction:
                    step(1,custom_sleeptime=fastest_sleeptime,big_only=True)
                
                # thats it. rest here now
                actual_keypad="A"
            
            
            # 4s per breathing cycle = 8s per motor cycle = 0.25Hz breathing frequency
            elif actual_keypad == "9":
                this_sleeptime = 2*4 / total_steps_per_revolution# for 0.25Hz = 4 s per breathing cycle = 8s per motor cycle
                step(1,custom_sleeptime=this_sleeptime)
                
            
            # check the keypad for action
            keypad_input = keypad()
            if keypad_input != "NOTHING":
                # Look for changes to the prvs input
                if actual_keypad != keypad_input:
                    print("keypad input: ", keypad_input)    
                actual_keypad = keypad_input
                
            
            # anyway always
            else:
                # sleeping time prevents processor overload
                time.sleep(overload_sleeptime)
                 
                
            # check if keypad_input is still "A" - for the reset function
            if keypad_input!= "A":
                reset_flag = False
                # start timer again
                reset_starttime = time.time_ns()
            
            
            # check if keypad_input is still "1" - for the alternative 1 function
            if keypad_input  != "1":
                pause_flag = False
                # start timer
                pause_starttime = time.time_ns()
    except KeyboardInterrupt:
        # return to parking position
        if motor_position or tumor_position:
            print("going backwards %s, %s steps" % (motor_position,tumor_position))        
            step(1,reset=True,custom_sleeptime=fastest_sleeptime)
            
        # turn off currents
        write_pins([0,0,0,0])
        for line in lines:
            if line==None:
                continue
            GPIO.output(line,GPIO.LOW)    

        # mandatory cleanup of GPIO assignments - prevents warnings in next setup
        GPIO.cleanup()
        print("illed..  ")
        return
    
    
    # eof
        
        

