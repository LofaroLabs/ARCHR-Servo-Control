#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Based on: https://github.com/thedancomplex/pydynamixel
# */

import sys
sys.path.append('/home/archr/hubo_simulation/files/dynamixel')
import os
import dynamixel
import serial_stream
import time
import random
import sys
import subprocess
import optparse
import yaml
import dynamixel_network
import numpy as np
import socket

#############################################################################################
##	Radian to Dynamixel conversion functions
def rad2dyn(rad):
    return np.int(np.floor( (rad + np.pi)/(2.0 * np.pi) * 1023 ))

def dyn2rad(en):
    return ((en*2.0*np.pi)/1024) - np.pi

##############################################################################################


def main(settings):
    #start server
    host = '192.168.0.102'		#gets comp ip address
    port = 20000						#random port
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)	#tells udp
    s.bind((str(host), port))					#binds it
    print str(s.bind)
    print "Server Started."
    #highestServoId, addr = s.recvfrom(1024)			#receves number of servos per robot
    
    # Establish a serial connection to the dynamixel network.
    # This usually requires a USB2Dynamixel
    portName = settings['port']				#pick the usb to dyn port
    baudRate = settings['baudRate']			
    highestServoId = settings['highestServoId']
    seriall = serial_stream.SerialStream(port=portName, baudrate=baudRate, timeout=1)
    net = dynamixel_network.DynamixelNetwork(seriall)

    # Ping the range of servos that are attached
    print "Scanning for Dynamixels..."
    net.scan(1, highestServoId)				#scans for dyns
    ST=[]						#start torque for the servos
    myActuators = []					#list with all the dynamixels
    for dyn in net.get_dynamixels():			#loop puts the dyns in the list
        print dyn.id
        myActuators.append(net[dyn.id])
    
    if not myActuators:					#if no dyns were found print none found
      print 'No Dynamixels Found!'
      sys.exit(0)
    else:
      print "...Done"
    
    for actuator in myActuators:			#loop for initializing dyns
        actuator.moving_speed = 250			#makes the robot move at this speed
        actuator.synchronized = True
        actuator.torque_enable = True
	ST.append(abs(actuator._get_current_load()))	#gets the initial load of a servo
	
    print 'waiting'

    while True:
        counter=0					#counts through the list below
        data, addr = s.recvfrom(1024)			#gets positions from the client
        data = data.split()				# splits the string at spaces
        for actuator in myActuators:			#assigns the positions to the servos
	    actuator._set_goal_position(int(data[counter]))
            counter+=1
	pos = ''					#preps a str to start sending info back
	pos = str(pos)					#i dont think this needs to be here
	counter=0					#counter to count through the initial loads
	for actuator in myActuators:			#str to send back data
   		pos = pos + str(actuator._get_current_position()) + ' ' #gets current position
		pos = pos + str(abs(actuator._get_current_load())) + ' '	# gets current load -initial load
		counter+=1
	pos.join(' ')					#adds another space at the end
	s.sendto(pos, addr)				#sends the str to the client
        net.synchronize()				#sets all the positions of the servos connected to the server
    c.close()

def validateInput(userInput, rangeMin, rangeMax):
    '''
    Returns valid user input or None
    '''
    try:
        inTest = int(userInput)
        if inTest < rangeMin or inTest > rangeMax:
            print "ERROR: Value out of range [" + str(rangeMin) + '-' + str(rangeMax) + "]"
            return None
    except ValueError:
        print("ERROR: Please enter an integer")
        return None
    
    return inTest

if __name__ == '__main__':
    
    parser = optparse.OptionParser()
    parser.add_option("-c", "--clean",
                      action="store_true", dest="clean", default=False,
                      help="Ignore the settings.yaml file if it exists and \
                      prompt for new settings.")
    
    (options, args) = parser.parse_args()
    
    # Look for a settings.yaml file
    settingsFile = 'settings.yaml'
    if not options.clean and os.path.exists(settingsFile):
        with open(settingsFile, 'r') as fh:
            settings = yaml.load(fh)
    # If we were asked to bypass, or don't have settings
    else:
        settings = {}
        if os.name == "posix":
            portPrompt = "Which port corresponds to your USB2Dynamixel? \n"
            # Get a list of ports that mention USB
            try:
                possiblePorts = subprocess.check_output('ls /dev/ | grep -i usb',
                                                        shell=True).split()
                possiblePorts = ['/dev/' + port for port in possiblePorts]
            except subprocess.CalledProcessError:
                sys.exit("USB2Dynamixel not found. Please connect one.")
                
            counter = 1
            portCount = len(possiblePorts)
            for port in possiblePorts:
                portPrompt += "\t" + str(counter) + " - " + port + "\n"
                counter += 1
            portPrompt += "Enter Choice: "
            portChoice = None
            while not portChoice:                
                portTest = raw_input(portPrompt)
                portTest = validateInput(portTest, 1, portCount)
                if portTest:
                    portChoice = possiblePorts[portTest - 1]

        else:
            portPrompt = "Please enter the port name to which the USB2Dynamixel is connected: "
            portChoice = raw_input(portPrompt)
    
        settings['port'] = portChoice
        
        # Baud rate
        baudRate = None
        while not baudRate:
            brTest = raw_input("Enter baud rate [Default: 1000000 bps]:")
            if not brTest:
                baudRate = 1000000
            else:
                baudRate = validateInput(brTest, 9600, 1000000)
                    
        settings['baudRate'] = baudRate
        
        # Servo ID
        highestServoId = None
        while not highestServoId:
            hsiTest = raw_input("Please enter the highest ID of the connected servos: ")
            highestServoId = validateInput(hsiTest, 1, 255)
        
        settings['highestServoId'] = highestServoId
        
        # Save the output settings to a yaml file
        #with open(settingsFile, 'w') as fh:
        #    yaml.dump(settings, fh)
        #    print("Your settings have been saved to 'settings.yaml'. \nTo " +
        #           "change them in the future either edit that file or run " +
        #           "this example with -c.")
    
    main(settings)

