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
from ovrsdk import *
import socket
import thread
import math

#############################################################################################
##    Radian to Dynamixel conversion functions
def rad2dyn(rad):
    return np.int(np.floor( (rad + np.pi)/(2.0 * np.pi) * 1023 ))

def dyn2rad(en):
    return ((en*2.0*np.pi)/1024) - np.pi

##############################################################################################
#global
#open a link to the server
x=1
host = '192.168.0.111'    #this is the computers current IP
port = 21111          #this is just a random port number
server = ('192.168.0.102',20000)    #tells the computer where to listen and send
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    #tells to use network/udp
while (x == 1):            #this makes it so that if a port is taken it will try for a new port
    try:
        print port
        s.bind((host, port))    #trys this port if it is taken then moves on to a new port
        x=0
    except IOError:
        port+=1
    else:
        port+=1


def main(settings):
    portName = settings['port']        #searches for a usb2dyn
    baudRate = 1000000             #this is set to max can be changed to =>settings['baudRate']<= to be asked for a speed
    highestServoId = settings['highestServoId']#asked for highest servo id
    seriall = serial_stream.SerialStream(port=portName, baudrate=baudRate, timeout=1)
    net = dynamixel_network.DynamixelNetwork(seriall) # Ping the range of servos that are attached
    print "Scanning for Dynamixels..."
    net.scan(1, highestServoId)        #scans for all attached servos
    
    myActuators = []            #starts a list
    for dyn in net.get_dynamixels():    #makes a list of all dyn's
        print dyn.id
        myActuators.append(net[dyn.id])
    
    if not myActuators:            #if the list has nothing show error
      print 'No Dynamixels Found!'
      sys.exit(0)
    else:                #if it found some it prints done
      print "...Done"
    for actuator in myActuators:    #this is just an initializing loop to tell the servos to move as fast as we want
        actuator.moving_speed = 1023
        actuator.synchronized = True
        actuator.torque_enable = True
    actuator._set_torque_limit(0)

    ovr_Initialize()            #this part is form the oculus rift(OR) sdk this gets the pan and tilt
    hmd = ovrHmd_Create(0)
    hmdDesc = ovrHmdDesc()
    ovrHmd_GetDesc(hmd, byref(hmdDesc))
    ovrHmd_StartSensor(hmd,ovrSensorCap_Orientation | ovrSensorCap_YawCorrection,0)#end this part of OR code
    while True:    #the main part of our code
        actuator.read_all()
        ss = ovrHmd_GetSensorState(hmd, ovr_GetTimeInSeconds())#more OR code
        pose = ss.Predicted.Pose
        tilt = rad2dyn(pose.Orientation.x*np.pi);                #gets tilt
        pan = rad2dyn(pose.Orientation.y*np.pi);                #gets pan
        pos = ''                    #this is the start of a string because that is how info is sent over the network
        for actuator in myActuators:                           #loop for actuators
               pos = pos + str(actuator._get_current_position()) + ' '     #adds the current position to the string we made
        pos = pos + str(pan) + ' ' + str(tilt)                    #adds the pan and tilt to the end of the list
        pos.join(' ')                                #adds a space at the end
#print 'sent', pos
        s.sendto(pos, server)                            #sends the string made above
        data, addr = s.recvfrom(1024)                        #waits to get back the current psoition and load
#print data
        data=data.split()                            #this splits the string into a list
        counter=0                                #just a counter
        for actuator in myActuators:                        #loop for out actuators
   #time.sleep(1)
   #print actuator.id 
            if(abs(int(data[counter+1])) < 180):     
       #actuator._set_torque_limit(0)                #this makes it so we dont feel feedback
            else:                                #this makes it feel feedback
                actuator._set_torque_limit(int(data[counter+1]))    
                actuator._set_goal_position(int(data[counter]))        
            counter+=2                            #this counts through the list of servos coming in
            if counter==len(data):                        #breaks the for loop if needs to
                break
        net.synchronize()                            #sends the information to each servo to update


    #return None

def validateInput(userInput, rangeMin, rangeMax):                #setting up from here down
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
