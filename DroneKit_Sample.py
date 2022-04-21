#!/usr/bin/env python3

from dronekit import connect, VehicleMode
import dronekit
import socket
import argparse
import time

# try to connect to the vehicle
def connectMySub():

    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect

    try:
        vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout=30, baud=115200)
        print('good')

        #return vehicle

    # check for bad TCP connection
    except socket.error:
        print('No server exists!')

    # check for API error
    except dronekit.APIException:
        print('Timeout!')

    # check for other error
    except:
        print('Some other error occured!')

    return vehicle

def mySubAttributes():

    print('Autopilot FIrmware version: %s' % vehicle.version)

def armSub():
    while not vehicle.is_armable:
        print("Waiting to become armable")
        time.sleep(1)

    #switch to stabilize mode
    vehicle.mode = VehicleMode("STABILIZE")
    while vehicle.mode != "STABILIZE":
        print("Waiting for vehicle to enter stabilize mode")
        time.sleep(1)

    print('Vehicle in stabilize mode')

    #arm the vehicle one in guided mode
    vehicle.armed = True
    while vehicle.armed != True:
        print("Waiting for vehicle to be armed")
        time.sleep(1)
    
    print("vehicle is armed")

vehicle = connectMySub()
mySubAttributes()
armSub()
