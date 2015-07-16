#!/usr/bin/python
#SYSTEM IMPORTS
import math
import time
from pymavlink import mavutil

#COMMOM IMPORTS
from Common.VN_config import VN_config
from Common.VN_logger import VN_logger

#DRONEAPI IMPORTS
from droneapi.lib import VehicleMode, Location, Attitude



"""
This class encapsulates the vehicle and some commonly used controls via the DroneAPI
"""

'''
TODO:
    Add support to retrieve home, landing waypoints, and rally points for simulator
'''



class VehicleControl(object):

    def __init__(self):
        # add variable initialisation here
        self.api = None
        self.vehicle = None

        self.last_mode_call = 0
        self.last_mode_state = 'STABILIZE'
        self.mode_update_rate = VN_config.get_float('vehicle control', 'mode_update_rate', 0.75)

        self.last_home_call = 0
        self.last_home = None
        self.home_update_rate = VN_config.get_float('vehicle control', 'home_update_rate', 10)

        self.last_set_velocity = 0
        self.vel_update_rate = VN_config.get_float('vehicle control', 'vel_update_rate', 0.1)

        self.last_report_landing_target = 0
        self.landing_update_rate = VN_config.get_float('vehicle control', 'landing_update_rate', 0.02)



    # connect - connects to droneAPI.
    #    because of scope issues the local_connect() must be called from the top level class
    def connect(self, api):
        # First get an instance of the API endpoint (the connect via web case will be similar)
        self.api = api

        # if we succesfully connect
        if not self.api is None:
            # get our vehicle (we assume the user is trying to control the virst vehicle attached to the GCS)
            self.vehicle = self.api.get_vehicles()[0]
            return

    #is_connected - are we connected to a DroneApi
    def is_connected(self):
        if (self.api is None) or (self.vehicle is None):
            return False
        return (not self.api.exit)

    #get_vehicle - returns the connected vehicle
    def get_vehicle(self):
        return self.vehicle

    # controlling_vehicle - return true if we have control of the vehicle
    def controlling_vehicle(self):
            if self.api is None:
                return False
            
            # we are active in guided mode
            if self.get_mode() == "GUIDED":
                return True
            
            else:
                return False
    # is_armed - returns arm status of vehicle
    def is_armed(self):
        return self.vehicle.armed

    # set_yaw - send condition_yaw mavlink command to vehicle so it points at specified heading (in degrees)
    def set_yaw(self, heading):
        # create the CONDITION_YAW command
        msg = self.vehicle.message_factory.mission_item_encode(0, 0,  # target system, target component
                                                     0,     # sequence
                                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
                                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW,         # command
                                                     2, # current - set to 2 to make it a guided command
                                                     0, # auto continue
                                                     heading, 0, 0, 0, 0, 0, 0) # param 1 ~ 7
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    # set_velocity - send nav_velocity command to vehicle to request it fly in specified direction
    def set_velocity(self, velocity_x, velocity_y, velocity_z):
        #only let commands through at 10hz
        if(time.time() - self.last_set_velocity) > self.vel_update_rate:
            self.last_set_velocity = time.time()
            # create the SET_POSITION_TARGET_LOCAL_NED command
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                                                         0,       # time_boot_ms (not used)
                                                         0, 0,    # target system, target component
                                                         mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                                                         0x01C7,  # type_mask (ignore pos | ignore acc)
                                                         0, 0, 0, # x, y, z positions (not used)
                                                         velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                                                         0, 0, 0, # x, y, z acceleration (not used)
                                                         0, 0)    # yaw, yaw_rate (not used)
            
            # send command to vehicle
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            VN_logger.text(VN_logger.AIRCRAFT, 'Sent Vx: {0}, Vy: {1}, Vz: {2}'.format(velocity_x,velocity_y,velocity_z))



    # report_landing_target - send LANDING_TARGET command to vehicle to shift landing location
    def report_landing_target(self, angle_x, angle_y, distance,size_x,size_y):
        #only let commands through at 33hz
        if(time.time() - self.last_report_landing_target) > self.landing_update_rate:
            self.last_report_landing_target_ = time.time()
            # create the LANDING TARGET message
            msg = self.vehicle.message_factory.landing_target_encode(
							 0,         # time(unused)
                                                         0,         # landing target number (not used)
                                                         8,         # frame
                                                         angle_x,   # Angular offset x axis
                                                         angle_y,   # Angular offset y axis
                                                         distance,  # Distance to target
							 size_x,    # unused
							 size_y)    # unused
            # send command to vehicle
            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()
            VN_logger.text(VN_logger.AIRCRAFT, 'Sent AngX: {0}, AngY: {1}, Dist: {2}'.format(angle_x,angle_y,distance))



    #get_location - returns the lat, lon, alt of vehicle
    def get_location(self):
        return self.vehicle.location

    #get_attitude - returns pitch, roll, and yaw of vehicle
    def get_attitude(self):
        return self.vehicle.attitude

    #get_home - get the home location for this mission
    def get_home(self, wait_for_arm = False):

        #wait unitl we are armed to grab the INITIAL home position. This will lock up the program.
        if(wait_for_arm and self.last_home is None):
            VN_logger.text(VN_logger.GENERAL, 'Waiting for intial home lock: Requires armed status....')
            while(self.vehicle.armed == False):
                time.sleep(0.3)
            VN_logger.text(VN_logger.GENERAL, 'Got valid intial home location')


        if(time.time() - self.last_home_call > self.home_update_rate):
            self.last_home_call = time.time()

            # download the vehicle waypoints
            mission_cmds = self.vehicle.commands
            mission_cmds.download()
            mission_cmds.wait_valid()
            # get the home lat and lon
            home_lat = mission_cmds[0].x
            home_lon = mission_cmds[0].y

            self.last_home = Location(home_lat,home_lon,0)

        return self.last_home

    #get_mode - get current mode of vehicle
    def get_mode(self):
        #limit how often we request the current mode
        if (time.time() - self.last_mode_call) > self.mode_update_rate:
            self.last_mode_call = time.time()
            self.last_mode = self.vehicle.mode.name

        return self.last_mode

    #set_mode - set the mode of the vehicle as long as we are in control
    def set_mode(self, mode):
        if self.controlling_vehicle():
            self.vehicle.mode = VehicleMode(mode)
            self.vehicle.flush()
            return True

        return False

    # run - should be called repeatedly from parent
    def run(self):
        # return immediately if not connected
        if self.api is None:
            return

        # we are connected so iterate
        if self.controlling_vehicle():
            # request vehicle to turn due east
            self.set_yaw(90)
        return

    # test - should be called to test this class from the simulator
    def test(self):
        # return immediately if not connected
        if self.api is None:
            return

        while not self.api.exit:
            # we are connected so iterate

            # control vehicle
            if self.controlling_vehicle():
                # request vehicle to turn due east
                self.set_yaw(90)
                #self.set_velocity(200,0,0)
                print self.get_attitude()
                print self.get_location()

            # sleep so we don't consume too much CPU
            time.sleep(1.0)




# create global object
veh_control = VehicleControl()

# if this is the parent class connect and run test
if __name__ == "__builtin__":
    veh_control.connect(local_connect())
    veh_control.test()

