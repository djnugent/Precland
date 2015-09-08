#!/usr/bin/python
#SYSTEM IMPORTS
import math
import time
import numpy as np
from pymavlink.mavutil import mavlink



#COMMOM IMPORTS
from Common.VN_config import VN_config
from Common.VN_logger import VN_logger
from Common.VN_position_vector import PositionVector
from Common.VN_util import *

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

        self.vehicle_pos = Vehicle_Pos() #used to interpolate drones location and attitude



    # connect - connects to droneAPI.
    #    because of scope issues the local_connect() must be called from the top level class
    def connect(self, api):
        # First get an instance of the API endpoint (the connect via web case will be similar)
        self.api = api

        # if we succesfully connect
        if not self.api is None:
            # get our vehicle (we assume the user is trying to control the virst vehicle attached to the GCS)
            self.vehicle = self.api.get_vehicles()[0]

        self.vehicle.set_mavlink_callback(self.message_handler)



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
                                                     mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
                                                     mavlink.MAV_CMD_CONDITION_YAW,         # command
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
                                                         mavlink.MAV_FRAME_LOCAL_NED, # frame
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
    def report_landing_target(self,usec,frame, angle_x, angle_y, distance,size_x,size_y):
        #only let commands through at 33hz
        if(time.time() - self.last_report_landing_target) > self.landing_update_rate:
            self.last_report_landing_target_ = time.time()
            # create the LANDING TARGET message
            msg = self.vehicle.message_factory.landing_target_encode(
							                             usec,      # time frame was captured
                                                         0,         # landing target number (not used)
                                                         frame,   # frame
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
    def get_location(self, timestamp = None):
        if timestamp is not None: #interpolate
            loc = self.vehicle_pos.get_location(timestamp)
            if loc is not None:
                return loc
        return self.vehicle.location #most recent reading if interpolation fails or isn't requested

#FIXME print if we fail interpolation

    #get_attitude - returns pitch, roll, and yaw of vehicle
    def get_attitude(self, timestamp = None):
        if timestamp is not None: #interpolate
            pos = self.vehicle_pos.get_attitude(timestamp)
            if pos is not None:
                return pos
        return self.vehicle.attitude #most recent reading if interpolation fails or isn't requested

#FIXME print if we fail interpolation


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
        self.vehicle.mode = VehicleMode(mode)
        self.vehicle.flush()

    def message_handler(self,m):
        typ = m.get_type()
        if typ == 'GLOBAL_POSITION_INT':
            timestamp = m.time_boot_ms * 1000 #usec
            (lat,lon,alt) = (m.lat /1.0e7, m.lat/1.0e7,m.relative_alt/1000.0)
            (vx,vy,vz) = (m.vx/100.0, m.vy/100.0, m.vz/100.0) #meter/sec
            loc = Location(lat,lon,alt)
            vel = Point3(vx,vy,vz)
            self.vehicle_pos.put_location(timestamp,loc,vel)

        if typ == 'ATTITUDE':
            timestamp = m.time_boot_ms * 1000 #usec
            att = Attitude(m.pitch, m.yaw, m.roll) #radians
            vel = Point3(m.rollspeed, m.pitchspeed, m.yawspeed) #rad/sec
            self.vehicle_pos.put_attitude(timestamp,att,vel)


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

# vehicle_pos - Used to interpolate between data points
class Vehicle_Pos(object):
    def __init__(self):
        self.size = 15
        self.location_buffer = np.empty((self.size),object)
        self.attitude_buffer = np.empty((self.size),object)
        self.lb_index = 0
        self.ab_index = 0

        #load buffers
        for i in range(0,self.size):
            self.put_location(0,Location(0,0,0), Point3(0,0,0))
            self.put_attitude(0,Attitude(0,0,0), Point3(0,0,0))



    def put_location(self,timestamp, location, vel):
        loc = PositionVector().get_from_location(location) # covert to meters
        self.location_buffer[self.lb_index] = (timestamp,loc,vel)
        if(self.lb_index == self.size - 1):
            self.lb_index = 0
        else:
            self.lb_index+=1


    def put_attitude(self, timestamp, attitude, ang_vel):
        pos = Point3(attitude.roll,attitude.pitch, attitude.yaw)
        self.attitude_buffer[self.ab_index] = (timestamp,pos,ang_vel)
        if(self.ab_index == self.size - 1):
            self.ab_index = 0
        else:
            self.ab_index+=1

    def _interpolate(self,in_buffer,timestamp):
        timestamp = timestamp / 1.0e6 #microseconds to seconds
        #print timestamp
        cp_buffer = in_buffer[:] # copy buffer incase it gets modified while we are searching through it
        for i in range(0,len(cp_buffer)):
            t0, p0, v0 = cp_buffer[i]
            t1, p1, v1 = cp_buffer[(i+1)%self.size]

            t0, t1 = t0 /1.0e6, t1/1.0e6 #microseconds to seconds

            dt = t1 - t0
            if timestamp >= t0 and timestamp <= t1: #located surrounding data points
                #interpolate between a bezier curve with the middle point being the intersection of velocity projections
                #p(t) = (t1 - t)/dt * (p0 + v0(t-t0)) + (t-t0)/dt *(p1-v1*(t-t1))
                x = ((t1-timestamp)/dt) * (p0.x + v0.x*(timestamp-t0)) + (timestamp-t0)/dt * (p1.x - v1.x*(t1-timestamp))
                y = ((t1-timestamp)/dt) * (p0.y + v0.y*(timestamp-t0)) + (timestamp-t0)/dt * (p1.y - v1.y*(t1-timestamp))
                z = ((t1-timestamp)/dt) * (p0.z + v0.z*(timestamp-t0)) + (timestamp-t0)/dt * (p1.z - v1.z*(t1-timestamp))

                return Point3(x,y,z)

    def get_location(self,timestamp):
        pnt = self._interpolate(self.location_buffer,timestamp)
        if pnt is not None:
            pos = PositionVector(pnt.x, pnt.y, pnt.z)
            return pos.get_location()
        return None

    def get_attitude(self,timestamp):
        pnt = self._interpolate(self.attitude_buffer,timestamp)
        if pnt is not None:
            return Attitude(pnt.y, pnt.z, pnt.x)
        return None

    def test(self):

        self.put_location(0,Location(37.691163, -122.155766,10), Point3(30,10,2))
        self.put_location(6000000,Location(37.691295, -122.151861,20), Point3(-30,10,3))
        self.put_location(10000000,Location(37.690735, -122.152164,20), Point3(5,-6,-10))
        for i in range(0,10000000,1000000):
            print self.get_location(i).lat , ',',self.get_location(i).lon
        #plot points on http://www.darrinward.com/lat-long/

        self.put_attitude(0,Attitude(30, 60,10), Point3(30,10,2))
        self.put_attitude(6000000,Attitude(30, 80,20), Point3(-30,10,3))
        self.put_attitude(10000000,Attitude(30, 90,20), Point3(5,-6,-10))
        for i in range(0,10000000,1000000):
            print self.get_attitude(i).roll , ',',self.get_attitude(i).pitch



# create global object
veh_control = VehicleControl()

# if this is the parent class connect and run test
if __name__ == "__builtin__":
    veh_control.connect(local_connect())
    veh_control.test()
