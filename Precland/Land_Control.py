
import math
import time
from cv_utils.helpers import *


class Land_Control():

    def __init__(self,v_controller, rapid_speed = 1.0, slow_speed = 0.25,alt_thres = 5):
        self.v_controller = v_controller
        self.rapid_speed = rapid_speed
        self.slow_speed = slow_speed
        self.alt_thres = alt_thres
        self.decent_vel = 0
        self.angular_offset = None
        self.timestamp = 0
        self.scalar = 1.3
        self.first_detect = 1


    #take in earth-frame target data
    def consume_target_offset(self,angular_offset, timestamp = 0):

        self.timestamp = timestamp
        self.angular_offset = angular_offset

        mode = self.v_controller.get_mode()
        if(mode == 'RTL' or mode == 'LAND'):
            mode = 'GUIDED'
            self.v_controller.set_mode(mode)

        self.update()


    def update(self):
        mode = self.v_controller.get_mode()
        if(mode == 'GUIDED' and self.angular_offset is not None):
            self.run_state_machine()


    def run_state_machine(self):
        location = self.v_controller.get_location(self.timestamp)
        attitude = self.v_controller.get_attitude(self.timestamp)

        if(location.alt > self.alt_thres): #fast decent
            self.decent_vel =  self.rapid_speed
        else:                #slow decent
            self.decent_vel = self.slow_speed

        if(location.alt <=0.4):
            self.v_controller.set_mode('LAND')

        vel = self.calc_vel_vector(self.angular_offset)
        self.v_controller.set_velocity(vel[0],vel[1],vel[2])


    def bodyframe_to_earthframe(self,angular_offset):


        attitude = self.v_controller.get_attitude(self.timestamp)
        '''
        x_rad = angular_offset.x - attitude.roll #radians
    	y_rad = -angular_offset.y + attitude.pitch
        '''

        x_rad, y_rad = angular_offset.x, -angular_offset.y

    	#rotate to earth-frame angles
    	x_ef = y_rad*math.cos(attitude.yaw) - x_rad*math.sin(attitude.yaw) #radians
    	y_ef = y_rad*math.sin(attitude.yaw) + x_rad*math.cos(attitude.yaw)
    	ef_angle_to_target = (x_ef,y_ef)

        return ef_angle_to_target

    def calc_vel_vector(self, angular_offset):

        ef_ang_offset = self.bodyframe_to_earthframe(angular_offset)

        vx = self.decent_vel * math.sin(ef_ang_offset[0] * self.scalar)
        vy = self.decent_vel * math.sin(ef_ang_offset[1] * self.scalar)
        vz = math.sqrt(self.decent_vel**2 - vx**2 - vy**2)

        return (vx,vy,vz)
