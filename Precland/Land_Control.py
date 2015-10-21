
import math
import time
from cv_utils.transforms import *


'''
TODO
Add abort_alt support
Add yaw support
'''


class Land_Control():

    def __init__(self,config, v_controller):
        #objects
        self.config = config
        self.v_controller = v_controller

        #parameters
        self.init_dec_speed = self.config.get_float('land_control', 'init_dec_speed', 1.5)
        self.final_dec_speed = self.config.get_float('land_control', 'final_dec_speed', 0.75)
        self.init_dec_alt = self.config.get_float('land_control' , 'init_dec_alt', 20)
        self.goto_alt = self.config.get_float('land_control' , 'goto_alt', 10)
        self.final_dec_alt = self.config.get_float('land_control' , 'final_dec_alt', 2)
        #self.abort_alt = self.config.get_float('land_control', 'abort_alt', 5)
        self.landed_alt = self.config.get_float('land_control', 'landed_alt', 0.5)
        self.input_scalar = self.config.get_float('land_control', 'input_scalar', 1.3)
        self.update_rate = self.config.get_integer('land_control', 'update_rate', 30)
        self.has_gimbal = self.config.get_integer('camera', 'has_gimbal', False)

        #state variables
        self.angular_offset = None
        self.timestamp = 0
        #self.last_detection_alt = init_dec_alt
        self.land_started = False
        self.last_update_time = 0


    #take in target data
    def consume_target_offset(self,angular_offset, timestamp = 0, alt_above_terrain = None):

        self.timestamp = timestamp
        self.angular_offset = angular_offset
        ret_l, self.location = self.v_controller.get_location(self.timestamp)
        ret_a, self.attitude = self.v_controller.get_attitude(self.timestamp)
        if alt_above_terrain is None:
            self.alt_above_terrain = self.location.alt
        else:
            self.alt_above_terrain = alt_above_terrain

        #if we fail to sync sensors then the reading is invalid
        if self.timestamp != 0 and (not ret_l or not ret_a):
            self.angular_offset = None
            print "sensor sync failed"

    #update the landing controller
    def update(self):
        #constrain update rate
        while self.update_rate != 0 and (time.time() - self.last_update_time < (1.0 / self.update_rate )):
            pass
        self.last_update_time = time.time()

        #get vehicle state
        mode = self.v_controller.get_mode()
        curr_alt = self.v_controller.get_location()[1].alt

        #start land
        if self.land_started == False and (mode=='RTL' or mode=='LAND') and curr_alt < self.init_dec_alt and curr_alt > self.landed_alt and self.angular_offset is not None:
            self.v_controller.set_mode('GUIDED')
            mode = self.v_controller.get_mode()
            self.land_started = True

        #run various stages of landing
        if mode == 'GUIDED' and self.land_started == True:
            #Rough location adjustment(GPS shifting)
            if curr_alt < self.init_dec_alt and curr_alt > self.goto_alt:
                #shift position
                veh_home = self.veh_control.get_home()
                ef_angle_offset = bodyframe_to_earthframe(self.angular_offset, self.attitude, ignore_tilt = self.has_gimbal)
                loc = earthframe_rad_to_relative_copter(ef_angle_to_target, self.location, self.alt_above_terrain)
                loc.alt = curr_alt - 1.0
                self.veh_control.get_vehicle().commands.goto(loc)

            #Fine control(velocity control)
            if curr_alt < self.goto_alt and curr_alt > self.landed_alt:

                #send fine control
                ''' Need to be in LAND mode, not GUIDED mode
                #send raw target angular offsets to autopilot
                if self.operation_mode == 'angular':
                    coord_frame = mavlink.MAV_FRAME_BODY_NED
                    #send commands to autopilot
                    self.veh_control.report_landing_target(self.timestamp,coord_frame,self.angular_offset.x, self.angular_offset.y, alt,0,0)
                '''
                #send absolute target location to autopilot
                if self.operation_mode == 'global':
                    #shift position
                    veh_home = self.veh_control.get_home()
                    ef_angle_offset = bodyframe_to_earthframe(self.angular_offset, self.attitude, ignore_tilt = self.has_gimbal)
                    loc = earthframe_rad_to_relative_copter(ef_angle_to_target, self.location, self.alt_above_terrain)
                    loc.alt = curr_alt - 1.0
                    self.veh_control.get_vehicle().commands.goto(loc)

                elif self.operation_mode == 'velocity':
                    descent_vel = 0
                    #rapid descent
                    if curr_alt > self.final_dec_alt:
                        descent_vel = self.init_dec_speed
                    #final descent
                    else:
                        descent_vel = self.final_dec_speed

                    ef_angle_offset = bodyframe_to_earthframe(self.angular_offset, self.attitude, ignore_tilt = self.has_gimbal)
                    vel = self.calc_vel_vector(ef_angle_offset, self.descent_vel, self.input_scalar)
                    self.v_controller.set_velocity(vel[0],vel[1],vel[2])


            #touchdown
            if curr_alt < self.landed_alt:
                self.v_controller.set_mode('LAND')
                self.land_started = False
                self.angular_offset = None



    def calc_vel_vector(self, angular_offset,descent_vel,scalar = 1.0):

        ef_ang_offset = self.bodyframe_to_earthframe(angular_offset)

        vx = descent_vel * math.sin(ef_ang_offset[0] * scalar)
        vy = descent_vel * math.sin(ef_ang_offset[1] * scalar)
        vz = math.sqrt(descent_vel**2 - vx**2 - vy**2)

        return (vx,vy,vz)
