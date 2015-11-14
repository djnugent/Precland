
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
        self.goto_distance = self.config.get_float('land_control' , 'goto_distance', 2.5)
        self.final_dec_alt = self.config.get_float('land_control' , 'final_dec_alt', 2)
        #self.abort_alt = self.config.get_float('land_control', 'abort_alt', 5)
        self.landed_alt = self.config.get_float('land_control', 'landed_alt', 0.5)
        self.input_scalar = self.config.get_float('land_control', 'input_scalar', 1.0)
        self.update_rate = self.config.get_integer('land_control', 'update_rate', 30)
        self.operation_mode = self.config.get_string('land_control', 'operation_mode', 'velocity')
        self.has_gimbal = self.config.get_boolean('camera', 'has_gimbal', False)


        #state variables
        self.angular_offset = None
        self.timestamp = 0
        #self.last_detection_alt = init_dec_alt
        self.land_started = False
        self.last_update_time = 0


    #take in target data
    def consume_target_offset(self,angular_offset, timestamp = 0, alt_above_terrain = None):
        mode = self.v_controller.get_mode()
        if mode == 'LAND' or mode == 'RTL' or mode == 'GUIDED':
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
            time.sleep(2.0/self.update_rate)
        self.last_update_time = time.time()

        #get vehicle state
        mode = self.v_controller.get_mode()
        ret,curr_location = self.v_controller.get_location()
        curr_alt = curr_location.alt

        #start land
        if self.land_started == False and (mode=='RTL' or mode=='LAND') and curr_alt < self.init_dec_alt and curr_alt > self.landed_alt and self.angular_offset is not None:
            self.v_controller.set_mode('GUIDED')
            mode = self.v_controller.get_mode()
            self.land_started = True

        #run various stages of landing
        if mode == 'GUIDED' and self.land_started == True:

            #calculate earth frame angular offset to target
            ef_angular_offset = bodyframe_to_earthframe(self.angular_offset, self.attitude, ignore_tilt = self.has_gimbal)

            #calculate XY distance to target
            target_loc_global = PositionVector().get_from_location(earthframe_rad_to_global(ef_angular_offset, self.location, self.alt_above_terrain))
            veh_loc_global = PositionVector().get_from_location(curr_location)
            dist_to_target = PositionVector().get_distance_xy(target_loc_global,veh_loc_global)

            #Rough location adjustment(GPS shifting) when far from target
            if curr_alt > self.final_dec_alt and dist_to_target > self.goto_distance:
                #shift position
                veh_home = self.v_controller.get_home()
                target_loc_global = earthframe_rad_to_global(ef_angular_offset, self.location, self.alt_above_terrain)
                target_loc_global.alt = curr_alt - 1
                self.v_controller.get_vehicle().commands.goto(target_loc_global)
                print "Sent goto dist {0}".format(round(dist_to_target,2))

            #Fine control(velocity control)
            elif curr_alt > self.landed_alt:

                #send fine control
                '''Need to be in LAND mode, not GUIDED mode
                #send raw target angular offsets to autopilot
                if self.operation_mode == 'angular':
                    coord_frame = mavlink.MAV_FRAME_BODY_NED
                    #send commands to autopilot
                    self.v_controller.report_landing_target(self.timestamp,coord_frame,self.angular_offset.x, self.angular_offset.y, alt,0,0)
                '''
                #send absolute target location to autopilot
                if self.operation_mode == 'global':
                    #shift position
                    veh_home = self.v_controller.get_home()
                    target_loc_global = earthframe_rad_to_global(ef_angular_offset, self.location, self.alt_above_terrain)
                    target_loc_global.alt = curr_alt - 0.25
                    self.v_controller.get_vehicle().commands.goto(target_loc_global)

                elif self.operation_mode == 'velocity':
                    descent_vel = 0
                    #rapid descent
                    if curr_alt > self.final_dec_alt:
                        descent_vel = self.init_dec_speed
                    #final descent
                    else:
                        descent_vel = self.final_dec_speed

                    vel = self.calc_vel_vector(ef_angular_offset, descent_vel, self.input_scalar)
                    self.v_controller.set_velocity(vel[0],vel[1],vel[2])
                    print "Sent Vel Mag: {0}, Vx: {1}, Vy: {2}, Vx:{3}".format(round(descent_vel,2),round(vel[0],2),round(vel[1],2),round(vel[2],2))



            #touchdown
            elif curr_alt < self.landed_alt:
                print "touchdown"
                self.v_controller.set_mode('LAND')
                self.land_started = False
                self.angular_offset = None



    def calc_vel_vector(self, angular_offset,descent_vel,scalar = 1.0):

        vx = descent_vel * math.sin(angular_offset.x * scalar)
        vy = descent_vel * math.sin(angular_offset.y * scalar)
        vz = math.sqrt(descent_vel**2 - vx**2 - vy**2)

        return (vx,vy,vz)
