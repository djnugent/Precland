
import math
import time
from cv_utils.transforms import *
from cv_utils.dataTypes import *


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
        self.init_dec_speed = self.config.get_float('land_control', 'init_dec_speed', 1.5) #m/s
        self.final_dec_speed = self.config.get_float('land_control', 'final_dec_speed', 0.75) #m/s
        self.goto_alt_step = self.config.get_float('land_control','goto_alt_step',1.0) #meters
        self.init_dec_alt = self.config.get_float('land_control' , 'init_dec_alt', 20) #meters
        self.goto_distance = self.config.get_float('land_control' , 'goto_distance', 2.5) #meters
        self.final_dec_alt = self.config.get_float('land_control' , 'final_dec_alt', 2) #meters
        #self.abort_alt = self.config.get_float('land_control', 'abort_alt', 5)
        self.landed_alt = self.config.get_float('land_control', 'landed_alt', 0.5) #meters
        self.horz_scalar = self.config.get_float('land_control', 'horz_scalar', 1.0) #scalar 0.5 - 3.0
        self.vert_scalar = self.config.get_float('land_control', 'vert_scalar', 0.0) #scalar 0.0 - 5.0
        self.horz_offset = self.config.get_float('land_control', 'horz_offset', 0.0) #degrees -15.0 - 15.0
        self.vert_offset = self.config.get_float('land_control', 'vert_offset', 0.0) #degrees -15.0 - 15.0
        self.update_rate = self.config.get_integer('land_control', 'update_rate', 12) #range 0.0 - 30.0
        self.operation_mode = self.config.get_string('land_control', 'operation_mode', 'velocity')
        self.has_gimbal = self.config.get_boolean('camera', 'has_gimbal', False)


        #state variables
        self.angular_offset = None
        self.timestamp = 0
        self.land_started = False
        self.last_update_time = 0


    #take in target data
    def consume_target_offset(self,angular_offset, timestamp = 0, alt_above_terrain = None):
        mode = self.v_controller.get_mode()
        #only accept sensor data if we are in a landing mode and our gimbal is pointed down
        if (mode == 'LAND' or mode == 'RTL' or mode == "GUIDED") and self.v_controller.vehicle.mount_status[0] < -89.0:
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

            '''
            #rotate copter to match target orientation
            if angular_offset is not None:
                self.v_controller.set_gimbal(angular_offset.z,-90)
            '''
        else:
            self.angular_offset = None
            self.land_started = False

    #update the landing controller
    def update(self):
        #constrain update rate
        while self.update_rate != 0 and (time.time() - self.last_update_time < (1.0 / self.update_rate )):
            time.sleep(2.0/self.update_rate)
        self.last_update_time = time.time()

        #get vehicle state
        #mode
        mode = self.v_controller.get_mode()
        #location
        veh_curr_location_meters = PositionVector().get_from_location(self.v_controller.get_location()[1])
        curr_alt = veh_curr_location_meters.z
        #home
        home_loc = PositionVector().get_from_location(self.v_controller.get_home(True))
        dist_to_home = PositionVector().get_distance_xy(home_loc,veh_curr_location_meters)


        #tilt gimbal down and fix heading if we are landing and are close to home
        if (mode=='RTL' or mode=='LAND' or mode=='GUIDED') and dist_to_home < 10:
            self.v_controller.set_gimbal(0,-90)
        else:
            self.v_controller.release_gimbal()

        #start precland
        if self.land_started == False and (mode=='RTL' or mode=='LAND') and curr_alt < self.init_dec_alt and curr_alt > self.landed_alt and self.angular_offset is not None:
            self.v_controller.set_mode('GUIDED')
            mode = self.v_controller.get_mode()
            self.land_started = True
            self.yaw = 0

        #run various stages of landing
        if mode == 'GUIDED' and self.land_started == True:

            #calculate earth frame angular offset to target
            ef_angular_offset = bodyframe_to_earthframe(self.angular_offset, self.attitude, ignore_tilt = self.has_gimbal)

            #calculate XY distance to target
            target_loc_global = earthframe_rad_to_global(ef_angular_offset, self.location, self.alt_above_terrain)
            target_loc_meters = PositionVector().get_from_location(target_loc_global)
            dist_to_target = PositionVector().get_distance_xy(target_loc_meters,veh_curr_location_meters)

            #Rough location adjustment(GPS shifting) when far from target
            if curr_alt > self.final_dec_alt:
                #calculate earth frame angular offset to target
                ef_angular_offset = bodyframe_to_earthframe(self.angular_offset, self.attitude, ignore_tilt = self.has_gimbal)
                #shift position
                target_loc_global.alt = curr_alt
                if dist_to_target < self.goto_distance:
                    target_loc_global.alt -= self.goto_alt_step

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
                    target_loc_global = earthframe_rad_to_global(ef_angular_offset, self.location, self.alt_above_terrain)
                    target_loc_global.alt = curr_alt
                    self.v_controller.get_vehicle().commands.goto(target_loc_global)

                elif self.operation_mode == 'velocity':
                    descent_vel = 0
                    #rapid descent
                    if curr_alt > self.final_dec_alt:
                        descent_vel = self.init_dec_speed
                    #final descent
                    else:
                        descent_vel = self.final_dec_speed

                    #shift target data with constant offset
                    angular_offset_shifted = self.angular_offset + Point3(math.radians(self.horz_offset), math.radians(self.vert_offset),0)

                    #calculate earth frame angular offset to target
                    ef_angular_offset = bodyframe_to_earthframe(angular_offset_shifted, self.attitude, ignore_tilt = self.has_gimbal)

                    vel = self.calc_vel_vector(ef_angular_offset, descent_vel, self.horz_scalar,self.vert_scalar)
                    self.v_controller.set_velocity(vel[0],vel[1],vel[2])
                    print "Sent Vel Mag: {0}, Vx: {1}, Vy: {2}, Vz:{3}".format(round(descent_vel,2),round(vel[0],2),round(vel[1],2),round(vel[2],2))


            #touchdown
            elif curr_alt < self.landed_alt:
                print "touchdown"
                self.v_controller.set_mode('LAND')
                self.land_started = False
                self.angular_offset = None



    def calc_vel_vector(self, angular_offset,descent_vel, horz_scalar = 1.0,vert_scalar =1.0):

        vx = descent_vel * math.sin(angular_offset.x*horz_scalar)
        vy = descent_vel * math.sin(angular_offset.y*horz_scalar)
        mag_offset = math.sqrt(angular_offset.x**2 + angular_offset.y**2)
        vz = math.sqrt(descent_vel**2 - vx**2 - vy**2) * max(1-(mag_offset*vert_scalar),0)

        return (vx,vy,vz)
