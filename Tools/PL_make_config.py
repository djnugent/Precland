#!/usr/bin/python
#SYSTEM IMPORTS
import numpy as np

#COMMOM IMPORTS
from VisNav.Common.VN_config import VN_config



#####ALGORITHM#######

#how round a circle needs to be. Perfect circle = 1
sc_config.config.set_float('algorithm', 'eccentricity', 0.6)
#acceptable distance(pixels) between cocentric circle centers
sc_config.config.set_integer('algorithm','distance_threshold', 15)
# number of circles needed for a valid target(times 2); 2 circles are often overlayed
sc_config.config.set_integer('algorithm','min_circles',5)
#pixels: used to identify repeat circles(stacked circles). Problem caused by findContours()
sc_config.config.set_integer('algorithm', 'radius_tolerance', 2)
#Tolerance used in comparing actaul ratios and preceived ratios 
sc_config.config.set_float('algorithm', 'ratio_tolerance', 0.015)


#target specific data
#target_code is the unique ratio between rings
target_code_def = np.array([0.8,0.91,0.76,0.84,0.7,0.66,0.49])
sc_config.config.set_array('algorithm', 'target_code',target_code_def)
#the outer_ring is a scaling factor for targets of various sizes; radius of outer ring in meters
sc_config.config.set_float('algorithm', 'outer_ring', 0.08255)




####PROCESSING#######

sc_config.config.set_integer('processing', 'desired_cores', 4)
#check if a core is already in use for background image capture
sc_config.config.set_boolean('processing','background_capture', True)




#####LOGGING######
sc_config.config.set_string('logging','location','~/SmartCamera/')

#levels = 'debug' , 'general' , 'aircraft' , 'algorithm' , ' performance'
#multiple message levels can be selected by concatination strings i.e. 'debug, aircraft' 
#what type of messages we print to the terminal
sc_config.config.set_string('logging','print_level','debug, general')
#what type of messages we log to a file
sc_config.config.set_string('logging','log_level','aircraft , algorithm, general')

sc_config.config.set_string('logging', 'display_level', 'raw, gui')
#what type of images we record
sc_config.config.set_string('logging', 'record_level', 'raw')




#######CAMERA######

# get which camera we will use
sc_config.config.set_integer('camera','index',0)
# get image resolution
sc_config.config.set_integer('camera','width',640)
sc_config.config.set_integer('camera','height',480)


# define field of view
sc_config.config.set_float('camera','horizontal-fov',70.42)
sc_config.config.set_float('camera','vertical-fov',43.3)



#get camera distortion matrix and intrinsics. Defaults: logitech c920
mtx = np.array([[ 614.01269552,0,315.00073982],
        [0,614.43556296,237.14926858],
        [0,0,1.0]])
dist = np.array([0.12269303, -0.26618881,0.00129035, 0.00081791,0.17005303])

sc_config.config.set_array('camera','matrix', mtx)
sc_config.config.set_array('camera', 'distortion', dist)



sc_config.config.save()