
import numpy as np
import sys
import os
import inspect
#Add script directory to path
script_dir =  os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) # script directory
script_dir = script_dir.replace('Tools','')
sys.path.append(script_dir)

from Common.ImageRW import *
from PrecisionLand_lib.PL_gui import PrecisionLandGUI as gui


import argparse
#parse arguments
parser = argparse.ArgumentParser(description="Run Ring_Detector")
#optional arguments
parser.add_argument('input', action="store", help='use a video filename as an input instead of a webcam')
args, unknown = parser.parse_known_args()


#video source
cam = ImageReader(args.input)

file_count = len([f for f in os.walk(args.input).next()[2] if f[-4:] == ".bmp"])
key = np.empty((file_count),object)
print "total frames", file_count


frame_id = 0
if cam is not None and cam.isOpened():
    while True:
        ret, img = cam.peek(frame_id)
        if(img is not None and ret == True):

            img = gui.add_stats(img, str(frame_id) + '/' + str(file_count),10,10)
            cv2.imshow('Image', img)
            k = cv2.waitKey(0)

            if k == 65362: #upkey
                key[frame_id] = 1.0
                frame_id += 1
                print frame_id, "Present"
            elif k == 65364: #down key
                key[frame_id] = 0.0
                frame_id += 1
                print frame_id, "Not present"
            elif k == 65363: #right key
                key[frame_id] = 0.5
                frame_id += 1
                print frame_id, "Partial"
            elif k == 65361: #left key
                frame_id -= 1
                print frame_id, "Go back"
            elif k == 27: #esc key
                print "exit key"
                break
            else:
                print "Invalid key"

        else:
            print "End of feed"
            break

    #save key data
    key_file = args.input.replace('gui','raw') + "/key.txt"

    f = open(key_file, "wb")
    for i in key:
        f.write(str(i) + "\n")
    # Close opened file
    f.close()
    print "key saved at {0}".format(key_file)
else:
    print "No video source detected"
