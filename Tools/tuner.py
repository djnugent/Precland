
import numpy as np
import sys
import os
import inspect
import subprocess
#Add script directory to path
script_dir =  os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) # script directory
script_dir = script_dir.replace('Tools','')
sys.path.append(script_dir)

#global score variables
global_tot_valid_targets = 0 #total number of valid targets in the key
global_tot_partial_targets = 0 #total number of partial targets in the key
global_tot_no_targets = 0 #total number of no targets in the key
global_detected_target = 0 #detected a valid target
global_detected_target_with_orient = 0 #detected a targets location and orientation
global_false_positives = 0 #detects nothing as a target
global_false_positives_with_orient = 0 #detects location and orientation of nothing
global_detected_partial = 0 #detects a partial target
global_detected_partial_with_orient = 0 #detected a partial targets locationand orientation

def score_clip(file_name,config_file = "Smart_Camera.cnf"):
    score = "["+ file_name+ "]\n"

    #run ring detector
    p = subprocess.Popen('python /home/daniel/visnav/PrecisionLand_lib/Ring_Detector.py -i ' + file_name + ' -f ' + config_file + ' -w True', shell = True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate()
    #open key file
    key = open(file_name + 'key.txt', "r").read()
    #split lines
    out_lines = out.split('\n')
    key_lines = key.split('\n')
    #clean up output and check for validity
    if out_lines[0] == "No video source detected":
        score += "No video source detected"
        return score
    out_lines.remove('failed to grab image')
    out_lines.remove('') #remove last line
    key_lines.remove('') #remove last line

    if len(key_lines) != len(out_lines):
        score += "Invalid key length\n"
        score += "key {0}, out {1}".format(len(key_lines),len(out_lines))
        return score

    #score variables
    tot_valid_targets = 0 #total number of valid targets in the key
    tot_partial_targets = 0 #total number of partial targets in the key
    tot_no_targets = 0 #total number of no targets in the key
    detected_target = 0 #detected a valid target
    detected_target_with_orient = 0 #detected a targets location and orientation
    false_positives = 0 #detects nothing as a target
    false_positives_with_orient = 0 #detects location and orientation of nothing
    detected_partial = 0 #detects a partial target
    detected_partial_with_orient = 0 #detected a partial targets locationand orientation

    for i in range(0,len(key_lines)):
        num = float(key_lines[i])
        line = out_lines[i].split(',')
        if num == 1.0:
            tot_valid_targets += 1
            if line[0].find('0') == -1: #found ring
                detected_target += 1
                if line[2].find('None') == -1: #valid orientation
                    detected_target_with_orient +=1
            else:
                print "failed image #", i


        elif num == 0.5:
            tot_partial_targets += 1
            if line[0].find('0') == -1: #found ring
                detected_partial += 1
                if line[2].find('None') == -1: #valid orientation
                    detected_partial_with_orient +=1

        elif num == 0.0:
            tot_no_targets += 1
            if line[0].find('0') == -1: #found ring
                false_positives += 1
                if line[2].find('None') == -1: #valid orientation
                    false_positives_with_orient +=1



    score += " Valid targets detected               {2}% -- {0}/{1}\n".format(detected_target,tot_valid_targets, 100 * detected_target/tot_valid_targets)
    score += " Targets with orientaion              {2}% -- {0}/{1}\n".format(detected_target_with_orient,tot_valid_targets, 100 * detected_target_with_orient/tot_valid_targets)
    score += " Partial targets detected             {2}% -- {0}/{1}\n".format(detected_partial,tot_partial_targets, 100 * detected_partial/tot_partial_targets)
    score += " Partials with orientaion             {2}% -- {0}/{1}\n".format(detected_partial_with_orient,tot_partial_targets, 100 * detected_partial_with_orient/tot_partial_targets)
    score += " False positives                      {2}% -- {0}/{1}\n".format(false_positives,tot_no_targets, 100 * false_positives/tot_no_targets)
    score += " False positives with orientaion      {2}% -- {0}/{1}\n".format(false_positives_with_orient,tot_no_targets, 100 * false_positives_with_orient/tot_no_targets)

    #add score to global count
    global global_tot_valid_targets, global_tot_partial_targets, global_tot_no_targets
    global global_detected_target, global_detected_target_with_orient,global_false_positives
    global global_false_positives_with_orient, global_detected_partial, global_detected_partial_with_orient
    global_tot_valid_targets += tot_valid_targets #total number of valid targets in the key
    global_tot_partial_targets += tot_partial_targets #total number of partial targets in the key
    global_tot_no_targets += tot_no_targets #total number of no targets in the key
    global_detected_target +=  detected_target#detected a valid target
    global_detected_target_with_orient +=  detected_target_with_orient#detected a targets location and orientation
    global_false_positives += false_positives #detects nothing as a target
    global_false_positives_with_orient += false_positives_with_orient #detects location and orientation of nothing
    global_detected_partial += detected_partial #detects a partial target
    global_detected_partial_with_orient += detected_partial_with_orient #detected a partial targets locationand orientation
    return score


def score_batch():
    score = "[Summary]\n"
    score += " Valid targets detected               {2}% -- {0}/{1}\n".format(global_detected_target,global_tot_valid_targets, 100 * global_detected_target/global_tot_valid_targets)
    score += " Targets with orientaion              {2}% -- {0}/{1}\n".format(global_detected_target_with_orient,global_tot_valid_targets, 100 * global_detected_target_with_orient/global_tot_valid_targets)
    score += " Partial targets detected             {2}% -- {0}/{1}\n".format(global_detected_partial,global_tot_partial_targets, 100 * global_detected_partial/global_tot_partial_targets)
    score += " Partials with orientaion             {2}% -- {0}/{1}\n".format(global_detected_partial_with_orient,global_tot_partial_targets, 100 * global_detected_partial_with_orient/global_tot_partial_targets)
    score += " False positives                      {2}% -- {0}/{1}\n".format(global_false_positives,global_tot_no_targets, 100 * global_false_positives/global_tot_no_targets)
    score += " False positives with orientaion      {2}% -- {0}/{1}\n".format(global_false_positives_with_orient,global_tot_no_targets, 100 * global_false_positives_with_orient/global_tot_no_targets)


    return score




def iter_next(level = 0):
    global params, iter_state
    #works like a mechical counter, except each digit/level has a different base
    param = params[level]
    bottom = param['bottom']
    top = param['top']
    step = param['step']

    steps = (top - bottom) / step
    if iter_state[level] == steps - 1:
        if level == len(iter_state - 1): #at highest level
            return False
        iter_state[level] = 0
        iter_next(level + 1)
    else:
        iter_state[level] += 1

    return True


def iter_get():
    global params, iter_state
    result = np.empty(len(params),object)
    for i in range(0,len(params)):
        param = params[i]
        index = iter_state[i]

        step,top,bottom,name,parent = param.values()

        value = step * index + bottom
        result[i] = {'parent':parent, 'name':name, 'value':value}

    return result



def set_config(index, config):
    os.mkdir(res_dir + '/config' + str(index))
    file_name = res_dir + '/config' + str(index)+ '/Smart_Camera.cnf'




def run_config(index):

    config_file = res_dir + '/config' + str(index)+ '/Smart_Camera.cnf'
    config_file = config_file.replace('/home/daniel', '')

    #run configuration on every file
    detailed = ''
    for f in files:
        detailed += score_clip(f,config_file)
    summary = score_batch()

    #write results
    f.open(res_dir + '/config' + str(index) + '/results.txt', wb)
    f.write(summary)
    f.write(detailed)
    f.close()


##############################################     MAIN    ############################################

import argparse
#parse arguments
parser = argparse.ArgumentParser(description="Run tuner")
#optional arguments
parser.add_argument('dir', action="store", help='directory to store results')
args, unknown = parser.parse_known_args()

res_dir = args.dir


best_config = 0
best_result = 0


files =["/home/daniel/Videos/flight_0/Smart_Camera-raw-4/",
        "/home/daniel/Videos/flight_1/Smart_Camera-raw-0/",
        "/home/daniel/Videos/flight_1/Smart_Camera-raw-1/",
        "/home/daniel/Videos/flight_1/Smart_Camera-raw-2/",
        "/home/daniel/Videos/flight_1/Smart_Camera-raw-3/",
        "/home/daniel/Videos/flight_1/Smart_Camera-raw-4/",
        "/home/daniel/Videos/flight_2/Smart_Camera-raw-0/",
        "/home/daniel/Videos/flight_3/Smart_Camera-raw-1/",
        "/home/daniel/Videos/flight_3/Smart_Camera-raw-2/"
        ]

params = np.array([dict(parent= 'algorithm', name= 'eccentricity', bottom= 0.4, top= 0.9, step=0.1),
                   dict(parent= 'algorithm', name= 'min_radius', bottom= 0, top= 10, step=2),
                   dict(parent= 'algorithm', name= 'area_ratio', bottom= 0.5, top= 0.9, step=0.1),
                   dict(parent= 'algorithm', name= 'min_ring_ratio', bottom= 0.45, top= 0.75, step=0.1),
                   dict(parent= 'algorithm', name= 'max_ring_ratio', bottom= 0.8, top= 1.0, step=0.1),
                   dict(parent= 'algorithm', name= 'min_range', bottom= 0, top= 50, step=5)], object)


iter_state = np.zeros(len(params))


detailed = ''
for f in files:
    detailed += score_clip(f)
summary = score_batch()

print detailed
print summary

'''
#calculate number of combinations
num_combinations = 1
for param in params:
    step,bottom,top,parent,name = param.values()
    print parent, name, bottom, top, step
    num_combinations *= (top - bottom + 1) / step
    print num_combinations

print num_combinations, "configurations"


#run every combination of configuration
for i in range(0,num_combintations):
    print "Running configuration", i
    config = iter_get()
    set_config(i, config)


    #run configuration on every file
    run_config(i)
    detailed = ''
    for f in files:
        detailed += score_clip(f)
    summary = score_batch()

    #write results
    f.open(res_dir + '/config' + str(i) + '.txt', wb)
    f.write(summary)
    f.write(detailed)
    f.close()

    #iterate configuration
    iter_next()
'''
