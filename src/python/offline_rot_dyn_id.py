#!/usr/bin/env python3
import argparse

import rospkg

from utilities.offline_rot_dyn_cal import OfflineRotDynCal
from utilities.offline_rot_dyn_cal import str2bool

import h5py
from horizon.utils import mat_storer

from termcolor import colored
import os
import shutil
from datetime import datetime
from datetime import date

import yaml

import time

file_name = os.path.splitext(os.path.basename(__file__))[0]
file_name.replace(".py", "")

unique_id = date.today().strftime("%d-%m-%Y") + "-" + \
            datetime.now().strftime("%H_%M_%S")

pretakeoff_opt_name = "pretakeoff"
apex_opt_name = "apex"
full_opt_name = "full"

sol_mat_basename = "awesome_jump"

rospack = rospkg.RosPack()
package_path = rospack.get_path("moving_horizon_jnt_calib")

def calibrate():
        
    rot_dyn_cal = OfflineRotDynCal(args.data_basepath, 
                                   args.config_path, 
                                   args.use_prev_sol_reg)

    for i in range(0, 1):
        
        rot_dyn_cal.fill_window_from_sample(i)

        rot_dyn_cal.run_calibration()

        rot_dyn_cal.retrieve_solution_data()

    print("Kd0: " + str(rot_dyn_cal.kd0_opt))
    print("Kd1: " + str(rot_dyn_cal.kd1_opt))
    print("Kt: " + str(rot_dyn_cal.kt_opt))
    print("rot_MoI: " + str(rot_dyn_cal.rot_moi_opt))

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description='Script to perform rotor dynamics calibration with offline-data')

    parser.add_argument('--data_basepath', '-dpath', type = str, 
                        default = "/media/andreap/super_storage/mhe_bags/MheRt__0_2023_04_18__14_59_55.mat")
    
    parser.add_argument('--config_path', '-cpath', type = str, default = package_path + "/config/jnt_mhe_opt_concert.yaml")

    parser.add_argument('--verbose', '-v', type = str2bool, \
                        help='', default = False)
    
    parser.add_argument('--use_prev_sol_reg', '-sol_ig', type = str2bool, \
                        help='', default = False)
    
    args = parser.parse_args()

    calibrate()
