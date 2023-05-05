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

def main(args):
    
    a = True

def calibrate():
        
    rot_dyn_cal = OfflineRotDynCal(args.data_basepath, 
                                   args.config_path)

if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description='Script to perform rotor dynamics calibration with offline-data')

    parser.add_argument('--data_basepath', '-dpath', type = str, 
                        default = "/media/andreap/super_storage/mhe_bags/MheRt__0_2023_04_18__14_39_37.mat")
    
    parser.add_argument('--config_path', '-cpath', type = str, default = package_path + "/config/jnt_mhe_opt_concert.yaml")

    parser.add_argument('--verbose', '-v', type = str2bool, \
                        help='', default = False)
    
    args = parser.parse_args()

    calibrate()

    main(args)