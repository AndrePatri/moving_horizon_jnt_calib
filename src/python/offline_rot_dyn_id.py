# Copyright (C) 2023  Andrea Patrizi (AndrePatri, andreapatrizi1b6e6@gmail.com)
# 
# This file is part of moving_horizon_jnt_calib and distributed under the General Public License version 2 license.
# 
# moving_horizon_jnt_calib is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
# 
# moving_horizon_jnt_calib is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with moving_horizon_jnt_calib.  If not, see <http://www.gnu.org/licenses/>.
# 
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
    
    overwrite_window_lentgh = False

    if (args.window_l != -1):
        
        overwrite_window_lentgh = True

    rot_dyn_cal = OfflineRotDynCal(args.data_basepath, 
                                   args.config_path, 
                                   verbose = args.verbose,
                                   use_prev_sol_reg = args.use_prev_sol_reg, 
                                   overwrite_window_lentgh = overwrite_window_lentgh, 
                                   window_length = args.window_l)
        
    rot_dyn_cal.fill_window_from_sample(0)

    rot_dyn_cal.run_calibration()

    rot_dyn_cal.add_solution_data()

    rot_dyn_cal.make_plots()

    print("regression error: " + str(rot_dyn_cal.calibrator.get_regr_error()))


    print("Kd0: " + str(rot_dyn_cal.kd0_opt))
    print("Kd1: " + str(rot_dyn_cal.kd1_opt))
    print("Kt: " + str(rot_dyn_cal.kt_opt))
    print("rot_MoI: " + str(rot_dyn_cal.rot_moi_opt))


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description='Script to perform rotor dynamics calibration with offline-data')

    parser.add_argument('--data_basepath', '-dpath', type = str, 
                        default = "/home//apatrizi/Desktop/mhe_bags/MheRt__0_2023_04_18__14_59_55.mat")
    
    parser.add_argument('--config_path', '-cpath', type = str, default = package_path + "/config/jnt_mhe_opt_concert.yaml")

    parser.add_argument('--verbose', '-v', type = str2bool, \
                        help='', default = False)
    
    parser.add_argument('--use_prev_sol_reg', '-sol_ig', type = str2bool, \
                        help='', default = False)

    parser.add_argument('--window_l', '-w', type = int, \
                        help='', default = -1)
    
    args = parser.parse_args()
    
    calibrate()
